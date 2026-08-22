export const LEGO_USB_VENDOR_ID = 0x0694;
export const SPIKE_DFU_PRODUCT_ID = 0x0008;
export const SPIKE_RT_SERIAL_BAUD_RATE = 115200;

export function isWebSerialAvailable() {
  return Boolean(window.isSecureContext && navigator.serial);
}

function messageOf(error) {
  return error instanceof Error ? error.message : String(error);
}

export function formatUsbId(value) {
  if (!Number.isInteger(value)) return "—";
  return `0x${value.toString(16).padStart(4, "0")}`;
}

export class SpikeSerialConnection {
  constructor(port, callbacks = {}) {
    this.port = port;
    this.info = port.getInfo();
    this.onData = callbacks.onData ?? (() => {});
    this.onError = callbacks.onError ?? (() => {});
    this.onDisconnect = callbacks.onDisconnect ?? (() => {});
    this.reader = null;
    this.readTask = null;
    this.decoder = new TextDecoder();
    this.closed = false;
    this.closing = false;
  }

  async open() {
    const { usbVendorId, usbProductId } = this.info;

    if (
      Number.isInteger(usbVendorId) &&
      usbVendorId !== LEGO_USB_VENDOR_ID
    ) {
      throw new Error(
        `LEGO USBデバイスではありません (VID=${formatUsbId(usbVendorId)})。`,
      );
    }

    if (usbProductId === SPIKE_DFU_PRODUCT_ID) {
      throw new Error(
        "DFUモードのHubです。書き込み後にSPIKE-RTを起動してからデバッグ接続してください。",
      );
    }

    await this.port.open({ baudRate: SPIKE_RT_SERIAL_BAUD_RATE });
    this.readTask = this.readLoop();
    return this;
  }

  async readLoop() {
    try {
      if (!this.port.readable) {
        throw new Error("USBシリアルの受信ストリームを開けませんでした。");
      }

      this.reader = this.port.readable.getReader();
      while (!this.closed) {
        const { value, done } = await this.reader.read();
        if (done) break;
        if (value?.byteLength) {
          const text = this.decoder.decode(value, { stream: true });
          if (text) this.onData(text);
        }
      }
    } catch (error) {
      if (!this.closed) {
        this.onError(error);
      }
    } finally {
      const tail = this.decoder.decode();
      if (tail) this.onData(tail);

      if (this.reader) {
        try {
          this.reader.releaseLock();
        } catch {
          // The port may already have disappeared from the system.
        }
        this.reader = null;
      }

      if (!this.closed) {
        this.closed = true;
        this.onDisconnect();
      }
    }
  }

  async close() {
    if (this.closing) return;
    this.closing = true;
    this.closed = true;

    try {
      if (this.reader) {
        try {
          await this.reader.cancel();
        } catch {
          // Ignore cancellation errors while a USB device is disappearing.
        }
      }

      if (this.readTask) {
        try {
          await this.readTask;
        } catch {
          // readLoop reports unexpected errors through onError.
        }
      }

      try {
        await this.port.close();
      } catch (error) {
        if (!/disconnected|not open|NetworkError|InvalidStateError/i.test(messageOf(error))) {
          throw error;
        }
      }
    } finally {
      this.closing = false;
    }
  }
}

export async function connectSpikeSerial(callbacks = {}) {
  if (!isWebSerialAvailable()) {
    throw new Error(
      "このブラウザではWeb Serialを利用できません。HTTPS上のChromeまたはEdgeを使用してください。",
    );
  }

  const port = await navigator.serial.requestPort({
    filters: [{ usbVendorId: LEGO_USB_VENDOR_ID }],
  });

  const connection = new SpikeSerialConnection(port, callbacks);
  try {
    return await connection.open();
  } catch (error) {
    try {
      await port.close();
    } catch {
      // Port may not have opened, or may already be disconnected.
    }
    throw new Error(`USBシリアル接続に失敗しました: ${messageOf(error)}`);
  }
}
