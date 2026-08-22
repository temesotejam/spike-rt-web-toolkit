import {
  connectSpikeSerial,
  formatUsbId,
  isWebSerialAvailable,
  SPIKE_RT_SERIAL_BAUD_RATE,
} from "./serial.js";

const elements = {
  browserStatus: document.querySelector("#serial-browser-status"),
  deviceStatus: document.querySelector("#serial-device-status"),
  baudRate: document.querySelector("#serial-baud-rate"),
  connect: document.querySelector("#serial-connect"),
  disconnect: document.querySelector("#serial-disconnect"),
  clear: document.querySelector("#serial-clear"),
  log: document.querySelector("#serial-log"),
};

const MAX_CONSOLE_CHARS = 200000;
let connection = null;
let busy = false;

function timestamp() {
  return new Date().toLocaleTimeString("ja-JP");
}

function trimConsole() {
  if (elements.log.textContent.length <= MAX_CONSOLE_CHARS) return;
  elements.log.textContent =
    "[古いログを省略しました]\n" +
    elements.log.textContent.slice(-Math.floor(MAX_CONSOLE_CHARS * 0.8));
}

function appendNotice(message) {
  if (elements.log.textContent === "デバッグ接続待ち") {
    elements.log.textContent = "";
  }
  if (elements.log.textContent && !elements.log.textContent.endsWith("\n")) {
    elements.log.textContent += "\n";
  }
  elements.log.textContent += `[${timestamp()}] ${message}\n`;
  trimConsole();
  elements.log.scrollTop = elements.log.scrollHeight;
}

function appendSerialData(text) {
  if (!text) return;
  if (elements.log.textContent === "デバッグ接続待ち") {
    elements.log.textContent = "";
  }
  elements.log.textContent += text.replaceAll("\r\n", "\n").replaceAll("\r", "\n");
  trimConsole();
  elements.log.scrollTop = elements.log.scrollHeight;
}

function updateControls() {
  const available = isWebSerialAvailable();
  elements.connect.disabled = busy || !available || Boolean(connection);
  elements.disconnect.disabled = busy || !connection;
  elements.clear.disabled = busy;
}

function setBusy(value) {
  busy = value;
  updateControls();
}

function setDisconnected(message = "未接続") {
  connection = null;
  elements.deviceStatus.textContent = message;
  elements.deviceStatus.className = "";
  updateControls();
}

async function connectSerial() {
  setBusy(true);
  elements.deviceStatus.textContent = "接続中";
  elements.deviceStatus.className = "";

  let opened = null;
  try {
    opened = await connectSpikeSerial({
      onData: appendSerialData,
      onError: (error) => {
        appendNotice(
          `シリアル受信エラー: ${error instanceof Error ? error.message : String(error)}`,
        );
      },
      onDisconnect: () => {
        if (connection === opened) {
          appendNotice("SPIKE-RT USBシリアルが切断されました。");
          setDisconnected("切断済み");
        }
      },
    });
    connection = opened;

    const { usbVendorId, usbProductId } = connection.info;
    elements.deviceStatus.textContent =
      `接続済み (VID=${formatUsbId(usbVendorId)}, PID=${formatUsbId(usbProductId)})`;
    elements.deviceStatus.className = "status-good";
    appendNotice(
      `SPIKE-RT USBシリアルへ接続しました (${SPIKE_RT_SERIAL_BAUD_RATE} bps)。`,
    );
  } catch (error) {
    setDisconnected("接続失敗");
    elements.deviceStatus.className = "status-error";
    appendNotice(error instanceof Error ? error.message : String(error));
  } finally {
    setBusy(false);
  }
}

async function disconnectSerial() {
  if (!connection) return;
  setBusy(true);
  const current = connection;
  try {
    await current.close();
    appendNotice("SPIKE-RT USBシリアルとの接続を解除しました。");
  } catch (error) {
    appendNotice(
      `シリアル切断時の警告: ${error instanceof Error ? error.message : String(error)}`,
    );
  } finally {
    if (connection === current) {
      setDisconnected();
    }
    setBusy(false);
  }
}

function clearConsole() {
  elements.log.textContent = "";
}

elements.connect.addEventListener("click", connectSerial);
elements.disconnect.addEventListener("click", disconnectSerial);
elements.clear.addEventListener("click", clearConsole);
elements.baudRate.textContent = `${SPIKE_RT_SERIAL_BAUD_RATE} bps`;

if (isWebSerialAvailable()) {
  elements.browserStatus.textContent = "利用可能（Chrome / Edge）";
  elements.browserStatus.className = "status-good";
} else {
  elements.browserStatus.textContent = "利用不可";
  elements.browserStatus.className = "status-error";
  appendNotice(
    "Web Serialを利用できません。HTTPS上のChromeまたはEdgeで開いてください。",
  );
}

updateControls();
