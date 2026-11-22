/*
 * USBシリアル出力用の標準サンプルアダプター。
 * 授業配布SPIKE-RTのsample/button/usb_serial.cを使用。
 */
#include <stdbool.h>
#include <stdint.h>

bool pbsys_usb_stdout_put_char(uint8_t c);

void low_putchar(char c)
{
    (void)pbsys_usb_stdout_put_char((uint8_t)c);
}
