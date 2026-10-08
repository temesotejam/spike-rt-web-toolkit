#include <kernel.h>                // RTOS（リアルタイムOS）の基本機能
#include <stdlib.h>                // exit() を使うため
#include <t_syslog.h>              // シリアルモニタにメッセージを出す
#include <CS_S2.h>
#include "spike/pup/colorsensor.h" // カラーセンサーを使う

// ──────────────────────────────
// Main関数（RTOSが最初に実行する関数）
// ──────────────────────────────
/* SPIKE-RT v0.2.0 has no pup_color_sensor_color_name().
 * Preserve single-character color output using its supported classified HSV API.
 * Color threshold details should be verified with a physical sensor.
 */
static char pd_color_name(pup_device_t *device)
{
    pup_color_hsv_t c = pup_color_sensor_color(device, true);
    if (c.v < 10) return 'N';
    if (c.s < 20) return 'W';
    if (c.h < 30 || c.h >= 330) return 'R';
    if (c.h < 90) return 'Y';
    if (c.h < 180) return 'G';
    return 'B';
}

void Main(intptr_t exinf)
{
    // 起動メッセージをシリアルモニタに表示
    syslog(LOG_NOTICE, "Program started.");

    // Dポートに接続されたカラーセンサーを取得
    // pup_device_t* センサーを操作するための変数
    pup_device_t *ColorSensor = pup_color_sensor_get_device(PBIO_PORT_ID_D);

    // ──────────────────────────────
    // カラーセンサーが認識した色を定期的に表示
    // ──────────────────────────────
    while (1)
    {
        // 1秒待つ（マイクロ秒単位）
        dly_tsk(1000000);

        // 認識した色の名前を1文字で取得（例：'R'、'G'、'B'など）。
        // ２番目に引数はtrueならLEDがON、falseならLEDがOFF
        char color = pd_color_name(ColorSensor);

        // シリアルモニタに色を表示
        syslog(LOG_NOTICE, "Color: %c", color);
    }

    // 実際にはここには到達しない（無限ループのため）
    exit(0);
}
