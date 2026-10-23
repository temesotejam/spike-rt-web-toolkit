#include <kernel.h>                  // RTOS（リアルタイムOS）の基本機能
#include <stdlib.h>                  // exit() を使うため
#include <t_syslog.h>                // シリアルモニタにメッセージを出す
#include <DC_S2.h>
#include "spike/pup/ultrasonicsensor.h"  // 超音波センサーを使う
#include "spike/hub/light.h"          // 中央ボタンのライトを使う

// ──────────────────────────────
// Main関数（RTOSが最初に実行する関数）
// ──────────────────────────────
void Main(intptr_t exinf)
{
    // 起動時のメッセージをシリアルモニタに出力
    syslog(LOG_NOTICE, "Program started.");

    // Eポートに接続された超音波センサーを取得
    // pup_device_t* はセンサーを操作するための変数
    pup_device_t *UltraSonicSensor = pup_ultrasonic_sensor_get_device(PBIO_PORT_ID_E);
    if (UltraSonicSensor == NULL) {
        syslog(LOG_ERROR, "Ultrasonic sensor not found on port E.");
        exit(1);
    }

    // ──────────────────────────────
    // 距離を測定して表示するループ
    // ──────────────────────────────
    while (1)
    {
        // 距離を取得（単位：ミリメートル）
        int32_t distance = pup_ultrasonic_sensor_distance(UltraSonicSensor);

        // シリアルモニタに距離と判定結果を表示
        syslog(LOG_NOTICE, "Distance: %ld mm", distance);

        if (distance < 0) {
            syslog(LOG_WARNING, "Distance measurement failed.");
            hub_light_off();  // 計測エラーのときはライトを消す
        } else if (distance < 250) {
            syslog(LOG_NOTICE, "Red");
            hub_light_on_color(PBIO_COLOR_RED);
        } else if (distance < 500) {
            syslog(LOG_NOTICE, "Yellow");
            hub_light_on_color(PBIO_COLOR_YELLOW);
        } else {
            syslog(LOG_NOTICE, "Blue");
            hub_light_on_color(PBIO_COLOR_BLUE);
        }

        // 0.01秒（10ミリ秒）待つ（マイクロ秒単位）
        dly_tsk(10000);
    }

    // 実際にはここには到達しない（無限ループのため）
    exit(0);
}
