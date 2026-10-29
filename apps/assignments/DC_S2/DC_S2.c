#include <kernel.h>                  // RTOS（リアルタイムOS）の基本機能
#include <stdlib.h>                  // exit() を使うため
#include <t_syslog.h>                // シリアルモニタにメッセージを出す
#include <DC_S2.h>
#include "spike/pup/ultrasonicsensor.h"  // 超音波センサーを使う
#include "spike/hub/display.h"        // LCDに1文字ずつ表示する
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

    // 文字列全体の表示を待たず、単一タスクで1文字ずつ更新する。
    const char *labels[] = { "Err", "Red", "Yellow", "Blue" };
    int previous_state = -1;
    unsigned int letter = 0;
    unsigned int display_ticks = 0;

    // ──────────────────────────────
    // 距離を測定して表示するループ
    // ──────────────────────────────
    while (1)
    {
        // 距離を取得（単位：ミリメートル）
        int32_t distance = pup_ultrasonic_sensor_distance(UltraSonicSensor);

        // シリアルモニタに距離と判定結果を表示
        syslog(LOG_NOTICE, "Distance: %ld mm", distance);

        int state;
        if (distance < 0) {
            state = 0;
            syslog(LOG_WARNING, "Distance measurement failed.");
            hub_light_off();  // 計測エラーのときはライトを消す
        } else if (distance < 250) {
            state = 1;
            syslog(LOG_NOTICE, "Red");
            hub_light_on_color(PBIO_COLOR_RED);
        } else if (distance < 500) {
            state = 2;
            syslog(LOG_NOTICE, "Yellow");
            hub_light_on_color(PBIO_COLOR_YELLOW);
        } else {
            state = 3;
            syslog(LOG_NOTICE, "Blue");
            hub_light_on_color(PBIO_COLOR_BLUE);
        }

        // 判定が変わったら、表示途中でも新しい文字列の先頭へ切り替える。
        if (state != previous_state) {
            previous_state = state;
            letter = 0;
            display_ticks = 0;
            hub_display_char(labels[state][letter]);
        } else {
            // 10ms待機を20回で約200ms点灯、5回で約50ms消灯。
            // 表示中・消灯中も毎ループ距離を測定する。
            display_ticks++;
            if (display_ticks == 20) {
                hub_display_off();
            } else if (display_ticks >= 25) {
                letter++;
                if (labels[state][letter] == '\0') {
                    letter = 0;
                }
                hub_display_char(labels[state][letter]);
                display_ticks = 0;
            }
        }

        // 0.01秒（10ミリ秒）待つ（マイクロ秒単位）
        dly_tsk(10000);
    }

    // 実際にはここには到達しない（無限ループのため）
    exit(0);
}
