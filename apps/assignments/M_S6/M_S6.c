#include <kernel.h>
#include <stdlib.h>
#include <stdint.h>
#include <M_S6.h>
#include "spike/pup/motor.h"      // モータの回転
#include "spike/hub/button.h"     // 左右・中央ボタン
#include "spike/hub/display.h"    // 5×5ドット表示
#include "spike/hub/speaker.h"    // 決定音

// Advanced Driving Base Assembly：左モータA、右モータE、直径88mmタイヤ
#define PI                 3.14159265f
#define WHEEL_DIAMETER_MM  88.0f
#define WHEEL_TRACK_MM     144.0f  // 左右タイヤ中心間距離（実機に合わせて調整）

void Main(intptr_t exinf)
{
    // 1. モータを取得して、前進方向をそろえる（M_S5と同じ）
    pup_motor_t *motorA = pup_motor_get_device(PBIO_PORT_ID_A);
    pup_motor_t *motorE = pup_motor_get_device(PBIO_PORT_ID_E);
    if (motorA == NULL || motorE == NULL) {
        exit(1);
    }
    if (pup_motor_setup(motorA, PUP_DIRECTION_COUNTERCLOCKWISE, true) != PBIO_SUCCESS ||
        pup_motor_setup(motorE, PUP_DIRECTION_CLOCKWISE, true) != PBIO_SUCCESS) {
        exit(1);
    }

    // 2. LCDに表示する円と正方形（5×5の各LEDの明るさ）
    uint8_t circle[25] = {
          0, 100, 100, 100,   0,
        100,   0,   0,   0, 100,
        100,   0,   0,   0, 100,
        100,   0,   0,   0, 100,
          0, 100, 100, 100,   0
    };
    uint8_t square[25] = {
        100, 100, 100, 100, 100,
        100,   0,   0,   0, 100,
        100,   0,   0,   0, 100,
        100,   0,   0,   0, 100,
        100, 100, 100, 100, 100
    };

    // 3. 走行に必要な車輪の回転角度を計算する（単位：度）
    // 円：左タイヤの軌跡が半径250mm、右タイヤはその内側を走る。
    int32_t circleAngle = (int32_t)(360.0f * 2.0f * 250.0f / WHEEL_DIAMETER_MM + 0.5f);
    int32_t innerSpeed = (int32_t)(300.0f * (250.0f - WHEEL_TRACK_MM) / 250.0f);

    // 正方形：左タイヤが500mm直進し、角では左を止めて右だけ後退。
    int32_t straightAngle = (int32_t)(360.0f * 500.0f / (PI * WHEEL_DIAMETER_MM) + 0.5f);
    int32_t turnAngle = (int32_t)(360.0f * WHEEL_TRACK_MM / (2.0f * WHEEL_DIAMETER_MM) + 0.5f);

    hub_button_t buttons = 0;
    int shape = 0;                // 0：円、1：正方形
    int stalled = 0;              // ストール検知で1にするフラグ
    hub_speaker_set_volume(5);

    while (1) {
        // 4. 左右ボタンで図形を切り替え、中央ボタンで決定する。
        hub_display_image(shape == 0 ? circle : square);

        while (1) {
            hub_button_is_pressed(&buttons);

            if (buttons & (HUB_BUTTON_LEFT | HUB_BUTTON_RIGHT)) {
                shape = 1 - shape;  // 円と正方形を切り替える
                hub_display_image(shape == 0 ? circle : square);

                // ボタンを押しっぱなしにしても1回だけ切り替える
                while (buttons & (HUB_BUTTON_LEFT | HUB_BUTTON_RIGHT)) {
                    dly_tsk(20000);
                    hub_button_is_pressed(&buttons);
                }
            }

            if (buttons & HUB_BUTTON_CENTER) {
                // 中央ボタンを離してから走り始める
                while (buttons & HUB_BUTTON_CENTER) {
                    dly_tsk(20000);
                    hub_button_is_pressed(&buttons);
                }
                break;
            }
            dly_tsk(20000);
        }

        // 5. 短い決定音を鳴らす。
        hub_speaker_play_tone(NOTE_C5, 150);

        if (shape == 0) {
            // 6-A. 円：左タイヤを300度/秒、右タイヤをそれより遅く回す。
            // 左タイヤが半径250mmの円を1周したら両方止める。
            pup_motor_reset_count(motorA);  // 左タイヤの回転角度を0に戻す
            pup_motor_set_speed(motorA, 300);
            pup_motor_set_speed(motorE, innerSpeed);

            while (pup_motor_get_count(motorA) < circleAngle) {
                if (pup_motor_is_stalled(motorA) || pup_motor_is_stalled(motorE)) {
                    stalled = 1;
                    break;  // 走行中の左右どちらかがストール
                }
                dly_tsk(10000); // 10msごとに回転角度を確認
            }
            // ストール時はフリー停止。正常終了時だけブレーキ。
            if (stalled) {
                break;
            }
            pup_motor_brake(motorA);
            pup_motor_brake(motorE);

        } else {
            // 6-B. 正方形：500mm直進→90度右折を4回繰り返す。
            for (int side = 0; side < 4; side++) {
                // 500mm直進する
                pup_motor_reset_count(motorA);  // 左タイヤの角度を0に戻す
                pup_motor_set_speed(motorA, 300);
                pup_motor_set_speed(motorE, 300);

                while (pup_motor_get_count(motorA) < straightAngle) {
                    if (pup_motor_is_stalled(motorA) || pup_motor_is_stalled(motorE)) {
                        stalled = 1;
                        break;
                    }
                    dly_tsk(10000);
                }
                if (stalled) {
                    break;  // 以降の辺の走行を飛ばす
                }
                pup_motor_brake(motorA);
                pup_motor_brake(motorE);

                // 左タイヤを止めて、右タイヤだけ後退させて90度曲がる
                dly_tsk(100000);
                pup_motor_reset_count(motorE);  // 右タイヤの角度を0に戻す
                pup_motor_set_speed(motorE, -220);

                while (pup_motor_get_count(motorE) > -turnAngle) {
                    if (pup_motor_is_stalled(motorE)) {
                        stalled = 1;
                        break;
                    }
                    dly_tsk(10000);
                }
                if (stalled) {
                    break;  // 以降の辺の走行を飛ばす
                }
                pup_motor_brake(motorE);
                dly_tsk(100000);
            }
        }

        // 7. ストールしたら選択画面に戻らず終了処理へ進む。
        if (stalled) {
            break;
        }

        // 正常な場合は選択画面に戻ってもう一度試せる。
        pup_motor_brake(motorA);
        pup_motor_brake(motorE);
    }

    // 8. ストール時：両モータをフリー停止して終了
    pup_motor_stop(motorA);
    pup_motor_stop(motorE);
    exit(1);
}
