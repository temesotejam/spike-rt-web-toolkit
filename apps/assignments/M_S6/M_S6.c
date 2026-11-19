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

/*
 * 【直進制御の改善：左右同期＋台形速度プロファイル】
 * 左右のエンコーダ差を10msごとに補正するだけでなく、
 * 走行距離に応じて「低速始動→加速→定速→減速→低速停止」とする。
 *
 * 始動直後の片側だけの動きや左右の始動遅れを抑え、
 * 終了直前の慣性による行き過ぎも小さくすることが狙い。
 *
 * 速度プロファイルは平均エンコーダ角度から決め、
 * 減速開始位置は先行側のエンコーダ角度から決める。
 * さらに左右の角度差に応じて先行側を減速、遅れ側を増速する。
 */
#define STRAIGHT_START_SPEED          120
#define STRAIGHT_CRUISE_SPEED         300
#define STRAIGHT_END_SPEED            120
#define STRAIGHT_MIN_SPEED             60
#define STRAIGHT_MAX_SPEED            330
#define STRAIGHT_SYNC_GAIN              3
#define STRAIGHT_SYNC_MAX_CORRECTION   60
#define STRAIGHT_RAMP_PERCENT          20
#define STRAIGHT_UPDATE_US          10000

static int32_t clamp_straight_speed(int32_t speed)
{
    if (speed < STRAIGHT_MIN_SPEED) return STRAIGHT_MIN_SPEED;
    if (speed > STRAIGHT_MAX_SPEED) return STRAIGHT_MAX_SPEED;
    return speed;
}

/* 走行位置から台形速度プロファイルの基準速度を求める */
static int32_t straight_profile_speed(int32_t countA,
                                      int32_t countE,
                                      int32_t target_angle)
{
    int32_t average_progress = (countA + countE) / 2;
    int32_t leading_progress =
        (countA > countE) ? countA : countE;
    int32_t ramp_distance =
        target_angle * STRAIGHT_RAMP_PERCENT / 100;

    if (ramp_distance < 1) {
        ramp_distance = 1;
    }
    if (average_progress < 0) {
        average_progress = 0;
    }

    /* 始動側：走行距離の最初の20%で120→300°/sへ加速 */
    int32_t acceleration_speed = STRAIGHT_CRUISE_SPEED;
    if (average_progress < ramp_distance) {
        acceleration_speed = STRAIGHT_START_SPEED +
            (STRAIGHT_CRUISE_SPEED - STRAIGHT_START_SPEED) *
            average_progress / ramp_distance;
    }

    /* 終了側：先行輪が残り20%に入ったら300→120°/sへ減速 */
    int32_t remaining = target_angle - leading_progress;
    if (remaining < 0) {
        remaining = 0;
    }

    int32_t deceleration_speed = STRAIGHT_CRUISE_SPEED;
    if (remaining < ramp_distance) {
        deceleration_speed = STRAIGHT_END_SPEED +
            (STRAIGHT_CRUISE_SPEED - STRAIGHT_END_SPEED) *
            remaining / ramp_distance;
    }

    /* 加速区間と減速区間が重なる短い走行距離でも安全な低い方を採用 */
    return (acceleration_speed < deceleration_speed)
        ? acceleration_speed : deceleration_speed;
}

/* 直進成功なら1、ストール検知なら0を返す */
static int drive_straight(pup_motor_t *motorA,
                          pup_motor_t *motorE,
                          int32_t target_angle)
{
    int doneA = 0;
    int doneE = 0;

    pup_motor_reset_count(motorA);
    pup_motor_reset_count(motorE);

    /* 最初から最高速を指令せず、低速で左右の始動差を小さくする */
    pup_motor_set_speed(motorA, STRAIGHT_START_SPEED);
    pup_motor_set_speed(motorE, STRAIGHT_START_SPEED);

    while (!doneA || !doneE) {
        int32_t countA = pup_motor_get_count(motorA);
        int32_t countE = pup_motor_get_count(motorE);

        if ((!doneA && pup_motor_is_stalled(motorA)) ||
            (!doneE && pup_motor_is_stalled(motorE))) {
            pup_motor_stop(motorA);
            pup_motor_stop(motorE);
            return 0;
        }

        /* 片側が目標に達したら、その側だけ先に停止する */
        if (!doneA && countA >= target_angle) {
            pup_motor_brake(motorA);
            doneA = 1;
        }
        if (!doneE && countE >= target_angle) {
            pup_motor_brake(motorE);
            doneE = 1;
        }

        if (doneA && doneE) {
            break;
        }

        {
            /* 走行位置に応じて基準速度を変化させる */
            int32_t base_speed =
                straight_profile_speed(countA, countE, target_angle);

            /* 左右のエンコーダ差を使って始動遅れ・負荷差を補正 */
            int32_t error = countA - countE;
            int32_t correction = error * STRAIGHT_SYNC_GAIN;

            if (correction > STRAIGHT_SYNC_MAX_CORRECTION) {
                correction = STRAIGHT_SYNC_MAX_CORRECTION;
            } else if (correction < -STRAIGHT_SYNC_MAX_CORRECTION) {
                correction = -STRAIGHT_SYNC_MAX_CORRECTION;
            }

            /* Aが先行ならAを遅く、Eを速くする。逆なら反対に補正 */
            int32_t speedA =
                clamp_straight_speed(base_speed - correction);
            int32_t speedE =
                clamp_straight_speed(base_speed + correction);

            if (!doneA) {
                pup_motor_set_speed(motorA, speedA);
            }
            if (!doneE) {
                pup_motor_set_speed(motorE, speedE);
            }
        }

        dly_tsk(STRAIGHT_UPDATE_US);
    }

    pup_motor_brake(motorA);
    pup_motor_brake(motorE);
    return 1;
}

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
                // 【改善】左右のエンコーダ差を補正し、両輪が目標角度に達してから次へ進む
                if (!drive_straight(motorA, motorE, straightAngle)) {
                    stalled = 1;
                    break;  // 以降の辺の走行を飛ばす
                }

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
