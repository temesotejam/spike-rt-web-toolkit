#include <kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <t_syslog.h>
#include <M_S6.h>
#include "spike/pup/motor.h"
#include "spike/hub/button.h"     // 左右・中央ボタン
#include "spike/hub/display.h"    // 5×5ドット表示
#include "spike/hub/speaker.h"    // 確定音

// M_S5を元に、左右ボタンで円／正方形を選択して走る課題。
// Advanced Driving Base Assembly準拠。A: 左車輪、E: 右車輪。
// 前進時は両方の角速度が正になる向きに初期化。
// 車体の組み方に応じて、車輪径・車輪中心間距離・回転方向を調整すること。
#define PI                 3.14159265f
#define WHEEL_DIAMETER_MM  88.0f   // 写真の大型タイヤ (88×14mm)
#define WHEEL_TRACK_MM     144.0f  // ADBの文献記載値14.4cm。実機で測って微調整。
#define CIRCLE_RADIUS_MM   250.0f  // 円の半径は外側の左タイヤの軌跡を基準とする
#define SQUARE_SIDE_MM    500.0f  // 正方形の辺長も左タイヤの軌跡を基準とする
#define MAX_SPEED_DPS      320     // モータ角速度(度/秒)
#define TURN_SPEED_DPS     220
#define POSITION_TOL_DEG   3       // 到達判定の許容角度
#define TIMEOUT_TICKS      2500    // 10ms × 2500 = 最大25秒/動作

// 走行距離(mm)を車輪の回転角度(度)に変換する。
static int32_t mm_to_deg(float mm)
{
    return (int32_t)(mm * 360.0f / (PI * WHEEL_DIAMETER_MM) + 0.5f);
}

// 指定した角度だけA/Eを動かす。エンコーダ角を確認して停止する。
// 初回の回転角は現在値を基準にするので、何度でも順番に呼び出せる。
static bool move_wheels(pup_motor_t *motorA, pup_motor_t *motorB,
                        int32_t degA, int32_t degB,
                        int32_t maxSpeedA, int32_t maxSpeedB)
{
    int32_t targetA = pup_motor_get_count(motorA) + degA;
    int32_t targetB = pup_motor_get_count(motorB) + degB;

    for (int ticks = 0; ticks < TIMEOUT_TICKS; ticks++) {
        int32_t errorA = targetA - pup_motor_get_count(motorA);
        int32_t errorB = targetB - pup_motor_get_count(motorB);
        bool doneA = errorA >= -POSITION_TOL_DEG && errorA <= POSITION_TOL_DEG;
        bool doneB = errorB >= -POSITION_TOL_DEG && errorB <= POSITION_TOL_DEG;

        if (doneA) {
            pup_motor_brake(motorA);
        } else {
            // 目標に近づいたら減速する。
            int32_t speedA = (errorA < 0 ? -errorA : errorA) * 2;
            if (speedA < 90) speedA = 90;
            if (speedA > maxSpeedA) speedA = maxSpeedA;
            pup_motor_set_speed(motorA, errorA > 0 ? speedA : -speedA);
        }

        if (doneB) {
            pup_motor_brake(motorB);
        } else {
            int32_t speedB = (errorB < 0 ? -errorB : errorB) * 2;
            if (speedB < 90) speedB = 90;
            if (speedB > maxSpeedB) speedB = maxSpeedB;
            pup_motor_set_speed(motorB, errorB > 0 ? speedB : -speedB);
        }

        if (doneA && doneB) return true;
        dly_tsk(10000);  // 10msごとに位置を確認
    }

    pup_motor_brake(motorA);
    pup_motor_brake(motorB);
    syslog(LOG_ERROR, "Movement timeout: check wheels and motors.");
    return false;
}

void Main(intptr_t exinf)
{
    // Advanced Driving Baseの走行モータはA(左)、E(右)。
    pup_motor_t *motorA = pup_motor_get_device(PBIO_PORT_ID_A);
    pup_motor_t *motorB = pup_motor_get_device(PBIO_PORT_ID_E);
    if (motorA == NULL || motorB == NULL) {
        syslog(LOG_ERROR, "Connect drive motors to A (left) and E (right).");
        exit(1);
    }

    // A=左・E=右の公式Advanced Driving Base配置を想定する。
    // 実機で両輪が前進しなければ、回転方向を調整する。
    if (pup_motor_setup(motorA, PUP_DIRECTION_COUNTERCLOCKWISE, true) != PBIO_SUCCESS ||
        pup_motor_setup(motorB, PUP_DIRECTION_CLOCKWISE, true) != PBIO_SUCCESS) {
        syslog(LOG_ERROR, "Motor initialization failed.");
        pup_motor_brake(motorA);
        pup_motor_brake(motorB);
        exit(1);
    }

    // 外側の左タイヤ(A)が半径250mmの円を右回りに1周する。
    // 内側の右タイヤ(E)の半径 = 250mm - 左右の車輪中心間距離。
    float outerMM = 2.0f * PI * CIRCLE_RADIUS_MM;
    float innerMM = 2.0f * PI * (CIRCLE_RADIUS_MM - WHEEL_TRACK_MM);
    int32_t innerSpeed = (int32_t)(MAX_SPEED_DPS * innerMM / outerMM);
    if (innerMM <= 0.0f || innerSpeed < 90) {
        syslog(LOG_ERROR, "Invalid wheel track or circle radius.");
        exit(1);
    }

    // 5×5ドットで、円と正方形の輪郭を表示する（0=消灯、100=点灯）。
    uint8_t circleImage[25] = {
          0, 100, 100, 100,   0,
        100,   0,   0,   0, 100,
        100,   0,   0,   0, 100,
        100,   0,   0,   0, 100,
          0, 100, 100, 100,   0
    };
    uint8_t squareImage[25] = {
        100, 100, 100, 100, 100,
        100,   0,   0,   0, 100,
        100,   0,   0,   0, 100,
        100,   0,   0,   0, 100,
        100, 100, 100, 100, 100
    };

    // 選択中は円が初期値。走行後も前回の選択を保持する。
    bool squareSelected = false;
    hub_button_t buttons = 0;
    hub_button_t previousButtons = 0;
    hub_speaker_set_volume(5);  // 教室で使うため、短い確認音は音量5%

    // 正方形の角度目標は車輪中心間距離から計算する。
    int32_t straightDeg = mm_to_deg(SQUARE_SIDE_MM);
    int32_t turnDeg = mm_to_deg(PI * WHEEL_TRACK_MM / 2.0f);

    syslog(LOG_NOTICE, "M_S6: LEFT/RIGHT=change shape, CENTER=start.");

    while (1) {
        // 走行が終わったら安全に停止し、選択画面に戻る。
        pup_motor_brake(motorA);
        pup_motor_brake(motorB);

        // 前回の中央ボタンが押されたままなら離すまで待つ。
        hub_button_is_pressed(&buttons);
        while (buttons & (HUB_BUTTON_LEFT | HUB_BUTTON_RIGHT | HUB_BUTTON_CENTER)) {
            dly_tsk(20000);
            hub_button_is_pressed(&buttons);
        }
        previousButtons = 0;
        hub_display_image(squareSelected ? squareImage : circleImage);

        // 左右ボタンのどちらかで切り替え、中央ボタンで確定。
        while (1) {
            hub_button_is_pressed(&buttons);

            if (((buttons & HUB_BUTTON_LEFT) && !(previousButtons & HUB_BUTTON_LEFT)) ||
                ((buttons & HUB_BUTTON_RIGHT) && !(previousButtons & HUB_BUTTON_RIGHT))) {
                squareSelected = !squareSelected;
                hub_display_image(squareSelected ? squareImage : circleImage);
            }

            if ((buttons & HUB_BUTTON_CENTER) && !(previousButtons & HUB_BUTTON_CENTER)) {
                break;
            }

            previousButtons = buttons;
            dly_tsk(20000);  // 20msごとにボタン確認
        }

        // 決定音を1回短く鳴らしてから、選んだ軌跡の走行を開始する。
        hub_speaker_play_tone(NOTE_C5, 150);

        if (!squareSelected) {
            // 円：外側の左タイヤが半径250mmの円を1周する。
            syslog(LOG_NOTICE, "CIRCLE START");
            if (!move_wheels(motorA, motorB,
                             mm_to_deg(outerMM), mm_to_deg(innerMM),
                             MAX_SPEED_DPS, innerSpeed)) {
                exit(1);
            }
            syslog(LOG_NOTICE, "CIRCLE FINISHED");
        } else {
            // 正方形：500mm直進→左タイヤを止めて右へ90°、を4回。
            syslog(LOG_NOTICE, "SQUARE START");
            for (int side = 0; side < 4; side++) {
                if (!move_wheels(motorA, motorB, straightDeg, straightDeg,
                                 MAX_SPEED_DPS, MAX_SPEED_DPS)) {
                    exit(1);
                }
                // 右タイヤを後退させて、左タイヤを角に固定して右旋回。
                if (!move_wheels(motorA, motorB, 0, -turnDeg,
                                 0, TURN_SPEED_DPS)) {
                    exit(1);
                }
            }
            syslog(LOG_NOTICE, "SQUARE FINISHED");
        }

        // 走行終了後は停止して、上の選択画面へ戻る。
        pup_motor_brake(motorA);
        pup_motor_brake(motorB);
    }
}
