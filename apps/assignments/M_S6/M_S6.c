#include <kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <t_syslog.h>
#include <M_S6.h>
#include "spike/pup/motor.h"

// M_S5を元に、A/Bの2モータで円と正方形を走る課題。
// A: 左車輪、B: 右車輪の想定。前進時は両方の角速度が正になる向きに初期化。
// 車体の組み方に応じて、車輪径・車輪中心間距離・回転方向を調整すること。
#define PI                 3.14159265f
#define WHEEL_DIAMETER_MM  56.0f   // SPIKE Prime基本セットの最大タイヤ(56mm)
                                  // 拡張セットの88mmタイヤなら88.0fに変更
#define WHEEL_TRACK_MM     120.0f  // 左右の車輪中心間距離の仮値。実測して変更する。
#define CIRCLE_RADIUS_MM   250.0f
#define SQUARE_SIDE_MM    500.0f
#define MAX_SPEED_DPS      320     // モータ角速度(度/秒)
#define TURN_SPEED_DPS     220
#define POSITION_TOL_DEG   3       // 到達判定の許容角度
#define TIMEOUT_TICKS      2500    // 10ms × 2500 = 最大25秒/動作

// 走行距離(mm)を車輪の回転角度(度)に変換する。
static int32_t mm_to_deg(float mm)
{
    return (int32_t)(mm * 360.0f / (PI * WHEEL_DIAMETER_MM) + 0.5f);
}

// 指定した角度だけA/Bを動かす。エンコーダ角を確認して停止する。
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
    // M_S5と同様にモータAとBを取得する。
    pup_motor_t *motorA = pup_motor_get_device(PBIO_PORT_ID_A);
    pup_motor_t *motorB = pup_motor_get_device(PBIO_PORT_ID_B);
    if (motorA == NULL || motorB == NULL) {
        syslog(LOG_ERROR, "Connect both motors to A and B.");
        exit(1);
    }

    // A=左・B=右の標準的な配置を想定する。
    // 実機で両輪が前進しなければ、回転方向を調整する。
    if (pup_motor_setup(motorA, PUP_DIRECTION_COUNTERCLOCKWISE, true) != PBIO_SUCCESS ||
        pup_motor_setup(motorB, PUP_DIRECTION_CLOCKWISE, true) != PBIO_SUCCESS) {
        syslog(LOG_ERROR, "Motor initialization failed.");
        pup_motor_brake(motorA);
        pup_motor_brake(motorB);
        exit(1);
    }

    // 車体中心が半径250mmの円を右回りに1周する。
    // 外側(左)のタイヤは内側(右)より長い距離を進む。
    float outerMM = 2.0f * PI * (CIRCLE_RADIUS_MM + WHEEL_TRACK_MM / 2.0f);
    float innerMM = 2.0f * PI * (CIRCLE_RADIUS_MM - WHEEL_TRACK_MM / 2.0f);
    int32_t innerSpeed = (int32_t)(MAX_SPEED_DPS * innerMM / outerMM);
    if (innerMM <= 0.0f || innerSpeed < 90) {
        syslog(LOG_ERROR, "Invalid wheel track or circle radius.");
        exit(1);
    }

    syslog(LOG_NOTICE, "M_S6: circle radius=250mm, then square side=500mm.");
    dly_tsk(2000000); // 安全のため、2秒後に走行開始

    syslog(LOG_NOTICE, "CIRCLE START");
    if (!move_wheels(motorA, motorB,
                     mm_to_deg(outerMM), mm_to_deg(innerMM),
                     MAX_SPEED_DPS, innerSpeed)) {
        exit(1);
    }
    pup_motor_brake(motorA);
    pup_motor_brake(motorB);
    dly_tsk(2000000); // 円と正方形の間は2秒停止

    // 正方形：500mm直進 -> その場で右へ90°旋回、を4回繰り返す。
    // 左右輪中心間距離をWとすると、90°旋回時の片輪移動距離は πW/4。
    int32_t straightDeg = mm_to_deg(SQUARE_SIDE_MM);
    int32_t turnDeg = mm_to_deg(PI * WHEEL_TRACK_MM / 4.0f);
    syslog(LOG_NOTICE, "SQUARE START");
    for (int side = 0; side < 4; side++) {
        if (!move_wheels(motorA, motorB, straightDeg, straightDeg,
                         MAX_SPEED_DPS, MAX_SPEED_DPS)) {
            exit(1);
        }
        // その場で右旋回するため、左輪を正・右輪を負方向へ動かす。
        if (!move_wheels(motorA, motorB, turnDeg, -turnDeg,
                         TURN_SPEED_DPS, TURN_SPEED_DPS)) {
            exit(1);
        }
    }

    pup_motor_brake(motorA);
    pup_motor_brake(motorB);
    syslog(LOG_NOTICE, "M_S6: completed circle and square.");
    exit(0);
}
