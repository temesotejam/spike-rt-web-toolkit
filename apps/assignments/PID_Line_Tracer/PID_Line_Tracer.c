/*
 * PID_Line_Tracer.c
 *
 * 元リポジトリ：
 * https://github.com/kouhei1970/pid_exercise
 *
 * 【今回の機体に合わせた変更点】
 * 1. PID演算、ゲイン、制御周期、反射光差によるライントレース処理は
 *    元プログラムの考え方を維持。
 * 2. 左右モーターを正方形走行プログラム M_S6 と同じA/Eポートへ変更。
 * 3. モーター初期化をSPIKE-RT v0.2.0の
 *    pup_motor_get_device() + pup_motor_setup() に変更。
 * 4. A/Eをモーターに使うため、カラーセンサーをB/Fポートへ変更。
 * 5. Cポートにフォースセンサーを追加。1回押すと開始、再度押すと停止。
 *
 * 機体仕様はM_S6に合わせる：
 *   左モーター A（反時計回り）
 *   右モーター E（時計回り）
 *   左カラーセンサー B、右カラーセンサー F
 *   フォースセンサー C（開始／停止）
 *   タイヤ直径 88mm、トレッド 144mm
 *
 * PIDの出力は左右の車輪速度 [deg/s] に反映する。
 * センサーはラインの左右に同じ高さ・同じ距離で取り付ける。
 */
#include <kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <t_syslog.h>

#include "PID_Line_Tracer.h"
#include "kernel_cfg.h"

#include "spike/pup/motor.h"
#include "spike/pup/colorsensor.h"
#include "spike/pup/forcesensor.h"  /* 追加：Cポートの開始・停止ボタン */
#include "spike/hub/speaker.h"

/*------------------------------------------------------------
 * ポート割り当て
 *
 * 元プログラム：センサーA/B、モーターC/D
 * 今回の機体  ：モーターA/EをM_S6と共通化し、
 *                センサーをB/Fへ移動。Cにはフォースセンサーを接続
 *-----------------------------------------------------------*/
#define LEFT_COLOR_SENSOR   PBIO_PORT_ID_B
#define RIGHT_COLOR_SENSOR  PBIO_PORT_ID_F
#define START_STOP_SENSOR   PBIO_PORT_ID_C
#define LEFT_MOTOR           PBIO_PORT_ID_A
#define RIGHT_MOTOR          PBIO_PORT_ID_E

/* --- 元プログラムのPIDパラメータを維持 --- */
#define BASE_POWER          100       /* 基準速度 [deg/s] */
#define MV_MAX              100.0f    /* 操作量の上限 */
#define CONTROL_PERIOD_MS   10
#define CONTROL_PERIOD_US   (CONTROL_PERIOD_MS * 1000)
#define BUTTON_DEBOUNCE_SAMPLES 2 /* 10ms×2回連続で押下／解放を確定 */

static const float kp = 1.0f;   /* 比例ゲイン */
static const float ki = 0.1f;   /* 積分ゲイン */
static const float kd = 0.1f;   /* 微分ゲイン */
static const float delta_T =
    (float)CONTROL_PERIOD_MS / 1000.0f;

/*------------------------------------------------------------
 * MAIN_TASK：2つのカラーセンサーを使ったPIDライントレース
 *-----------------------------------------------------------*/
void Main(intptr_t exinf)
{
    int32_t reflection_left;
    int32_t reflection_right;
    int32_t left_speed;
    int32_t right_speed;

    /* 元プログラムと同じく、左右センサーの反射光差を0にする */
    const float midpoint = 0.0f;

    /* PID制御の状態変数 */
    float last_error = 0.0f;
    float integral = 0.0f;
    float position = 0.0f;
    float error = 0.0f;
    float mv = 0.0f;

    /*--------------------------------------------------------
     * モーター初期化
     *
     * 【変更】M_S6と同じA/Eポート・回転方向に統一。
     * pup_motor_init()ではなく、v0.2.0のAPIを使用する。
     *-------------------------------------------------------*/
    pup_motor_t *motor_left =
        pup_motor_get_device(LEFT_MOTOR);
    pup_motor_t *motor_right =
        pup_motor_get_device(RIGHT_MOTOR);

    if (motor_left == NULL || motor_right == NULL) {
        syslog(LOG_ERROR, "PID: motor not found (A/E)");
        ext_tsk();
    }

    if (pup_motor_setup(
            motor_left, PUP_DIRECTION_COUNTERCLOCKWISE, true)
            != PBIO_SUCCESS ||
        pup_motor_setup(
            motor_right, PUP_DIRECTION_CLOCKWISE, true)
            != PBIO_SUCCESS) {
        syslog(LOG_ERROR, "PID: motor setup failed");
        pup_motor_stop(motor_left);
        pup_motor_stop(motor_right);
        ext_tsk();
    }

    /*--------------------------------------------------------
     * カラーセンサー初期化
     *
     * 【変更】モーターA/Eと重ならないB/Fポートを使用する。
     *-------------------------------------------------------*/
    pup_device_t *color_sensor_left =
        pup_color_sensor_get_device(LEFT_COLOR_SENSOR);
    pup_device_t *color_sensor_right =
        pup_color_sensor_get_device(RIGHT_COLOR_SENSOR);

    if (color_sensor_left == NULL || color_sensor_right == NULL) {
        syslog(LOG_ERROR, "PID: color sensor not found (B/F)");
        pup_motor_stop(motor_left);
        pup_motor_stop(motor_right);
        ext_tsk();
    }

    /*--------------------------------------------------------
     * 【追加】フォースセンサーの初期化
     * Cポートに接続。押下はpup_force_sensor_touched()で検知する。
     * センサーがない場合はモーターを動かさずに終了。
     *-------------------------------------------------------*/
    pup_device_t *start_stop_sensor =
        pup_force_sensor_get_device(START_STOP_SENSOR);
    if (start_stop_sensor == NULL) {
        syslog(LOG_ERROR, "PID: force sensor not found (C)");
        pup_motor_stop(motor_left);
        pup_motor_stop(motor_right);
        ext_tsk();
    }

    syslog(LOG_NOTICE,
           "PID exercise: motors A/E, color sensors B/F, force C");

    /* 元プログラムと同じ初期待機（3秒） */
    dly_tsk(3000000);

    /* 元プログラムの合図音は、ここでは走行開始ではなく操作受付の合図。 */
    hub_speaker_set_volume(100);
    hub_speaker_play_tone(695, 200);
    dly_tsk(200000);

    /* 【追加】初期状態は停止。Cポートを押すまで走り出さない。 */
    bool running = false;
    bool press_armed = true;
    unsigned int pressed_samples = 0;
    unsigned int released_samples = 0;
    syslog(LOG_NOTICE, "PID: ready. Press force sensor C to start.");

    /*--------------------------------------------------------
     * PID制御ループ
     *
     * 以下の反射光取得・PID演算は元プログラムを維持。
     * モーター速度の出力先だけをA/Eへ変更している。
     *-------------------------------------------------------*/
    while (1) {
        /*----------------------------------------------------
         * 【追加】フォースセンサーで走行／停止を切り替える
         * ・押下を2回連続確認したときだけ1回の操作として扱う。
         * ・次の操作は、センサーを2回連続で離してから受け付ける。
         * ・停止時は直ちに左右のモーターをブレーキする。
         *   押しっぱなしによる連続切り替えは発生しない。
         *---------------------------------------------------*/
        bool touched = pup_force_sensor_touched(start_stop_sensor);

        if (press_armed) {
            if (touched) {
                pressed_samples++;
                if (pressed_samples >= BUTTON_DEBOUNCE_SAMPLES) {
                    press_armed = false;
                    pressed_samples = 0;
                    released_samples = 0;

                    if (running) {
                        /* 走行中の押下で停止。PID値もリセット。 */
                        running = false;
                        pup_motor_brake(motor_left);
                        pup_motor_brake(motor_right);
                        integral = 0.0f;
                        last_error = 0.0f;
                        syslog(LOG_NOTICE, "PID: stopped by force sensor C");
                    } else {
                        /* 停止中の押下で再開。古い積分値を引き継がない。 */
                        integral = 0.0f;
                        last_error = 0.0f;
                        running = true;
                        syslog(LOG_NOTICE, "PID: started by force sensor C");
                    }
                }
            } else {
                pressed_samples = 0;
            }
        } else {
            if (!touched) {
                released_samples++;
                if (released_samples >= BUTTON_DEBOUNCE_SAMPLES) {
                    press_armed = true;
                    released_samples = 0;
                }
            } else {
                released_samples = 0;
            }
        }

        /* 停止中はPID演算も速度指令もしない。ボタン監視だけ継続。 */
        if (!running) {
            dly_tsk(CONTROL_PERIOD_US);
            continue;
        }

        /* 1. 左右カラーセンサーの反射光を取得 */
        reflection_left =
            pup_color_sensor_reflection(color_sensor_left);
        reflection_right =
            pup_color_sensor_reflection(color_sensor_right);

        /* 2. 左右センサーの差を位置情報として扱う */
        position = (float)(reflection_right - reflection_left);

        /* 3. 目標値との差（偏差） */
        error = midpoint - position;

        /* 4. 積分項（台形積分） */
        integral +=
            ((error + last_error) / 2.0f) * delta_T;

        /* 積分のたまりすぎを防止（元プログラムと同じ制限） */
        if (ki * integral > MV_MAX) {
            integral = MV_MAX / ki;
        }
        if (ki * integral < -MV_MAX) {
            integral = -MV_MAX / ki;
        }

        /* 5. PID制御量 */
        float derivative = (error - last_error) / delta_T;
        mv = kp * error + ki * integral + kd * derivative;

        /* 6. 操作量の制限 */
        if (mv > MV_MAX) {
            mv = MV_MAX;
        }
        if (mv < -MV_MAX) {
            mv = -MV_MAX;
        }

        /* 7. 左右モーター速度
         * mvが正：右旋回 → 左輪を速く、右輪を遅くする
         */
        left_speed =
            (int32_t)((float)BASE_POWER + mv);
        right_speed =
            (int32_t)((float)BASE_POWER - mv);

        /* 8. M_S6と同じA/Eモーターへ出力 */
        pup_motor_set_speed(motor_left, left_speed);
        pup_motor_set_speed(motor_right, right_speed);

        /* 9. 次回の微分・積分計算用に偏差を保存 */
        last_error = error;

        /* 10. 制御周期（10ms） */
        dly_tsk(CONTROL_PERIOD_US);
    }

    /* 通常は無限ループ。将来終了条件を追加した場合の停止処理 */
    pup_motor_stop(motor_left);
    pup_motor_stop(motor_right);
    ext_tsk();
}
