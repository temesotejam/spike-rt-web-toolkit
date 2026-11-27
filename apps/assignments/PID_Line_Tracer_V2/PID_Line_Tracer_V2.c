/*
 * PID_Line_Tracer_V2.c
 *
 * SPIKE-RT v0.2.0の標準APIのみを使用します。カーネルやデバイスドライバは変更しません。
 *
 * 機体・ポート構成:
 *   左モーター       A（反時計回り）
 *   右モーター       E（時計回り）
 *   左カラーセンサー B
 *   右カラーセンサー D
 *   開始・停止用フォースセンサー C
 *
 * モード選択:
 *   Hub左・右ボタン -> ライントレース／校正／データ出力を選択
 *   Hub中央ボタン   -> 決定
 *   フォースセンサーC短押し -> 走行開始／終了
 *   フォースセンサーC長押し（5秒） -> プログラム終了
 *
 * 走行データ:
 *   走行中のセンサー値・オドメトリ・モーター指令をRAMに記録。
 *   走行終了後はモード選択画面に戻り、USBシリアルからCSV形式で出力可能。
 *
 * 校正時の機体配置:
 *   左右のカラーセンサーを白い面に置き、機体を進行方向と直角に交わる
 *   黒いラインに向けてまっすぐ配置します。
 *   機体を前進させ、横断ラインを次の順番で通過します。
 *       白 -> 黒 -> 白
 *   左右のセンサーを個別に測定し、それぞれの白・黒の反射光を校正します。
 *   校正に成功すると、モード選択画面に戻ります。
 *
 * V2の主な機能:
 *   - 10ms周期の通知とセマフォによる制御タスクの起床。
 *   - fch_hrt()を使った実際の経過時間の測定。
 *   - ラインを横切る白・黒・白の移動校正。
 *   - センサーごとの反射光正規化。
 *   - 微分項のフィルタと積分アンチワインドアップ。
 *   - 基準速度の適応調整とモーター指令値の制限。
 *   - 車輪エンコーダだけを使う差動二輪オドメトリで相対位置・方位を推定。
 *   - B/Dカラーセンサーの左右反射光平均をPID偏差として操舵に使用。
 *   - ライン状態、反射光、モーター指令、エンコーダオドメトリを5Hzで記録。
 *   - IMU/EKFは使用しない。
 *   - 直前の走行データと校正データをUSBシリアルへCSV出力。
 *   - フォースセンサーCの5秒長押しで、全モードからプログラムを終了。
 *   - Hub RGBライトでB/Dのライン認識状態を待機中・走行中に色表示。
 *   - IMU/EKFは使用せず、車輪エンコーダを位置・方位推定の基礎にする。
 */

#include <kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <t_syslog.h>

#include "PID_Line_Tracer_V2.h"
#include "kernel_cfg.h"

#include "spike/pup/motor.h"
#include "spike/pup/colorsensor.h"
#include "spike/pup/forcesensor.h"
#include "spike/hub/button.h"
#include "spike/hub/display.h"
#include "spike/hub/speaker.h"
#include "spike/hub/light.h"

/* ポート割り当て */
#define LEFT_COLOR_SENSOR       PBIO_PORT_ID_B
#define RIGHT_COLOR_SENSOR      PBIO_PORT_ID_D
#define START_STOP_SENSOR       PBIO_PORT_ID_C
#define LEFT_MOTOR              PBIO_PORT_ID_A
#define RIGHT_MOTOR             PBIO_PORT_ID_E

/* 制御タイミング */
#define CONTROL_PERIOD_US       10000U
#define CONTROL_DT_MIN_US       1000U
#define CONTROL_DT_MAX_US       50000U

/* 診断ログの出力頻度は、100Hzの制御ループより低くする */
#define DIAGNOSTIC_LOG_INTERVAL_TICKS  20U
#define RUN_LOG_CAPACITY        512U

/* Hubのモード選択ボタンを確認する周期 */
#define MENU_POLL_US            20000U

/* 校正時は、機体を低速で横断する黒いラインの上を通過させる */
#define CAL_WHITE_SAMPLES       8U
#define CAL_WHITE_MIN_REFLECT   20
#define CAL_BLACK_RATIO         0.85f
#define CAL_WHITE_AFTER_SAMPLES 8U
#define CAL_MIN_BLACK_SAMPLES   2U
#define CAL_MIN_CONTRAST        12.0f
#define CAL_SPEED               60
#define CAL_MIN_SPEED           35
#define CAL_MAX_SPEED           80
#define CAL_SYNC_GAIN           1
#define CAL_SYNC_LIMIT          15
#define CAL_MAX_TRAVEL_DEG      540
#define CAL_TIMEOUT_US          12000000U
#define CAL_STALL_IGNORE_US     500000U

/* フォースセンサーの短押し判定とプログラム終了判定 */
#define FORCE_PRESS_DEBOUNCE_US 120000U
#define FORCE_LONG_PRESS_US     5000000U

/* PIDゲインは演習プログラムの初期値を引き継ぐ */
#define KP                      1.0f
#define KI                      0.1f
#define KD                      0.1f
#define I_TERM_LIMIT            20.0f
#define DERIVATIVE_FILTER_TAU_S 0.040f

/* 左右車輪の速度を適応調整する範囲 [deg/s] */
#define BASE_SPEED_STRAIGHT     120
#define BASE_SPEED_TURN         80
#define SLOWDOWN_ERROR          40.0f
#define MIN_WHEEL_SPEED         30
#define MAX_WHEEL_SPEED         200
#define MAX_STEERING            100.0f

/* 差動二輪オドメトリの機体寸法：M_S6の構成値 */
#define ODOM_PI                  3.14159265f
#define WHEEL_DIAMETER_MM        88.0f
#define WHEEL_TRACK_MM           144.0f
#define ODOM_RAD_TO_DEG          57.2957795f
#define ODOM_YAW_RATE_THRESHOLD  8.0f
#define ODOM_YAW_RATE_FULL       60.0f
#define ODOM_YAW_DAMPING_GAIN    0.15f
/* 正規化反射光の状態判定しきい値（黒=0、白=100） */
#define LINE_DARK_THRESHOLD      45.0f
#define LINE_WHITE_THRESHOLD     70.0f
#define RECOVERY_MAX_MM          120.0f
#define RECOVERY_MAX_US          3000000U
#define RECOVERY_CONFIRM_TICKS   3U
#define LINE_CENTER_DEADBAND     4.0f
#define LINE_BOTH_DARK_MAX_SPEED 65
#define LINE_TRANSITION_MAX_SPEED 95

typedef enum {
    MODE_LINE_TRACE = 0,
    MODE_CALIBRATION = 1,
    MODE_EXPORT_DATA = 2,
    MODE_RIGHT_BOUNDARY = 3,
    MODE_EXIT = 4
} app_mode_t;

typedef enum {
    FORCE_PRESS_NONE = 0,
    FORCE_PRESS_SHORT,
    FORCE_PRESS_LONG
} force_press_event_t;

typedef struct {
    bool active;
    uint32_t start_hrt;
    uint32_t last_duration_us;
} force_press_state_t;

typedef enum {
    RUN_END_NONE = 0,
    RUN_END_RUNNING,
    RUN_END_FORCE_SENSOR,
    RUN_END_PROGRAM_EXIT,
    RUN_END_COLOR_SENSOR_ERROR,
    RUN_END_MOTOR_ERROR,
    RUN_END_CONTROL_ERROR,
    RUN_END_LINE_LOST
} run_end_reason_t;

typedef enum {
    RUN_ERROR_NONE = 0,
    RUN_ERROR_COLOR_SENSOR_INVALID,
    RUN_ERROR_MOTOR_COMMAND,
    RUN_ERROR_FORCE_SENSOR_SHORT_PRESS,
    RUN_ERROR_CONTROL_CLOCK
} run_error_code_t;

typedef enum {
    CAL_SEEK_BLACK = 0,
    CAL_MEASURE_BLACK,
    CAL_SEEK_WHITE_AFTER,
    CAL_DONE
} calibration_phase_t;

typedef enum {
    CAL_RESULT_OK = 0,
    CAL_RESULT_FAILED,
    CAL_RESULT_CANCELLED,
    CAL_RESULT_EXIT
} calibration_result_t;

typedef struct {
    calibration_phase_t phase;
    float white_before;
    float threshold;
    float black_sum;
    uint16_t black_samples;
    float white_after_sum;
    uint16_t white_after_samples;
} sensor_crossing_t;

typedef struct {
    float white_left;
    float black_left;
    float white_right;
    float black_right;
    bool valid;
} line_calibration_t;

typedef enum {
    LINE_STATE_BOTH_WHITE = 0,
    LINE_STATE_LEFT_EDGE,
    LINE_STATE_RIGHT_EDGE,
    LINE_STATE_BOTH_DARK,
    LINE_STATE_TRANSITION,
    LINE_STATE_INVALID
} line_state_t;

typedef enum {
    MOTION_STATE_STRAIGHT = 0,
    MOTION_STATE_TURNING_LEFT,
    MOTION_STATE_TURNING_RIGHT
} motion_state_t;

typedef struct {
    int32_t previous_left_count;
    int32_t previous_right_count;
    float x_mm;
    float y_mm;
    float heading_rad;
    float distance_mm;
    float linear_speed_mm_s;
    float encoder_yaw_rate_deg_s;
    float curvature_per_m;
    bool initialized;
} odometry_t;

/* 1サンプル分の走行記録。浮動小数点値は出力時の扱いを簡単にするため
 * 所定の倍率を掛けた整数として保持する。 */
typedef struct {
    uint32_t elapsed_ms;
    int16_t raw_left;
    int16_t raw_right;
    int16_t normalized_left_x10;
    int16_t normalized_right_x10;
    int32_t line_error_x10;
    int16_t steering_x10;
    int16_t motor_left;
    int16_t motor_right;
    int32_t odom_x_mm;
    int32_t odom_y_mm;
    int32_t odom_distance_mm;
    int16_t odom_heading_x10;
    int16_t encoder_yaw_x10;
    int16_t curvature_x1000;
    int16_t speed_mm_s;
    uint8_t line_state;
    uint8_t motion_state;
} run_log_record_t;

static bool latest_right_boundary = false;
static volatile uint32_t control_tick_overruns = 0;
/* 実機で成功した校正値を初期値として使用する。
 * 起動後に校正を行わなくても、この値でライントレースを開始できる。
 * 値は2026-11-26の実測成功値（反射光スケール）：
 * 左B 白82.4/黒26.4、右D 白72.1/黒21.1。 */
static line_calibration_t saved_calibration = {
    .white_left = 82.4f,
    .black_left = 26.4f,
    .white_right = 72.1f,
    .black_right = 21.1f,
    .valid = true
};
static run_log_record_t run_log[RUN_LOG_CAPACITY];
static uint32_t run_log_count = 0;
static uint32_t run_log_write_index = 0;
static uint32_t run_log_total_samples = 0;
static uint32_t run_log_overwritten = 0;
static bool latest_run_valid = false;

static force_press_event_t update_force_press(
    force_press_state_t *state,
    bool touched,
    uint32_t now_hrt);
static uint32_t latest_run_start_hrt = 0;
static uint32_t latest_run_end_hrt = 0;
static run_end_reason_t latest_run_end_reason = RUN_END_NONE;
static run_error_code_t latest_run_error_code = RUN_ERROR_NONE;
static uint32_t latest_run_error_elapsed_ms = 0;
static int32_t latest_run_error_left = 0;
static int32_t latest_run_error_right = 0;
static uint32_t latest_force_press_duration_ms = 0;
static int line_status_light_state = -1;

/* 浮動小数点値を指定範囲内に収める */
static float clamp_float(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static int16_t log_i16(float value)
{
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    return (int16_t)value;
}

static void clear_latest_run_log(void)
{
    run_log_count = 0;
    run_log_write_index = 0;
    run_log_total_samples = 0;
    run_log_overwritten = 0;
    latest_run_valid = false;
    latest_run_start_hrt = 0;
    latest_run_end_hrt = 0;
    latest_run_end_reason = RUN_END_NONE;
}

static void append_run_log(
    uint32_t now_hrt,
    uint32_t run_start_hrt,
    int32_t reflection_left,
    int32_t reflection_right,
    float normalized_left,
    float normalized_right,
    float line_error,
    float steering,
    int32_t motor_left,
    int32_t motor_right,
    const odometry_t *odometry,
    line_state_t line_state,
    motion_state_t motion_state)
{
    run_log_record_t *record = &run_log[run_log_write_index];

    record->elapsed_ms = (uint32_t)(now_hrt - run_start_hrt) / 1000U;
    record->raw_left = (int16_t)reflection_left;
    record->raw_right = (int16_t)reflection_right;
    record->normalized_left_x10 = log_i16(normalized_left * 10.0f);
    record->normalized_right_x10 = log_i16(normalized_right * 10.0f);
    record->line_error_x10 = (int32_t)(line_error * 10.0f);
    record->steering_x10 = log_i16(steering * 10.0f);
    record->motor_left = log_i16((float)motor_left);
    record->motor_right = log_i16((float)motor_right);
    record->odom_x_mm = (int32_t)odometry->x_mm;
    record->odom_y_mm = (int32_t)odometry->y_mm;
    record->odom_distance_mm = (int32_t)odometry->distance_mm;
    record->odom_heading_x10 =
        log_i16(odometry->heading_rad * ODOM_RAD_TO_DEG * 10.0f);
    record->encoder_yaw_x10 =
        log_i16(odometry->encoder_yaw_rate_deg_s * 10.0f);
    record->curvature_x1000 = log_i16(odometry->curvature_per_m * 1000.0f);
    record->speed_mm_s = log_i16(odometry->linear_speed_mm_s);
    record->line_state = (uint8_t)line_state;
    record->motion_state = (uint8_t)motion_state;

    run_log_write_index++;
    if (run_log_write_index >= RUN_LOG_CAPACITY) {
        run_log_write_index = 0;
    }

    if (run_log_count < RUN_LOG_CAPACITY) {
        run_log_count++;
    } else {
        run_log_overwritten++;
    }
    run_log_total_samples++;
}

static const char *run_end_reason_name(run_end_reason_t reason)
{
    switch (reason) {
    case RUN_END_RUNNING:            return "RUNNING";
    case RUN_END_FORCE_SENSOR:       return "FORCE_SENSOR";
    case RUN_END_PROGRAM_EXIT:       return "PROGRAM_EXIT";
    case RUN_END_COLOR_SENSOR_ERROR: return "COLOR_SENSOR_ERROR";
    case RUN_END_MOTOR_ERROR:        return "MOTOR_ERROR";
    case RUN_END_CONTROL_ERROR:      return "CONTROL_ERROR";
    case RUN_END_LINE_LOST:           return "LINE_LOST";
    case RUN_END_NONE:
    default:                         return "NONE";
    }
}

static const char *run_error_code_name(run_error_code_t code)
{
    switch (code) {
    case RUN_ERROR_COLOR_SENSOR_INVALID: return "COLOR_SENSOR_INVALID";
    case RUN_ERROR_MOTOR_COMMAND:        return "MOTOR_COMMAND_FAILED";
    case RUN_ERROR_FORCE_SENSOR_SHORT_PRESS: return "FORCE_SENSOR_SHORT_PRESS";
    case RUN_ERROR_CONTROL_CLOCK: return "CONTROL_CLOCK";
    case RUN_ERROR_NONE:
    default:                             return "NONE";
    }
}

/* 走行終了後に、校正値・直前の走行ログをCSV形式でUSBへ出力する。
 * この処理は周期制御を停止した通常タスク上で実行する。 */
static bool export_pause(force_press_state_t *state, pup_device_t *sensor)
{
    fflush(stdout);
    dly_tsk(20000U);
    if (update_force_press(state, pup_force_sensor_touched(sensor),
                           (uint32_t)fch_hrt()) != FORCE_PRESS_LONG) {
        return false;
    }
    printf("\r\n# EXPORT_ABORTED_BY_FORCE_SENSOR_LONG_PRESS\r\n");
    fflush(stdout);
    return true;
}

static bool export_latest_data(pup_device_t *sensor)
{
    force_press_state_t press = {0};
    printf("\r\n# PID_LINE_TRACER_V2_EXPORT,2\r\n");
    printf("# CALIBRATION,valid=%d,white_left_x10=%ld,black_left_x10=%ld,"
           "white_right_x10=%ld,black_right_x10=%ld\r\n",
           saved_calibration.valid ? 1 : 0,
           (long)(saved_calibration.white_left * 10.0f),
           (long)(saved_calibration.black_left * 10.0f),
           (long)(saved_calibration.white_right * 10.0f),
           (long)(saved_calibration.black_right * 10.0f));
    if (export_pause(&press, sensor)) return true;
    printf("# SETTINGS,wheel_diameter_mm=%d,wheel_track_mm=%d,"
           "kp_x1000=%ld,ki_x1000=%ld,kd_x1000=%ld\r\n",
           (int)WHEEL_DIAMETER_MM, (int)WHEEL_TRACK_MM,
           (long)(KP * 1000.0f), (long)(KI * 1000.0f), (long)(KD * 1000.0f));
    printf("# RUN,valid=%d,total_samples=%lu,stored_samples=%lu,"
           "overwritten_samples=%lu,duration_ms=%lu,end_reason=%s\r\n",
           latest_run_valid ? 1 : 0,
           (unsigned long)run_log_total_samples,
           (unsigned long)run_log_count,
           (unsigned long)run_log_overwritten,
           (unsigned long)(latest_run_valid ?
               (latest_run_end_hrt - latest_run_start_hrt) / 1000U : 0U),
           run_end_reason_name(latest_run_end_reason));
    if (export_pause(&press, sensor)) return true;
    printf("# RUN_DIAGNOSTIC,code=%s,error_elapsed_ms=%lu,"
           "detail_left=%ld,detail_right=%ld,force_press_ms=%lu\r\n",
           run_error_code_name(latest_run_error_code),
           (unsigned long)latest_run_error_elapsed_ms,
           (long)latest_run_error_left, (long)latest_run_error_right,
           (unsigned long)latest_force_press_duration_ms);
    printf("# BOUNDARY,%s\r\n", latest_right_boundary ? "RIGHT" : "LEFT");
    printf("# LINE_STATE,0=BOTH_WHITE,1=B_DARK,2=D_DARK,3=BOTH_DARK,4=TRANSITION\r\n");
    printf("# MOTION_STATE,0=STRAIGHT,1=TURNING_LEFT,2=TURNING_RIGHT\r\n");
    if (export_pause(&press, sensor)) return true;
    printf("DATA_BEGIN\r\n");
    printf("elapsed_ms,raw_left,raw_right,norm_left_x10,norm_right_x10,"
           "line_state,motion_state,error_x10,steering_x10,motor_left,motor_right,"
           "odom_x_mm,odom_y_mm,odom_distance_mm,odom_heading_x10,"
           "encoder_yaw_x10,curvature_x1000,speed_mm_s\r\n");
    if (export_pause(&press, sensor)) return true;
    if (latest_run_valid) {
        uint32_t oldest = run_log_count == RUN_LOG_CAPACITY ? run_log_write_index : 0U;
        for (uint32_t i = 0; i < run_log_count; ++i) {
            const run_log_record_t *r = &run_log[(oldest + i) % RUN_LOG_CAPACITY];
            printf("%lu,%d,%d,%d,%d,%u,%u,%ld,%d,%d,%d,%ld,%ld,%ld,%d,%d,%d,%d\r\n",
                   (unsigned long)r->elapsed_ms, (int)r->raw_left, (int)r->raw_right,
                   (int)r->normalized_left_x10, (int)r->normalized_right_x10,
                   (unsigned int)r->line_state, (unsigned int)r->motion_state,
                   (long)r->line_error_x10, (int)r->steering_x10,
                   (int)r->motor_left, (int)r->motor_right,
                   (long)r->odom_x_mm, (long)r->odom_y_mm, (long)r->odom_distance_mm,
                   (int)r->odom_heading_x10, (int)r->encoder_yaw_x10,
                   (int)r->curvature_x1000, (int)r->speed_mm_s);
            if (export_pause(&press, sensor)) return true;
        }
    }
    printf("DATA_END\r\n# EXPORT_COMPLETE\r\n");
    fflush(stdout);
    return false;
}

static int32_t clamp_int32(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static int32_t abs_int32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int32_t clamp_speed(int32_t value)
{
    return clamp_int32(value, MIN_WHEEL_SPEED, MAX_WHEEL_SPEED);
}

/* ライン偏差または旋回角速度が大きい場合に前進速度を下げる。
 * オドメトリによる速度調整は過度にならないよう控えめにする。 */
static int32_t calculate_base_speed(float abs_error, float abs_yaw_rate)
{
    const float error_ratio =
        clamp_float(abs_error / SLOWDOWN_ERROR, 0.0f, 1.0f);
    const float yaw_ratio =
        clamp_float(abs_yaw_rate / ODOM_YAW_RATE_FULL, 0.0f, 1.0f);
    const float ratio =
        error_ratio > yaw_ratio ? error_ratio : yaw_ratio;

    return (int32_t)(
        (float)BASE_SPEED_STRAIGHT -
        ratio * (float)(BASE_SPEED_STRAIGHT - BASE_SPEED_TURN) + 0.5f
    );
}

/* 走行開始時に相対オドメトリの原点をリセットする */
static void reset_odometry(odometry_t *odometry)
{
    odometry->previous_left_count = 0;
    odometry->previous_right_count = 0;
    odometry->x_mm = 0.0f;
    odometry->y_mm = 0.0f;
    odometry->heading_rad = 0.0f;
    odometry->distance_mm = 0.0f;
    odometry->linear_speed_mm_s = 0.0f;
    odometry->encoder_yaw_rate_deg_s = 0.0f;
    odometry->curvature_per_m = 0.0f;
    odometry->initialized = false;
}

/* 差動二輪の車輪エンコーダによる相対位置推定。 */
static void update_odometry(
    odometry_t *odometry,
    pup_motor_t *motor_left,
    pup_motor_t *motor_right,
    float dt)
{
    const int32_t left_count = pup_motor_get_count(motor_left);
    const int32_t right_count = pup_motor_get_count(motor_right);

    const float mm_per_degree =
        (ODOM_PI * WHEEL_DIAMETER_MM) / 360.0f;

    const float left_speed_mm_s =
        (float)pup_motor_get_speed(motor_left) * mm_per_degree;
    const float right_speed_mm_s =
        (float)pup_motor_get_speed(motor_right) * mm_per_degree;

    odometry->linear_speed_mm_s =
        (left_speed_mm_s + right_speed_mm_s) * 0.5f;

    odometry->encoder_yaw_rate_deg_s =
        ((right_speed_mm_s - left_speed_mm_s) / WHEEL_TRACK_MM) *
        ODOM_RAD_TO_DEG;

    if (!odometry->initialized) {
        odometry->previous_left_count = left_count;
        odometry->previous_right_count = right_count;
        odometry->initialized = true;
        return;
    }

    const int32_t delta_left_deg =
        left_count - odometry->previous_left_count;
    const int32_t delta_right_deg =
        right_count - odometry->previous_right_count;

    odometry->previous_left_count = left_count;
    odometry->previous_right_count = right_count;

    const float delta_left_mm =
        (float)delta_left_deg * mm_per_degree;
    const float delta_right_mm =
        (float)delta_right_deg * mm_per_degree;

    const float delta_distance =
        (delta_left_mm + delta_right_mm) * 0.5f;
    const float delta_heading =
        (delta_right_mm - delta_left_mm) / WHEEL_TRACK_MM;
    const float midpoint_heading =
        odometry->heading_rad + delta_heading * 0.5f;

    odometry->x_mm += delta_distance * cosf(midpoint_heading);
    odometry->y_mm += delta_distance * sinf(midpoint_heading);
    odometry->distance_mm += delta_distance;
    odometry->heading_rad += delta_heading;

    while (odometry->heading_rad > ODOM_PI) {
        odometry->heading_rad -= 2.0f * ODOM_PI;
    }
    while (odometry->heading_rad < -ODOM_PI) {
        odometry->heading_rad += 2.0f * ODOM_PI;
    }

    const float linear_speed_m_s =
        odometry->linear_speed_mm_s / 1000.0f;

    if (fabsf(linear_speed_m_s) > 0.005f) {
        const float encoder_yaw_rad_s =
            odometry->encoder_yaw_rate_deg_s / ODOM_RAD_TO_DEG;
        odometry->curvature_per_m =
            encoder_yaw_rad_s / linear_speed_m_s;
    } else {
        odometry->curvature_per_m = 0.0f;
    }

    (void)dt;
}

/* 車輪エンコーダの旋回角速度から機体の運動状態を分類する。
 * 正の旋回角速度は左旋回、負の旋回角速度は右旋回を表す。 */
static motion_state_t classify_motion(float yaw_rate_deg_s)
{
    if (yaw_rate_deg_s > ODOM_YAW_RATE_THRESHOLD) {
        return MOTION_STATE_TURNING_LEFT;
    }
    if (yaw_rate_deg_s < -ODOM_YAW_RATE_THRESHOLD) {
        return MOTION_STATE_TURNING_RIGHT;
    }
    return MOTION_STATE_STRAIGHT;
}

static const char *line_state_name(line_state_t state)
{
    switch (state) {
    case LINE_STATE_BOTH_WHITE:   return "BOTH_WHITE";
    case LINE_STATE_LEFT_EDGE:  return "B_DARK";
    case LINE_STATE_RIGHT_EDGE: return "D_DARK";
    case LINE_STATE_BOTH_DARK:  return "BOTH_DARK";
    case LINE_STATE_TRANSITION: return "TRANSITION";
    default:                    return "UNKNOWN";
    }
}

static const char *motion_state_name(motion_state_t state)
{
    switch (state) {
    case MOTION_STATE_TURNING_LEFT:  return "LEFT";
    case MOTION_STATE_TURNING_RIGHT: return "RIGHT";
    case MOTION_STATE_STRAIGHT:
    default:                         return "STRAIGHT";
    }
}


/* Sensor state is independent of the selected route boundary. */
static line_state_t classify_line_state(float b, float d)
{
    if (b <= LINE_DARK_THRESHOLD && d <= LINE_DARK_THRESHOLD) return LINE_STATE_BOTH_DARK;
    if (b >= LINE_WHITE_THRESHOLD && d >= LINE_WHITE_THRESHOLD) return LINE_STATE_BOTH_WHITE;
    if (b <= LINE_DARK_THRESHOLD && d >= LINE_WHITE_THRESHOLD) return LINE_STATE_LEFT_EDGE;
    if (d <= LINE_DARK_THRESHOLD && b >= LINE_WHITE_THRESHOLD) return LINE_STATE_RIGHT_EDGE;
    return LINE_STATE_TRANSITION;
}

/* left boundary: B white / D dark; right boundary: B dark / D white. */
static bool correct_boundary(line_state_t state, bool right_boundary)
{
    return state == (right_boundary ? LINE_STATE_LEFT_EDGE : LINE_STATE_RIGHT_EDGE);
}

static void set_line_status_light(line_state_t state, bool right_boundary)
{
    int color_state = correct_boundary(state, right_boundary) ? 0 :
        state == LINE_STATE_BOTH_WHITE ? 1 :
        state == LINE_STATE_BOTH_DARK ? 2 :
        state == LINE_STATE_TRANSITION ? 3 : 4;
    if (line_status_light_state == color_state) return;
    switch (color_state) {
    case 0: hub_light_on_color(PBIO_COLOR_GREEN); break;
    case 1: hub_light_on_color(PBIO_COLOR_BLUE); break;
    case 2: hub_light_on_color(PBIO_COLOR_ORANGE); break;
    case 3: hub_light_on_color(PBIO_COLOR_YELLOW); break;
    default: hub_light_on_color(PBIO_COLOR_RED); break;
    }
    line_status_light_state = color_state;
}

static void clear_line_status_light(void)
{
    if (line_status_light_state != -2) {
        hub_light_off();
        line_status_light_state = -2;
    }
}

/* Hub APIから、現在押されている本体ボタンをビットマスクで取得する */
static bool read_hub_buttons(uint32_t *pressed_bits)
{
    hub_button_t pressed = 0;

    if (hub_button_is_pressed(&pressed) != PBIO_SUCCESS) {
        return false;
    }

    *pressed_bits = (uint32_t)pressed;
    return true;
}

/* フォースセンサーCの押下時間から短押し・長押しを判定する。
 * 短押しは解放時に通知し、5秒以上押し続けた場合は長押しを通知する。 */
static force_press_event_t update_force_press(
    force_press_state_t *state,
    bool touched,
    uint32_t now_hrt)
{
    if (touched) {
        if (!state->active) {
            state->active = true;
            state->start_hrt = now_hrt;
            return FORCE_PRESS_NONE;
        }

        if ((uint32_t)(now_hrt - state->start_hrt) >=
            FORCE_LONG_PRESS_US) {
            state->active = false;
            return FORCE_PRESS_LONG;
        }

        return FORCE_PRESS_NONE;
    }

    if (state->active) {
        const uint32_t duration_us =
            (uint32_t)(now_hrt - state->start_hrt);
        state->last_duration_us = duration_us;
        state->active = false;

        if (duration_us >= FORCE_LONG_PRESS_US) {
            return FORCE_PRESS_LONG;
        }
        if (duration_us >= FORCE_PRESS_DEBOUNCE_US) {
            return FORCE_PRESS_SHORT;
        }
    }

    return FORCE_PRESS_NONE;
}


static void wait_hub_buttons_released(void)
{
    const uint32_t mask =
        (uint32_t)HUB_BUTTON_LEFT |
        (uint32_t)HUB_BUTTON_CENTER |
        (uint32_t)HUB_BUTTON_RIGHT;

    while (1) {
        uint32_t pressed = 0;

        if (read_hub_buttons(&pressed) && (pressed & mask) == 0U) {
            return;
        }

        dly_tsk(MENU_POLL_US);
    }
}

/* 2つの動作モードから選択する。
 * 左ボタンでライントレース、右ボタンで校正を選び、中央ボタンで決定する。
 * ボタンの押下エッジを検出し、押しっぱなしで選択や決定が繰り返されないようにする。 */
static void show_selected_mode(app_mode_t selected)
{
    if (selected == MODE_CALIBRATION) {
        hub_display_char('C');
    } else if (selected == MODE_EXPORT_DATA) {
        hub_display_char('D');
    } else if (selected == MODE_RIGHT_BOUNDARY) {
        hub_display_char('R');
    } else {
        hub_display_char('L');
    }
}

static const char *selected_mode_name(app_mode_t selected)
{
    if (selected == MODE_CALIBRATION) {
        return "CALIBRATION";
    }
    if (selected == MODE_EXPORT_DATA) {
        return "DATA EXPORT";
    }
    return selected == MODE_RIGHT_BOUNDARY ? "RIGHT BOUNDARY" : "LEFT BOUNDARY";
}

/* 左右ボタンで3つのモードを選び、中央ボタンで決定する。
 * L=ライントレース、C=校正、D=走行・校正データのUSB出力。 */
static app_mode_t select_mode(
    pup_device_t *start_stop_sensor,
    app_mode_t initial_selection)
{
    app_mode_t selected = initial_selection;
    uint32_t previous = 0;
    force_press_state_t force_press = {0};

    clear_line_status_light();
    show_selected_mode(selected);
    syslog(LOG_NOTICE,
           "PID V2: LEFT/RIGHT=SELECT L/C/D/R, CENTER=CONFIRM");

    wait_hub_buttons_released();

    while (1) {
        uint32_t current = 0;

        if (!read_hub_buttons(&current)) {
            dly_tsk(MENU_POLL_US);
            continue;
        }

        const force_press_event_t force_event = update_force_press(
            &force_press,
            pup_force_sensor_touched(start_stop_sensor),
            (uint32_t)fch_hrt()
        );
        if (force_event == FORCE_PRESS_LONG) {
            syslog(LOG_NOTICE,
                   "PID V2: program exit by 5-second force-sensor hold");
            return MODE_EXIT;
        }

        const bool left_rise =
            (current & (uint32_t)HUB_BUTTON_LEFT) != 0U &&
            (previous & (uint32_t)HUB_BUTTON_LEFT) == 0U;

        const bool right_rise =
            (current & (uint32_t)HUB_BUTTON_RIGHT) != 0U &&
            (previous & (uint32_t)HUB_BUTTON_RIGHT) == 0U;

        const bool center_rise =
            (current & (uint32_t)HUB_BUTTON_CENTER) != 0U &&
            (previous & (uint32_t)HUB_BUTTON_CENTER) == 0U;

        if (left_rise) {
            selected = (selected == MODE_LINE_TRACE)
                ? MODE_RIGHT_BOUNDARY
                : (app_mode_t)(selected - 1);
            show_selected_mode(selected);
            if (selected != MODE_EXPORT_DATA) {
                syslog(LOG_NOTICE,
                       "PID V2: mode selected %s",
                       selected_mode_name(selected));
            }
        } else if (right_rise) {
            selected = (selected == MODE_RIGHT_BOUNDARY)
                ? MODE_LINE_TRACE
                : (app_mode_t)(selected + 1);
            show_selected_mode(selected);
            if (selected != MODE_EXPORT_DATA) {
                syslog(LOG_NOTICE,
                       "PID V2: mode selected %s",
                       selected_mode_name(selected));
            }
        }

        previous = current;

        if (center_rise) {
            if (selected != MODE_EXPORT_DATA) {
                syslog(LOG_NOTICE,
                       "PID V2: mode confirmed %s",
                       selected_mode_name(selected));
            }
            return selected;
        }

        dly_tsk(MENU_POLL_US);
    }
}

/* バイナリセマフォを空の状態にして周期通知を開始する */
static bool start_control_clock(void)
{
    if (ini_sem(SEM_CONTROL_TICK) != E_OK) {
        return false;
    }

    control_tick_overruns = 0;
    return sta_cyc(CYC_CONTROL_TICK) == E_OK;
}

/* 周期通知を停止してから、非周期処理のモード選択画面へ戻る */
static void stop_control_clock(void)
{
    stp_cyc(CYC_CONTROL_TICK);
    ini_sem(SEM_CONTROL_TICK);
}

/* 周期ハンドラ内では通知だけを行う。
 * センサー取得やモーター操作は、後で通常タスクコンテキストのMainが実行する。 */
void control_tick_handler(intptr_t exinf)
{
    if (isig_sem(SEM_CONTROL_TICK) != E_OK) {
        control_tick_overruns++;
    }
}

static bool wait_control_tick(void)
{
    return wai_sem(SEM_CONTROL_TICK) == E_OK;
}

static bool center_button_pressed(void)
{
    uint32_t pressed = 0;

    if (!read_hub_buttons(&pressed)) {
        return false;
    }

    return (pressed & (uint32_t)HUB_BUTTON_CENTER) != 0U;
}

static bool valid_reflection(int32_t value)
{
    return value >= 0 && value <= 100;
}

/* 白色面の基準値を使って、各センサーのライン横断検出器を初期化する */
static void init_sensor_crossing(
    sensor_crossing_t *sensor,
    float white_before)
{
    sensor->phase = CAL_SEEK_BLACK;
    sensor->white_before = white_before;
    sensor->threshold = white_before * CAL_BLACK_RATIO;
    sensor->black_sum = 0.0f;
    sensor->black_samples = 0;
    sensor->white_after_sum = 0.0f;
    sensor->white_after_samples = 0;
}

/* 各センサーについて、白 -> 黒 -> 白の変化を個別に追跡する */
static void update_sensor_crossing(
    sensor_crossing_t *sensor,
    int32_t reflection)
{
    const float value = (float)reflection;

    switch (sensor->phase) {
    case CAL_SEEK_BLACK:
        if (value <= sensor->threshold) {
            sensor->black_sum = value;
            sensor->black_samples = 1;
            sensor->phase = CAL_MEASURE_BLACK;
        }
        break;

    case CAL_MEASURE_BLACK:
        if (value <= sensor->threshold) {
            sensor->black_sum += value;
            sensor->black_samples++;
        } else {
            sensor->white_after_sum = value;
            sensor->white_after_samples = 1;
            sensor->phase = CAL_SEEK_WHITE_AFTER;

            if (sensor->white_after_samples >= CAL_WHITE_AFTER_SAMPLES) {
                sensor->phase = CAL_DONE;
            }
        }
        break;

    case CAL_SEEK_WHITE_AFTER:
        if (value > sensor->threshold) {
            sensor->white_after_sum += value;
            sensor->white_after_samples++;

            if (sensor->white_after_samples >= CAL_WHITE_AFTER_SAMPLES) {
                sensor->phase = CAL_DONE;
            }
        } else {
            /* 短い白色区間はノイズ、またはライン横断が完了していない状態とみなす */
            sensor->black_sum += value;
            sensor->black_samples++;
            sensor->white_after_sum = 0.0f;
            sensor->white_after_samples = 0;
            sensor->phase = CAL_MEASURE_BLACK;
        }
        break;

    case CAL_DONE:
    default:
        break;
    }
}

/* 横断する黒いラインを通過する間、低速で直進する。
 * 開始時は左右両方のセンサーが白い面を検出していなければならない。
 * 左右両方が白 -> 黒 -> 白を完了し、コントラスト検査に合格した場合だけ、
 * 校正結果を保存する。 */
static calibration_result_t calibrate_across_line(
    pup_motor_t *motor_left,
    pup_motor_t *motor_right,
    pup_device_t *sensor_left,
    pup_device_t *sensor_right,
    pup_device_t *start_stop_sensor,
    line_calibration_t *candidate)
{
    force_press_state_t force_press = {0};
    float white_left_sum = 0.0f;
    float white_right_sum = 0.0f;

    sensor_crossing_t left_crossing = {0};
    sensor_crossing_t right_crossing = {0};

    hub_display_char('W');
    syslog(LOG_NOTICE,
           "PID V2 CAL: keep both sensors on white; sampling baseline");

    /* 機体が動き始める前に、安定した白色基準値を取得する */
    for (unsigned int i = 0; i < CAL_WHITE_SAMPLES; i++) {
        if (!wait_control_tick()) {
            syslog(LOG_ERROR, "PID V2 CAL: control tick wait failed");
            return CAL_RESULT_FAILED;
        }

        if (center_button_pressed()) {
            syslog(LOG_NOTICE, "PID V2 CAL: cancelled before movement");
            return CAL_RESULT_CANCELLED;
        }

        if (update_force_press(
                &force_press,
                pup_force_sensor_touched(start_stop_sensor),
                (uint32_t)fch_hrt()) == FORCE_PRESS_LONG) {
            syslog(LOG_NOTICE,
                   "PID V2 CAL: program exit by force-sensor long press");
            return CAL_RESULT_EXIT;
        }

        int32_t left = pup_color_sensor_reflection(sensor_left);
        int32_t right = pup_color_sensor_reflection(sensor_right);

        if (!valid_reflection(left) || !valid_reflection(right) ||
            left < CAL_WHITE_MIN_REFLECT ||
            right < CAL_WHITE_MIN_REFLECT) {
            syslog(LOG_WARNING,
                   "PID V2 CAL: both sensors must start on white");
            return CAL_RESULT_FAILED;
        }

        white_left_sum += (float)left;
        white_right_sum += (float)right;
    }

    const float white_left_before =
        white_left_sum / (float)CAL_WHITE_SAMPLES;
    const float white_right_before =
        white_right_sum / (float)CAL_WHITE_SAMPLES;

    init_sensor_crossing(&left_crossing, white_left_before);
    init_sensor_crossing(&right_crossing, white_right_before);

    const int32_t start_left_count = pup_motor_get_count(motor_left);
    const int32_t start_right_count = pup_motor_get_count(motor_right);
    const uint32_t start_time = (uint32_t)fch_hrt();

    hub_display_char('C');
    syslog(LOG_NOTICE,
           "PID V2 CAL: crossing line at %d deg/s", CAL_SPEED);

    if (pup_motor_set_speed(motor_left, CAL_SPEED) != PBIO_SUCCESS ||
        pup_motor_set_speed(motor_right, CAL_SPEED) != PBIO_SUCCESS) {
        pup_motor_brake(motor_left);
        pup_motor_brake(motor_right);
        syslog(LOG_ERROR, "PID V2 CAL: failed to start motors");
        return CAL_RESULT_FAILED;
    }

    while (1) {
        if (!wait_control_tick()) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            syslog(LOG_ERROR, "PID V2 CAL: control tick wait failed");
            return CAL_RESULT_FAILED;
        }

        if (center_button_pressed()) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            syslog(LOG_NOTICE, "PID V2 CAL: cancelled by center button");
            return CAL_RESULT_CANCELLED;
        }

        if (update_force_press(
                &force_press,
                pup_force_sensor_touched(start_stop_sensor),
                (uint32_t)fch_hrt()) == FORCE_PRESS_LONG) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            syslog(LOG_NOTICE,
                   "PID V2 CAL: program exit by force-sensor long press");
            return CAL_RESULT_EXIT;
        }

        int32_t reflection_left =
            pup_color_sensor_reflection(sensor_left);
        int32_t reflection_right =
            pup_color_sensor_reflection(sensor_right);

        if (!valid_reflection(reflection_left) ||
            !valid_reflection(reflection_right)) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            syslog(LOG_ERROR, "PID V2 CAL: invalid reflection reading");
            return CAL_RESULT_FAILED;
        }

        update_sensor_crossing(&left_crossing, reflection_left);
        update_sensor_crossing(&right_crossing, reflection_right);

        /* 左右両方のセンサーが、それぞれ白色面に戻るまで走行を続ける */
        if (left_crossing.phase == CAL_DONE &&
            right_crossing.phase == CAL_DONE) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            break;
        }

        const uint32_t elapsed_us =
            (uint32_t)((uint32_t)fch_hrt() - start_time);

        const int32_t delta_left =
            pup_motor_get_count(motor_left) - start_left_count;
        const int32_t delta_right =
            pup_motor_get_count(motor_right) - start_right_count;

        const int32_t travel_left = abs_int32(delta_left);
        const int32_t travel_right = abs_int32(delta_right);
        const int32_t max_travel =
            travel_left > travel_right ? travel_left : travel_right;

        if (elapsed_us > CAL_TIMEOUT_US ||
            max_travel > CAL_MAX_TRAVEL_DEG) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            syslog(LOG_WARNING,
                   "PID V2 CAL: line not completed within travel/time limit");
            return CAL_RESULT_FAILED;
        }

        if (elapsed_us > CAL_STALL_IGNORE_US &&
            (pup_motor_is_stalled(motor_left) ||
             pup_motor_is_stalled(motor_right))) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            syslog(LOG_ERROR, "PID V2 CAL: motor stalled");
            return CAL_RESULT_FAILED;
        }

        /* エンコーダの左右同期によって、校正時の走行経路を直進に保つ */
        int32_t correction =
            (travel_left - travel_right) * CAL_SYNC_GAIN;
        correction = clamp_int32(
            correction, -CAL_SYNC_LIMIT, CAL_SYNC_LIMIT);

        const int32_t speed_left =
            clamp_int32(CAL_SPEED - correction,
                        CAL_MIN_SPEED, CAL_MAX_SPEED);
        const int32_t speed_right =
            clamp_int32(CAL_SPEED + correction,
                        CAL_MIN_SPEED, CAL_MAX_SPEED);

        if (pup_motor_set_speed(motor_left, speed_left) != PBIO_SUCCESS ||
            pup_motor_set_speed(motor_right, speed_right) != PBIO_SUCCESS) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            syslog(LOG_ERROR, "PID V2 CAL: motor command failed");
            return CAL_RESULT_FAILED;
        }
    }

    if (left_crossing.black_samples < CAL_MIN_BLACK_SAMPLES ||
        right_crossing.black_samples < CAL_MIN_BLACK_SAMPLES ||
        left_crossing.white_after_samples < CAL_WHITE_AFTER_SAMPLES ||
        right_crossing.white_after_samples < CAL_WHITE_AFTER_SAMPLES) {
        syslog(LOG_WARNING, "PID V2 CAL: insufficient line samples");
        return CAL_RESULT_FAILED;
    }

    const float white_left_after =
        left_crossing.white_after_sum /
        (float)left_crossing.white_after_samples;
    const float white_right_after =
        right_crossing.white_after_sum /
        (float)right_crossing.white_after_samples;

    const float white_left =
        (white_left_before + white_left_after) * 0.5f;
    const float white_right =
        (white_right_before + white_right_after) * 0.5f;

    const float black_left =
        left_crossing.black_sum / (float)left_crossing.black_samples;
    const float black_right =
        right_crossing.black_sum / (float)right_crossing.black_samples;

    if ((white_left - black_left) < CAL_MIN_CONTRAST ||
        (white_right - black_right) < CAL_MIN_CONTRAST) {
        syslog(LOG_WARNING,
               "PID V2 CAL: insufficient white/black contrast L=%ld R=%ld",
               (long)((white_left - black_left) * 10.0f),
               (long)((white_right - black_right) * 10.0f));
        return CAL_RESULT_FAILED;
    }

    candidate->white_left = white_left;
    candidate->black_left = black_left;
    candidate->white_right = white_right;
    candidate->black_right = black_right;
    candidate->valid = true;

    syslog(LOG_NOTICE,
           "PID V2 CAL: L W/B=%ld/%ld R W/B=%ld/%ld",
           (long)(white_left * 10.0f),
           (long)(black_left * 10.0f),
           (long)(white_right * 10.0f),
           (long)(black_right * 10.0f));

    return CAL_RESULT_OK;
}

/* 各センサーの生の反射光を共通の0～100の尺度に変換する。
 * 0 = 校正した黒、100 = 校正した白。 */
static float normalize_reflection(
    int32_t reflection,
    float black_value,
    float white_value)
{
    const float span = white_value - black_value;

    if (span < CAL_MIN_CONTRAST) {
        return 100.0f;
    }

    return clamp_float(
        ((float)reflection - black_value) * 100.0f / span,
        0.0f,
        100.0f
    );
}

/* MAIN_TASK：起動時のモード選択、移動校正、周期ライントレースを実行する */
void Main(intptr_t exinf)
{
    pup_motor_t *motor_left =
        pup_motor_get_device(LEFT_MOTOR);
    pup_motor_t *motor_right =
        pup_motor_get_device(RIGHT_MOTOR);

    if (motor_left == NULL || motor_right == NULL) {
        syslog(LOG_ERROR, "PID V2: motor not found on A/E");
        if (motor_left != NULL) {
            pup_motor_stop(motor_left);
        }
        if (motor_right != NULL) {
            pup_motor_stop(motor_right);
        }
        ext_tsk();
    }

    if (pup_motor_setup(
            motor_left, PUP_DIRECTION_COUNTERCLOCKWISE, true)
            != PBIO_SUCCESS ||
        pup_motor_setup(
            motor_right, PUP_DIRECTION_CLOCKWISE, true)
            != PBIO_SUCCESS) {
        syslog(LOG_ERROR, "PID V2: motor setup failed");
        pup_motor_stop(motor_left);
        pup_motor_stop(motor_right);
        ext_tsk();
    }

    pup_motor_brake(motor_left);
    pup_motor_brake(motor_right);

    pup_device_t *sensor_left =
        pup_color_sensor_get_device(LEFT_COLOR_SENSOR);
    pup_device_t *sensor_right =
        pup_color_sensor_get_device(RIGHT_COLOR_SENSOR);
    pup_device_t *start_stop_sensor =
        pup_force_sensor_get_device(START_STOP_SENSOR);

    if (sensor_left == NULL || sensor_right == NULL) {
        syslog(LOG_ERROR, "PID V2: color sensor not found on B/D");
        pup_motor_stop(motor_left);
        pup_motor_stop(motor_right);
        ext_tsk();
    }

    if (start_stop_sensor == NULL) {
        syslog(LOG_ERROR, "PID V2: force sensor not found on C");
        pup_motor_stop(motor_left);
        pup_motor_stop(motor_right);
        ext_tsk();
    }

    syslog(LOG_NOTICE,
           "PID V2: initialized; A/E motors, B/D colors, C force sensor");
    syslog(LOG_NOTICE,
           "PID V2: default calibration B=82.4/26.4 D=72.1/21.1");
    dly_tsk(3000000);

    syslog(LOG_NOTICE, "PID V2: encoder odometry + B/D line feedback");

    hub_speaker_set_volume(5);
    hub_speaker_play_tone(695, 200);

    /* 起動時は実測成功値をデフォルト校正値として使用する。
     * Cモードで校正に成功すれば、その実測値で上書きする。
     * 値はRAM上に保持され、電源再投入時はデフォルト値に戻る。 */
    app_mode_t next_menu_selection = MODE_LINE_TRACE;

    while (1) {
        const app_mode_t selected =
            select_mode(start_stop_sensor, next_menu_selection);

        if (selected == MODE_EXIT) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            hub_display_char('E');
            syslog(LOG_NOTICE, "PID V2: program terminated from mode menu");
            ext_tsk();
        }

        if (selected == MODE_CALIBRATION) {
            wait_hub_buttons_released();

            if (!start_control_clock()) {
                next_menu_selection = MODE_CALIBRATION;
                hub_display_char('E');
                syslog(LOG_ERROR, "PID V2 CAL: failed to start control clock");
                dly_tsk(1000000);
                continue;
            }

            line_calibration_t candidate = {0};
            const calibration_result_t result =
                calibrate_across_line(
                    motor_left,
                    motor_right,
                    sensor_left,
                    sensor_right,
                    start_stop_sensor,
                    &candidate
                );

            stop_control_clock();
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);

            if (result == CAL_RESULT_EXIT) {
                pup_motor_brake(motor_left);
                pup_motor_brake(motor_right);
                hub_display_char('E');
                syslog(LOG_NOTICE,
                       "PID V2: program terminated from calibration");
                ext_tsk();
            } else if (result == CAL_RESULT_OK) {
                next_menu_selection = MODE_LINE_TRACE;
                saved_calibration = candidate;
                saved_calibration.valid = true;

                hub_display_char('O');
                syslog(LOG_NOTICE,
                       "PID V2 CAL: success; return to mode menu");
            } else if (result == CAL_RESULT_CANCELLED) {
                next_menu_selection = MODE_CALIBRATION;
                hub_display_char('X');
                syslog(LOG_NOTICE,
                       "PID V2 CAL: cancelled; previous calibration retained");
            } else {
                next_menu_selection = MODE_CALIBRATION;
                hub_display_char('E');
                syslog(LOG_WARNING,
                       "PID V2 CAL: failed; previous calibration retained; retry available");
            }

            dly_tsk(1000000);
            wait_hub_buttons_released();
            continue;
        }

        if (selected == MODE_EXPORT_DATA) {
            wait_hub_buttons_released();
            hub_display_char('D');
            const bool exit_requested = export_latest_data(start_stop_sensor);
            if (exit_requested) {
                pup_motor_brake(motor_left);
                pup_motor_brake(motor_right);
                hub_display_char('E');
                ext_tsk();
            }
            hub_display_char('O');
            dly_tsk(500000);
            wait_hub_buttons_released();
            continue;
        }

        if (!saved_calibration.valid) {
            next_menu_selection = MODE_CALIBRATION;
            hub_display_char('C');
            syslog(LOG_WARNING,
                   "PID V2: calibration required before line tracing");
            dly_tsk(1000000);
            wait_hub_buttons_released();
            continue;
        }

        if (!start_control_clock()) {
            hub_display_char('E');
            syslog(LOG_ERROR, "PID V2: failed to start control clock");
            dly_tsk(1000000);
            wait_hub_buttons_released();
            continue;
        }

        const bool right_boundary = selected == MODE_RIGHT_BOUNDARY;
        hub_display_char(right_boundary ? 'R' : 'L');

    bool running = false;
    bool run_finished = false;
    bool pid_initialized = false;
    bool overrun_warning_sent = false;
    uint32_t run_start_hrt = 0;
    force_press_state_t force_press = {0};
    uint32_t reported_overruns = 0;

    float integral = 0.0f;
    float last_error = 0.0f;
    float derivative_filtered = 0.0f;
    bool recovery = false;
    uint32_t recovery_start = 0;
    float recovery_distance = 0.0f;
    float recovery_direction = 0.0f;
    float last_direction = 0.0f;
    unsigned int boundary_ticks = 0;

    odometry_t odometry = {0};

    uint32_t last_tick_hrt = 0;
    bool have_last_tick = false;
    unsigned int diagnostic_log_ticks = 0;

    syslog(LOG_NOTICE, "PID V2: boundary=%s; force C starts/stops", right_boundary ? "RIGHT" : "LEFT");

    while (1) {
        if (!wait_control_tick()) {
            syslog(LOG_ERROR, "PID V2: control semaphore wait failed");
            latest_run_end_reason = RUN_END_CONTROL_ERROR;
            latest_run_error_code = RUN_ERROR_CONTROL_CLOCK;
            latest_run_end_hrt = (uint32_t)fch_hrt();
            latest_run_error_elapsed_ms = running ?
                (latest_run_end_hrt - run_start_hrt) / 1000U : 0U;
            break;
        }

        /* 前回周期でセンサー異常・モーター異常を検出していた場合は、
         * 新しい走行を開始する前に制御ループを終了する。 */
        if (run_finished) {
            break;
        }

        const uint32_t now_hrt = (uint32_t)fch_hrt();
        uint32_t elapsed_us = CONTROL_PERIOD_US;

        if (have_last_tick) {
            elapsed_us = (uint32_t)(now_hrt - last_tick_hrt);
        } else {
            have_last_tick = true;
        }
        last_tick_hrt = now_hrt;

        if (elapsed_us < CONTROL_DT_MIN_US) {
            elapsed_us = CONTROL_PERIOD_US;
        }
        if (elapsed_us > CONTROL_DT_MAX_US) {
            elapsed_us = CONTROL_DT_MAX_US;
            derivative_filtered = 0.0f;
        }
        const float dt = (float)elapsed_us / 1000000.0f;

        if (!overrun_warning_sent &&
            control_tick_overruns != reported_overruns) {
            reported_overruns = control_tick_overruns;
            overrun_warning_sent = true;
            syslog(LOG_WARNING,
                   "PID V2: control tick overrun detected");
        }

        /* Cの短押しは解放時に走行開始・終了へ割り当てる。
         * 5秒以上の長押しは、どの状態からでもプログラム終了とする。 */
        const force_press_event_t force_event = update_force_press(
            &force_press,
            pup_force_sensor_touched(start_stop_sensor),
            now_hrt
        );

        if (force_event == FORCE_PRESS_LONG) {
            if (running) {
                latest_run_end_reason = RUN_END_PROGRAM_EXIT;
                latest_run_end_hrt = now_hrt;
            }

            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            stop_control_clock();
            hub_display_char('E');
            syslog(LOG_NOTICE,
                   "PID V2: program terminated by 5-second force-sensor hold");
            ext_tsk();
        }

        if (force_event == FORCE_PRESS_SHORT) {
            if (running) {
                running = false;
                run_finished = true;
                latest_run_end_reason = RUN_END_FORCE_SENSOR;
                latest_run_error_code = RUN_ERROR_FORCE_SENSOR_SHORT_PRESS;
                latest_run_error_elapsed_ms =
                    (uint32_t)(now_hrt - run_start_hrt) / 1000U;
                latest_force_press_duration_ms =
                    force_press.last_duration_us / 1000U;
                latest_run_error_left = (int32_t)latest_force_press_duration_ms;
                latest_run_error_right = 0;
                latest_run_end_hrt = now_hrt;
                pup_motor_brake(motor_left);
                pup_motor_brake(motor_right);

                integral = 0.0f;
                last_error = 0.0f;
                derivative_filtered = 0.0f;
                recovery = false;
                boundary_ticks = 0;
                last_direction = 0.0f;
                pid_initialized = false;

                syslog(LOG_NOTICE,
                       "PID V2: stopped by force sensor C; press_ms=%lu "
                       "ODO D=%ld X=%ld Y=%ld H=%ld",
                       (unsigned long)latest_force_press_duration_ms,
                       (long)odometry.distance_mm,
                       (long)odometry.x_mm,
                       (long)odometry.y_mm,
                       (long)(odometry.heading_rad * ODOM_RAD_TO_DEG));
            } else {
                running = true;
                run_finished = false;
                clear_latest_run_log();
                latest_run_valid = true;
                latest_right_boundary = right_boundary;
                latest_run_start_hrt = now_hrt;
                latest_run_end_hrt = 0;
                latest_run_end_reason = RUN_END_RUNNING;
                latest_run_error_code = RUN_ERROR_NONE;
                latest_run_error_elapsed_ms = 0;
                latest_run_error_left = 0;
                latest_run_error_right = 0;
                latest_force_press_duration_ms = 0;
                run_start_hrt = now_hrt;
                diagnostic_log_ticks = 0;

                integral = 0.0f;
                last_error = 0.0f;
                derivative_filtered = 0.0f;
                recovery = false;
                boundary_ticks = 0;
                last_direction = 0.0f;
                pid_initialized = false;
                reset_odometry(&odometry);

                syslog(LOG_NOTICE,
                       "PID V2: started by force sensor C; press_ms=%lu",
                       (unsigned long)(force_press.last_duration_us / 1000U));
            }
        }

        if (run_finished) {
            break;
        }

        /* 車輪エンコーダのみで相対位置・方位・速度を更新する。 */
        update_odometry(&odometry, motor_left, motor_right, dt);

        const motion_state_t motion_state =
            classify_motion(odometry.encoder_yaw_rate_deg_s);

        if (!running) {
            /* モーター停止中もB/Dの認識状態を表示し、走行前にラインとの位置関係を確認する。 */
            const int32_t idle_left =
                pup_color_sensor_reflection(sensor_left);
            const int32_t idle_right =
                pup_color_sensor_reflection(sensor_right);
            if (!valid_reflection(idle_left) ||
                !valid_reflection(idle_right)) {
                set_line_status_light(LINE_STATE_INVALID, right_boundary);
            } else {
                const float idle_norm_left = normalize_reflection(
                    idle_left,
                    saved_calibration.black_left,
                    saved_calibration.white_left);
                const float idle_norm_right = normalize_reflection(
                    idle_right,
                    saved_calibration.black_right,
                    saved_calibration.white_right);
                const line_state_t idle_state = classify_line_state(
                    idle_norm_left,
                    idle_norm_right);
                set_line_status_light(idle_state, right_boundary);
            }

            /* 待機中はモーターを動かさず、オドメトリだけを診断する。 */
            diagnostic_log_ticks++;
            if (diagnostic_log_ticks >= DIAGNOSTIC_LOG_INTERVAL_TICKS) {
                diagnostic_log_ticks = 0;
                syslog(LOG_NOTICE, "IDLE B=%ld D=%ld C=%d",
                       (long)idle_left, (long)idle_right,
                       force_press.active ? 1 : 0);
            }
            continue;
        }

        const int32_t reflection_left =
            pup_color_sensor_reflection(sensor_left);
        const int32_t reflection_right =
            pup_color_sensor_reflection(sensor_right);

        if (!valid_reflection(reflection_left) ||
            !valid_reflection(reflection_right)) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            running = false;
            run_finished = true;
            latest_run_end_reason = RUN_END_COLOR_SENSOR_ERROR;
            latest_run_error_code = RUN_ERROR_COLOR_SENSOR_INVALID;
            latest_run_error_elapsed_ms =
                (uint32_t)(now_hrt - run_start_hrt) / 1000U;
            latest_run_error_left = reflection_left;
            latest_run_error_right = reflection_right;
            latest_run_end_hrt = now_hrt;
            set_line_status_light(LINE_STATE_INVALID, right_boundary);
            pid_initialized = false;
            integral = 0.0f;
            derivative_filtered = 0.0f;
            syslog(LOG_ERROR, "PID V2: invalid color sensor reading");
            continue;
        }

        const float normalized_left = normalize_reflection(
            reflection_left,
            saved_calibration.black_left,
            saved_calibration.white_left
        );
        const float normalized_right = normalize_reflection(
            reflection_right,
            saved_calibration.black_right,
            saved_calibration.white_right
        );

        const line_state_t line_state = classify_line_state(normalized_left, normalized_right);
        set_line_status_light(line_state, right_boundary);
        const float sign = right_boundary ? -1.0f : 1.0f;
        float error = sign * ((normalized_left + normalized_right) * 0.5f - 50.0f);
        const bool matched = correct_boundary(line_state, right_boundary);
        const bool opposite = line_state == (right_boundary ? LINE_STATE_RIGHT_EDGE : LINE_STATE_LEFT_EDGE);
        if (matched) {
            if (boundary_ticks < RECOVERY_CONFIRM_TICKS) ++boundary_ticks;
            if (boundary_ticks >= RECOVERY_CONFIRM_TICKS && recovery) {
                recovery = false;
                pid_initialized = false;
                syslog(LOG_NOTICE, "BOUNDARY reacquired");
            }
        } else {
            boundary_ticks = 0;
        }
        if (!recovery && (opposite || line_state == LINE_STATE_BOTH_WHITE ||
                          line_state == LINE_STATE_BOTH_DARK)) {
            recovery = true;
            recovery_start = now_hrt;
            recovery_distance = odometry.distance_mm;
            recovery_direction = opposite ? last_direction : (error >= 0.0f ? 1.0f : -1.0f);
            if (recovery_direction == 0.0f) recovery_direction = sign;
            pid_initialized = false;
            syslog(LOG_NOTICE, "BOUNDARY recovery LS=%s", line_state_name(line_state));
        }
        if (!recovery && fabsf(error) > LINE_CENTER_DEADBAND) last_direction = error > 0 ? 1.0f : -1.0f;
        if (fabsf(error) < LINE_CENTER_DEADBAND) error = 0.0f;
        if (!pid_initialized) {
            last_error = error;
            derivative_filtered = 0.0f;
            integral = 0.0f;
            pid_initialized = true;
        }

        const float derivative_raw = (error - last_error) / dt;
        const float filter_alpha =
            dt / (DERIVATIVE_FILTER_TAU_S + dt);

        derivative_filtered +=
            filter_alpha * (derivative_raw - derivative_filtered);

        const float abs_error = fabsf(error);
        int32_t base_speed = calculate_base_speed(
            abs_error, fabsf(odometry.encoder_yaw_rate_deg_s));

        /* 旋回が検出された場合は、直進時の速度上限を少し下げる。
         * ライン偏差や旋回角速度がさらに大きい場合は、後続処理で追加減速する。 */
        if (motion_state != MOTION_STATE_STRAIGHT &&
            base_speed > BASE_SPEED_STRAIGHT - 15) {
            base_speed = BASE_SPEED_STRAIGHT - 15;
        }

        /* 状態が曖昧な遷移中や、幅の広いライン・交差部では減速する */
        if (recovery &&
            base_speed > LINE_BOTH_DARK_MAX_SPEED) {
            base_speed = LINE_BOTH_DARK_MAX_SPEED;
        } else if (line_state == LINE_STATE_TRANSITION &&
                   base_speed > LINE_TRANSITION_MAX_SPEED) {
            base_speed = LINE_TRANSITION_MAX_SPEED;
        }

        /* エンコーダから推定した旋回角速度をフィードバックし、機体の回転を抑える。
         * このモーター符号設定では、正のヨー角速度は左旋回を表し、
         * 正の操舵量は右方向への修正指令になる。 */
        const float odometry_damping =
            ODOM_YAW_DAMPING_GAIN * odometry.encoder_yaw_rate_deg_s;

        float steering_limit = MAX_STEERING;
        const float limit_from_min =
            (float)base_speed - (float)MIN_WHEEL_SPEED;
        const float limit_from_max =
            (float)MAX_WHEEL_SPEED - (float)base_speed;

        if (steering_limit > limit_from_min) {
            steering_limit = limit_from_min;
        }
        if (steering_limit > limit_from_max) {
            steering_limit = limit_from_max;
        }
        if (steering_limit < 0.0f) {
            steering_limit = 0.0f;
        }

        const float integral_candidate =
            clamp_float(
                integral + ((error + last_error) * 0.5f * dt),
                -(I_TERM_LIMIT / KI),
                (I_TERM_LIMIT / KI)
            );

        const float p_term = KP * error;
        const float d_term = KD * derivative_filtered;
        const float candidate_output =
            p_term + KI * integral_candidate + d_term;

        const bool pushes_high =
            candidate_output > steering_limit && error > 0.0f;
        const bool pushes_low =
            candidate_output < -steering_limit && error < 0.0f;

        /* 左右両方のセンサーが黒を検出している間は積分しない。
         * この状態では、左右位置の差を信頼できる形で取得できないため。 */
        if (!recovery &&
            !pushes_high && !pushes_low) {
            integral = integral_candidate;
        }

        float steering;
        if (recovery) {
            /* Bounded search, initially toward the last known boundary.
             * Reverse search direction once if the first sweep fails. */
            uint32_t lost_us = now_hrt - recovery_start;
            float lost_mm = fabsf(odometry.distance_mm - recovery_distance);
            if (lost_us >= RECOVERY_MAX_US || lost_mm >= RECOVERY_MAX_MM) {
                pup_motor_brake(motor_left);
                pup_motor_brake(motor_right);
                latest_run_end_reason = RUN_END_LINE_LOST;
                latest_run_end_hrt = now_hrt;
                syslog(LOG_WARNING, "BOUNDARY recovery limit; return to menu");
                break;
            }
            float direction = recovery_direction;
            if (lost_us >= RECOVERY_MAX_US / 2U) direction = -direction;
            steering = direction * steering_limit;
            if (matched) steering = 0.0f;
        } else {
            steering = clamp_float(p_term + KI * integral + d_term + odometry_damping,
                                   -steering_limit, steering_limit);
        }

        int32_t left_speed =
            (int32_t)((float)base_speed + steering + 0.5f);
        int32_t right_speed =
            (int32_t)((float)base_speed - steering + 0.5f);

        left_speed = clamp_speed(left_speed);
        right_speed = clamp_speed(right_speed);

        const pbio_error_t left_result =
            pup_motor_set_speed(motor_left, left_speed);
        const pbio_error_t right_result =
            pup_motor_set_speed(motor_right, right_speed);

        if (left_result != PBIO_SUCCESS ||
            right_result != PBIO_SUCCESS) {
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            running = false;
            run_finished = true;
            latest_run_end_reason = RUN_END_MOTOR_ERROR;
            latest_run_error_code = RUN_ERROR_MOTOR_COMMAND;
            latest_run_error_elapsed_ms =
                (uint32_t)(now_hrt - run_start_hrt) / 1000U;
            latest_run_error_left = (int32_t)left_result;
            latest_run_error_right = (int32_t)right_result;
            latest_run_end_hrt = now_hrt;
            pid_initialized = false;
            integral = 0.0f;
            derivative_filtered = 0.0f;

            odometry.initialized = false;

            syslog(LOG_ERROR,
                   "PID V2: motor command failed L=%d R=%d",
                   (int)left_result, (int)right_result);
        }

        /* 5Hzの診断ログ：センサーの生値・正規化値、ライン状態、
         * 車輪エンコーダオドメトリ、左右モーター指令値。 */
        diagnostic_log_ticks++;
        if (diagnostic_log_ticks >= DIAGNOSTIC_LOG_INTERVAL_TICKS) {
            diagnostic_log_ticks = 0;

            append_run_log(
                now_hrt,
                run_start_hrt,
                reflection_left,
                reflection_right,
                normalized_left,
                normalized_right,
                error,
                steering,
                left_speed,
                right_speed,
                &odometry,
                line_state,
                motion_state
            );

            syslog(LOG_NOTICE,
                   "TRACE t=%ld LS=%s MS=%s RAW=%ld,%ld",
                   (long)(now_hrt / 1000U),
                   line_state_name(line_state),
                   motion_state_name(motion_state),
                   (long)reflection_left,
                   (long)reflection_right);
            syslog(LOG_NOTICE,
                   "TRACE SENSE NORMx10=%ld,%ld ERRx10=%ld STEERx10=%ld",
                   (long)(normalized_left * 10.0f),
                   (long)(normalized_right * 10.0f),
                   (long)(error * 10.0f),
                   (long)(steering * 10.0f));

            syslog(LOG_NOTICE,
                   "ODOM ENC_Hx10=%ld X=%ld Y=%ld D=%ld V=%ld",
                   (long)(odometry.heading_rad * ODOM_RAD_TO_DEG * 10.0f),
                   (long)odometry.x_mm,
                   (long)odometry.y_mm,
                   (long)odometry.distance_mm,
                   (long)odometry.linear_speed_mm_s);
            syslog(LOG_NOTICE,
                   "ODOM CURVATUREx1000=%ld",
                   (long)(odometry.curvature_per_m * 1000.0f));

            syslog(LOG_NOTICE,
                   "MOTOR CMD L=%ld R=%ld",
                   (long)left_speed,
                   (long)right_speed);
        }

        last_error = error;
    }

    stop_control_clock();
    pup_motor_brake(motor_left);
    pup_motor_brake(motor_right);
    hub_display_char('M');
    syslog(LOG_NOTICE,
           "PID V2: run finished; return to menu; samples=%lu overwritten=%lu",
           (unsigned long)run_log_count,
           (unsigned long)run_log_overwritten);
    wait_hub_buttons_released();
    }
}
