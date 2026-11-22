/*
 * PID_Line_Tracer_V2.c
 *
 * SPIKE-RT v0.2.0の標準APIのみを使用します。カーネルやデバイスドライバは変更しません。
 *
 * 機体・ポート構成:
 *   左モーター       A（反時計回り）
 *   右モーター       E（時計回り）
 *   左カラーセンサー B
 *   右カラーセンサー F
 *   開始・停止用フォースセンサー C
 *
 * モード選択:
 *   Hub左・右ボタン -> ライントレース／校正／データ出力を選択
 *   Hub中央ボタン   -> 決定
 *   フォースセンサーC短押し -> 走行開始／終了
 *   フォースセンサーC長押し（2.5秒） -> プログラム終了
 *
 * 走行データ:
 *   走行中のセンサー値・オドメトリ・EKF状態・モーター指令をRAMに記録。
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
 *   - 内蔵IMUの初期化と、静止状態でのジャイロバイアス・ノイズ校正。
 *   - IMUジャイロ、エンコーダ、ライン観測を使う6状態EKF。
 *   - 走行距離、横偏差、方位誤差、旋回角速度、ジャイロバイアス、
 *     局所的な経路曲率を推定。
 *   - 将来、コースマップから曲率と目標速度を取得できる接続口を用意。
 *   - 生の観測値、3軸IMU、EKFの状態と共分散を5Hzで記録。
 *   - 直前の走行データと校正データをUSBシリアルへCSV出力。
 *   - フォースセンサーCの2.5秒長押しでモーターを停止してプログラムを終了。
 *   - IMU初期化または静止校正の品質が不十分な場合はエンコーダのみで動作。
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
#include "spike/hub/imu.h"

/* ポート割り当て */
#define LEFT_COLOR_SENSOR       PBIO_PORT_ID_B
#define RIGHT_COLOR_SENSOR      PBIO_PORT_ID_F
#define START_STOP_SENSOR       PBIO_PORT_ID_C
#define LEFT_MOTOR              PBIO_PORT_ID_A
#define RIGHT_MOTOR             PBIO_PORT_ID_E

/* 制御タイミング */
#define CONTROL_PERIOD_US       10000U
#define CONTROL_DT_MIN_US       1000U
#define CONTROL_DT_MAX_US       50000U

/* IMUの初期化と、静止状態でのジャイロバイアス校正 */
#define IMU_INIT_RETRY_COUNT    300U
#define IMU_INIT_RETRY_US       10000U
#define IMU_BIAS_SAMPLES        100U
#define IMU_BIAS_SAMPLE_US      10000U
#define IMU_BIAS_WARN_STD_DPS   3.0f
#define IMU_BIAS_MAX_ABS_DPS    8.0f
#define IMU_DISAGREE_CONFIRM_TICKS 5U

/* 診断ログの出力頻度は、100Hzの制御ループより低くする */
#define IMU_LOG_INTERVAL_TICKS  20U
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
#define FORCE_PRESS_DEBOUNCE_US 30000U
#define FORCE_LONG_PRESS_US     2500000U

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
#define IMU_ENCODER_DISAGREE_DPS 35.0f

/* 6状態の局所経路座標系EKF:
 * [s, 横偏差, 方位誤差, 旋回角速度, ジャイロバイアス, 経路曲率]
 * 単位: mm, mm, rad, rad/s, rad/s, 1/m
 */
#define EKF_STATE_DIM            6U
#define EKF_S                    0U
#define EKF_EY                   1U
#define EKF_EPSI                 2U
#define EKF_OMEGA                3U
#define EKF_GYRO_BIAS            4U
#define EKF_CURVATURE            5U
#define EKF_INNOVATION_GATE2     16.0f

/* 初期観測モデル。B/Fセンサー間隔とコースのライン幅を実測した後、
 * LINE_EKF_MM_PER_PERCENTを調整してください。 */
#define LINE_EKF_MM_PER_PERCENT  0.10f
#define LINE_EKF_MIN_DIFF        4.0f
#define LINE_EKF_CENTER_GATE_MM  20.0f
#define LINE_EKF_R_EDGE_MM2      25.0f
#define LINE_EKF_R_TRANSITION_MM2 100.0f
#define LINE_EKF_R_CENTER_MM2    400.0f
#define LINE_EKF_HEADING_LOOKAHEAD_MM 35.0f
#define LINE_EKF_MAX_SIGMA_MM    20.0f
#define LINE_EKF_MAX_SIGMA_HEADING_RAD 0.35f

/* 観測ノイズの標準偏差。起動時に測定したジャイロノイズも考慮する */
#define EKF_GYRO_STD_DPS         3.0f
#define EKF_ENCODER_STD_DPS      12.0f
#define EKF_MAP_CURVATURE_STD_PER_M 0.10f

/* 連続時間モデルにおけるプロセスノイズの強度 */
#define EKF_Q_S_MM2_PER_S        4.0f
#define EKF_Q_EY_MM2_PER_S       9.0f
#define EKF_Q_EPSI_RAD2_PER_S    0.0004f
#define EKF_Q_OMEGA_RAD2_PER_S   0.0100f
#define EKF_Q_BIAS_RAD2_PER_S    0.000001f
#define EKF_Q_CURVATURE_PER_M2_PER_S 0.05f

/* 正規化反射光の状態判定しきい値（黒=0、白=100） */
#define LINE_DARK_THRESHOLD      45.0f
#define LINE_WHITE_THRESHOLD     70.0f
#define LINE_CENTER_DIFF         12.0f
#define LINE_CENTER_DEADBAND     4.0f
#define LINE_BOTH_DARK_MAX_SPEED 65
#define LINE_TRANSITION_MAX_SPEED 95

typedef enum {
    MODE_LINE_TRACE = 0,
    MODE_CALIBRATION = 1,
    MODE_EXPORT_DATA = 2,
    MODE_EXIT = 3
} app_mode_t;

typedef enum {
    FORCE_PRESS_NONE = 0,
    FORCE_PRESS_SHORT,
    FORCE_PRESS_LONG
} force_press_event_t;

typedef struct {
    bool active;
    uint32_t start_hrt;
} force_press_state_t;

typedef enum {
    RUN_END_NONE = 0,
    RUN_END_RUNNING,
    RUN_END_FORCE_SENSOR,
    RUN_END_PROGRAM_EXIT,
    RUN_END_COLOR_SENSOR_ERROR,
    RUN_END_MOTOR_ERROR
} run_end_reason_t;

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
    LINE_STATE_CENTERED = 0,
    LINE_STATE_LEFT_EDGE,
    LINE_STATE_RIGHT_EDGE,
    LINE_STATE_BOTH_DARK,
    LINE_STATE_TRANSITION
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
    float gyro_yaw_rate_deg_s;
    float yaw_rate_difference_deg_s;
    float yaw_rate_deg_s; /* EKFによる推定値 */
    float curvature_per_m;
    bool initialized;
} odometry_t;

typedef struct {
    float x[EKF_STATE_DIM];
    float P[EKF_STATE_DIM][EKF_STATE_DIM];
    uint32_t rejected_line_updates;
    bool initialized;
} line_ekf_t;

/* 現段階では経路マップを読み込みません。将来、コースマップの機能から
 * EKFが推定した経路方向の位置に応じた曲率と速度上限を取得できるようにします。 */
typedef struct {
    bool valid;
    float curvature_per_m;
    float target_speed_deg_s;
} course_map_reference_t;

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
    int16_t gyro_yaw_x10;
    int16_t yaw_difference_x10;
    int32_t ekf_s_mm;
    int32_t ekf_ey_mm;
    int16_t ekf_heading_x1000;
    int16_t ekf_yaw_x10;
    int16_t gyro_bias_x10;
    int16_t curvature_x1000;
    uint16_t sigma_ey_mm;
    uint16_t sigma_heading_x1000;
    int32_t acceleration_mm_s2[3];
    int16_t angular_velocity_x10[3];
    uint8_t line_state;
    uint8_t motion_state;
    uint8_t line_update_accepted;
    uint8_t reserved;
} run_log_record_t;

static volatile uint32_t control_tick_overruns = 0;
static line_calibration_t saved_calibration = {0};
static run_log_record_t run_log[RUN_LOG_CAPACITY];
static uint32_t run_log_count = 0;
static uint32_t run_log_write_index = 0;
static uint32_t run_log_total_samples = 0;
static uint32_t run_log_overwritten = 0;
static bool latest_run_valid = false;
static uint32_t latest_run_start_hrt = 0;
static uint32_t latest_run_end_hrt = 0;
static run_end_reason_t latest_run_end_reason = RUN_END_NONE;

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

static uint16_t log_u16(float value)
{
    if (value < 0.0f) {
        return 0;
    }
    if (value > 65535.0f) {
        return 65535;
    }
    return (uint16_t)value;
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
    const line_ekf_t *ekf,
    const float acceleration[3],
    const float angular_velocity[3],
    line_state_t line_state,
    motion_state_t motion_state,
    bool line_update_accepted)
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
    record->gyro_yaw_x10 =
        log_i16(odometry->gyro_yaw_rate_deg_s * 10.0f);
    record->yaw_difference_x10 =
        log_i16(odometry->yaw_rate_difference_deg_s * 10.0f);
    record->ekf_s_mm = (int32_t)ekf->x[EKF_S];
    record->ekf_ey_mm = (int32_t)ekf->x[EKF_EY];
    record->ekf_heading_x1000 =
        log_i16(ekf->x[EKF_EPSI] * 1000.0f);
    record->ekf_yaw_x10 =
        log_i16(ekf->x[EKF_OMEGA] * ODOM_RAD_TO_DEG * 10.0f);
    record->gyro_bias_x10 =
        log_i16(ekf->x[EKF_GYRO_BIAS] * ODOM_RAD_TO_DEG * 10.0f);
    record->curvature_x1000 =
        log_i16(ekf->x[EKF_CURVATURE] * 1000.0f);
    record->sigma_ey_mm =
        log_u16(sqrtf(ekf->P[EKF_EY][EKF_EY]));
    record->sigma_heading_x1000 =
        log_u16(sqrtf(ekf->P[EKF_EPSI][EKF_EPSI]) * 1000.0f);
    for (unsigned int axis = 0; axis < 3; axis++) {
        record->acceleration_mm_s2[axis] =
            (int32_t)acceleration[axis];
        record->angular_velocity_x10[axis] =
            log_i16(angular_velocity[axis] * 10.0f);
    }
    record->line_state = (uint8_t)line_state;
    record->motion_state = (uint8_t)motion_state;
    record->line_update_accepted = line_update_accepted ? 1U : 0U;
    record->reserved = 0U;

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
    case RUN_END_NONE:
    default:                         return "NONE";
    }
}

/* 走行終了後に、校正値・IMU校正値・直前の走行ログをCSV形式でUSBへ出力する。
 * この処理は周期制御を停止した通常タスク上で実行する。 */
static void export_latest_data(
    const float gyro_bias[3],
    const float gyro_stddev[3],
    bool imu_available,
    bool imu_fusion_enabled)
{
    dly_tsk(300000U);

    printf("\r\n# PID_LINE_TRACER_V2_EXPORT,1\r\n");
    fflush(stdout);
    dly_tsk(20000U);
    printf("# CALIBRATION,valid=%d,white_left_x10=%ld,black_left_x10=%ld,"
           "white_right_x10=%ld,black_right_x10=%ld\r\n",
           saved_calibration.valid ? 1 : 0,
           (long)(saved_calibration.white_left * 10.0f),
           (long)(saved_calibration.black_left * 10.0f),
           (long)(saved_calibration.white_right * 10.0f),
           (long)(saved_calibration.black_right * 10.0f));
    fflush(stdout);
    dly_tsk(20000U);

    printf("# IMU,available=%d,fusion=%d,bias_x10=%ld,%ld,%ld,"
           "stddev_x10=%ld,%ld,%ld\r\n",
           imu_available ? 1 : 0,
           imu_fusion_enabled ? 1 : 0,
           (long)(gyro_bias[0] * 10.0f),
           (long)(gyro_bias[1] * 10.0f),
           (long)(gyro_bias[2] * 10.0f),
           (long)(gyro_stddev[0] * 10.0f),
           (long)(gyro_stddev[1] * 10.0f),
           (long)(gyro_stddev[2] * 10.0f));
    fflush(stdout);
    dly_tsk(20000U);

    printf("# SETTINGS,wheel_diameter_mm=%d,wheel_track_mm=%d,"
           "kp_x1000=%ld,ki_x1000=%ld,kd_x1000=%ld,"
           "line_mm_per_percent_x1000=%ld\r\n",
           (int)WHEEL_DIAMETER_MM,
           (int)WHEEL_TRACK_MM,
           (long)(KP * 1000.0f),
           (long)(KI * 1000.0f),
           (long)(KD * 1000.0f),
           (long)(LINE_EKF_MM_PER_PERCENT * 1000.0f));
    fflush(stdout);
    dly_tsk(20000U);

    printf("# RUN,valid=%d,total_samples=%lu,stored_samples=%lu,"
           "overwritten_samples=%lu,duration_ms=%lu,end_reason=%s\r\n",
           latest_run_valid ? 1 : 0,
           (unsigned long)run_log_total_samples,
           (unsigned long)run_log_count,
           (unsigned long)run_log_overwritten,
           (unsigned long)(latest_run_valid
               ? (latest_run_end_hrt - latest_run_start_hrt) / 1000U : 0U),
           run_end_reason_name(latest_run_end_reason));
    fflush(stdout);
    dly_tsk(20000U);

    printf("# LINE_STATE,0=CENTERED,1=LEFT_EDGE,2=RIGHT_EDGE,"
           "3=BOTH_DARK,4=TRANSITION\r\n");
    printf("# MOTION_STATE,0=STRAIGHT,1=TURNING_LEFT,2=TURNING_RIGHT\r\n");
    fflush(stdout);
    dly_tsk(20000U);
    printf("DATA_BEGIN\r\n");
    fflush(stdout);
    dly_tsk(20000U);
    printf("elapsed_ms,raw_left,raw_right,norm_left_x10,norm_right_x10,"
           "line_state,motion_state,line_update,error_x10,steering_x10,"
           "motor_left,motor_right,odom_x_mm,odom_y_mm,odom_distance_mm,"
           "odom_heading_x10,encoder_yaw_x10,gyro_yaw_x10,yaw_diff_x10,");
    fflush(stdout);
    dly_tsk(20000U);
    printf("ekf_s_mm,ekf_ey_mm,ekf_heading_x1000,ekf_yaw_x10,"
           "gyro_bias_x10,curvature_x1000,sigma_ey_mm,sigma_heading_x1000,"
           "accel_x_mm_s2,accel_y_mm_s2,accel_z_mm_s2,"
           "gyro_x_x10,gyro_y_x10,gyro_z_raw_x10\r\n");
    fflush(stdout);
    dly_tsk(20000U);

    if (latest_run_valid) {
        const uint32_t oldest =
            (run_log_count == RUN_LOG_CAPACITY) ? run_log_write_index : 0U;

        for (uint32_t i = 0; i < run_log_count; i++) {
            const uint32_t index = (oldest + i) % RUN_LOG_CAPACITY;
            const run_log_record_t *record = &run_log[index];

            printf("%lu,%d,%d,%d,%d,%u,%u,%u,%ld,%d,%d,%d,"
                   "%ld,%ld,%ld,%d,%d,%d,%d,%ld,%ld,%d,%d,%d,%d,%u,%u,"
                   "%ld,%ld,%ld,%d,%d,%d\r\n",
                   (unsigned long)record->elapsed_ms,
                   (int)record->raw_left,
                   (int)record->raw_right,
                   (int)record->normalized_left_x10,
                   (int)record->normalized_right_x10,
                   (unsigned int)record->line_state,
                   (unsigned int)record->motion_state,
                   (unsigned int)record->line_update_accepted,
                   (long)record->line_error_x10,
                   (int)record->steering_x10,
                   (int)record->motor_left,
                   (int)record->motor_right,
                   (long)record->odom_x_mm,
                   (long)record->odom_y_mm,
                   (long)record->odom_distance_mm,
                   (int)record->odom_heading_x10,
                   (int)record->encoder_yaw_x10,
                   (int)record->gyro_yaw_x10,
                   (int)record->yaw_difference_x10,
                   (long)record->ekf_s_mm,
                   (long)record->ekf_ey_mm,
                   (int)record->ekf_heading_x1000,
                   (int)record->ekf_yaw_x10,
                   (int)record->gyro_bias_x10,
                   (int)record->curvature_x1000,
                   (unsigned int)record->sigma_ey_mm,
                   (unsigned int)record->sigma_heading_x1000,
                   (long)record->acceleration_mm_s2[0],
                   (long)record->acceleration_mm_s2[1],
                   (long)record->acceleration_mm_s2[2],
                   (int)record->angular_velocity_x10[0],
                   (int)record->angular_velocity_x10[1],
                   (int)record->angular_velocity_x10[2]);
            fflush(stdout);
            dly_tsk(20000U);
        }
    }

    printf("DATA_END\r\n# EXPORT_COMPLETE\r\n");
    fflush(stdout);
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
    odometry->gyro_yaw_rate_deg_s = 0.0f;
    odometry->yaw_rate_difference_deg_s = 0.0f;
    odometry->yaw_rate_deg_s = 0.0f;
    odometry->curvature_per_m = 0.0f;
    odometry->initialized = false;
}

/* 標準モーターエンコーダAPIを使った差動二輪オドメトリ。
 * pup_motor_setup()で各モーターの正方向を前進方向に設定しているため、
 * 直進時は左右のエンコーダ増分がともに正になる。 */
/* 差動二輪のエンコーダオドメトリを計算する。
 *
 * - 車輪エンコーダの角度増分から前進距離と方位変化を求める。
 * - バイアス補正済みのIMU Z軸ジャイロから機体の旋回角速度を取得する。
 * - ジャイロとエンコーダは、固定比率で混ぜずにEKFの別々の観測として扱う。
 *
 * 車輪寸法はM_S6.cの値を使用する。タイヤ直径88mm、左右車輪中心間隔144mm。
 * 両方の情報源で、正のヨー角速度は左旋回を表す。
 * （右車輪の移動量 - 左車輪の移動量）/ 車輪間隔
 */
/* エンコーダのみを使った差動二輪オドメトリ。
 * EKFの診断用として、独立したデッドレコニング結果を保持する。
 * EKFでは、ラインに対する局所位置と旋回状態を推定する。 */
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

/* 角度を-piから+piの範囲に収める */
static float wrap_angle_rad(float angle)
{
    while (angle > ODOM_PI) {
        angle -= 2.0f * ODOM_PI;
    }
    while (angle < -ODOM_PI) {
        angle += 2.0f * ODOM_PI;
    }
    return angle;
}

/* ラインに対する局所座標系のEKFを初期化する。
 * 初期姿勢は、機体がライン付近にあり、ラインの接線方向におおむね
 * 向いていると仮定する。初期値は推定値なので、共分散を0にはしない。 */
static void line_ekf_reset(line_ekf_t *ekf, float initial_gyro_bias_dps)
{
    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        ekf->x[i] = 0.0f;
        for (unsigned int j = 0; j < EKF_STATE_DIM; j++) {
            ekf->P[i][j] = 0.0f;
        }
    }

    ekf->x[EKF_GYRO_BIAS] =
        initial_gyro_bias_dps / ODOM_RAD_TO_DEG;

    ekf->P[EKF_S][EKF_S] = 10000.0f; /* 標準偏差100mm */
    ekf->P[EKF_EY][EKF_EY] = 225.0f; /* 標準偏差15mm */
    ekf->P[EKF_EPSI][EKF_EPSI] = 0.12185f; /* 標準偏差20度 */
    ekf->P[EKF_OMEGA][EKF_OMEGA] = 0.12185f; /* 標準偏差20度/秒 */
    ekf->P[EKF_GYRO_BIAS][EKF_GYRO_BIAS] = 0.002742f; /* 標準偏差3度/秒 */
    ekf->P[EKF_CURVATURE][EKF_CURVATURE] = 0.25f; /* 標準偏差0.5 1/m */

    ekf->rejected_line_updates = 0;
    ekf->initialized = true;
}

/* 局所経路座標系におけるEKFの予測処理。
 * s: 経路に沿った距離、ey: 横方向の偏差、epsi: 機体の方位誤差。
 * 現段階では曲率も推定し、将来はコースマップの情報で補正できるようにする。 */
static void line_ekf_predict(
    line_ekf_t *ekf,
    float linear_speed_mm_s,
    float dt)
{
    if (!ekf->initialized || dt <= 0.0f) {
        return;
    }

    const float heading_error = ekf->x[EKF_EPSI];
    const float speed_m_s = linear_speed_mm_s / 1000.0f;
    const float path_yaw_rate =
        speed_m_s * ekf->x[EKF_CURVATURE];

    ekf->x[EKF_S] +=
        linear_speed_mm_s * cosf(heading_error) * dt;
    ekf->x[EKF_EY] +=
        linear_speed_mm_s * sinf(heading_error) * dt;
    ekf->x[EKF_EPSI] +=
        (ekf->x[EKF_OMEGA] - path_yaw_rate) * dt;
    ekf->x[EKF_EPSI] = wrap_angle_rad(ekf->x[EKF_EPSI]);

    float F[EKF_STATE_DIM][EKF_STATE_DIM] = {{0.0f}};
    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        F[i][i] = 1.0f;
    }

    F[EKF_S][EKF_EPSI] =
        -linear_speed_mm_s * sinf(heading_error) * dt;
    F[EKF_EY][EKF_EPSI] =
        linear_speed_mm_s * cosf(heading_error) * dt;
    F[EKF_EPSI][EKF_OMEGA] = dt;
    F[EKF_EPSI][EKF_CURVATURE] = -speed_m_s * dt;

    float FP[EKF_STATE_DIM][EKF_STATE_DIM] = {{0.0f}};
    float nextP[EKF_STATE_DIM][EKF_STATE_DIM] = {{0.0f}};

    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        for (unsigned int j = 0; j < EKF_STATE_DIM; j++) {
            for (unsigned int k = 0; k < EKF_STATE_DIM; k++) {
                FP[i][j] += F[i][k] * ekf->P[k][j];
            }
        }
    }

    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        for (unsigned int j = 0; j < EKF_STATE_DIM; j++) {
            for (unsigned int k = 0; k < EKF_STATE_DIM; k++) {
                nextP[i][j] += FP[i][k] * F[j][k];
            }
        }
    }

    nextP[EKF_S][EKF_S] += EKF_Q_S_MM2_PER_S * dt;
    nextP[EKF_EY][EKF_EY] += EKF_Q_EY_MM2_PER_S * dt;
    nextP[EKF_EPSI][EKF_EPSI] += EKF_Q_EPSI_RAD2_PER_S * dt;
    nextP[EKF_OMEGA][EKF_OMEGA] += EKF_Q_OMEGA_RAD2_PER_S * dt;
    nextP[EKF_GYRO_BIAS][EKF_GYRO_BIAS] += EKF_Q_BIAS_RAD2_PER_S * dt;
    nextP[EKF_CURVATURE][EKF_CURVATURE] +=
        EKF_Q_CURVATURE_PER_M2_PER_S * dt;

    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        for (unsigned int j = 0; j < EKF_STATE_DIM; j++) {
            ekf->P[i][j] = nextP[i][j];
        }
    }
}

/* スカラー観測を使ったEKFの観測更新。共分散の更新にはJoseph形式を使用する。
 * Hは観測関数のヤコビ行列、zは実測値、hは予測した観測値。 */
static bool line_ekf_scalar_update(
    line_ekf_t *ekf,
    const float H[EKF_STATE_DIM],
    float measurement,
    float predicted_measurement,
    float measurement_variance)
{
    float PHt[EKF_STATE_DIM] = {0.0f};
    float innovation_variance = measurement_variance;

    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        for (unsigned int j = 0; j < EKF_STATE_DIM; j++) {
            PHt[i] += ekf->P[i][j] * H[j];
        }
        innovation_variance += H[i] * PHt[i];
    }

    if (innovation_variance <= 1.0e-9f) {
        return false;
    }

    const float innovation = measurement - predicted_measurement;
    if ((innovation * innovation) >
        EKF_INNOVATION_GATE2 * innovation_variance) {
        return false;
    }

    float K[EKF_STATE_DIM];
    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        K[i] = PHt[i] / innovation_variance;
        ekf->x[i] += K[i] * innovation;
    }
    ekf->x[EKF_EPSI] = wrap_angle_rad(ekf->x[EKF_EPSI]);

    float A[EKF_STATE_DIM][EKF_STATE_DIM] = {{0.0f}};
    float AP[EKF_STATE_DIM][EKF_STATE_DIM] = {{0.0f}};
    float nextP[EKF_STATE_DIM][EKF_STATE_DIM] = {{0.0f}};

    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        for (unsigned int j = 0; j < EKF_STATE_DIM; j++) {
            A[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];
        }
    }

    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        for (unsigned int j = 0; j < EKF_STATE_DIM; j++) {
            for (unsigned int k = 0; k < EKF_STATE_DIM; k++) {
                AP[i][j] += A[i][k] * ekf->P[k][j];
            }
        }
    }

    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        for (unsigned int j = 0; j < EKF_STATE_DIM; j++) {
            for (unsigned int k = 0; k < EKF_STATE_DIM; k++) {
                nextP[i][j] += AP[i][k] * A[j][k];
            }
            nextP[i][j] += K[i] * measurement_variance * K[j];
        }
    }

    /* 数値計算による非対称を補正し、対角成分の分散が正になるようにする */
    for (unsigned int i = 0; i < EKF_STATE_DIM; i++) {
        for (unsigned int j = i; j < EKF_STATE_DIM; j++) {
            const float symmetric =
                0.5f * (nextP[i][j] + nextP[j][i]);
            ekf->P[i][j] = symmetric;
            ekf->P[j][i] = symmetric;
        }
        if (ekf->P[i][i] < 1.0e-9f) {
            ekf->P[i][i] = 1.0e-9f;
        }
    }

    return true;
}

/* IMU観測：生のジャイロZ値 = 機体の旋回角速度 + ジャイロバイアス */
static bool line_ekf_update_gyro(
    line_ekf_t *ekf,
    float gyro_z_raw_dps,
    float gyro_noise_std_dps)
{
    const float H[EKF_STATE_DIM] = {
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f
    };
    const float measurement =
        gyro_z_raw_dps / ODOM_RAD_TO_DEG;
    const float predicted =
        ekf->x[EKF_OMEGA] + ekf->x[EKF_GYRO_BIAS];
    const float sigma_dps =
        gyro_noise_std_dps > EKF_GYRO_STD_DPS
            ? gyro_noise_std_dps : EKF_GYRO_STD_DPS;
    const float sigma_rad_s =
        sigma_dps / ODOM_RAD_TO_DEG;

    return line_ekf_scalar_update(
        ekf, H, measurement, predicted,
        sigma_rad_s * sigma_rad_s
    );
}

/* エンコーダ観測：左右車輪速度の差から、独立した旋回角速度の観測値を求める。
 * 車輪とジャイロの不一致が大きい場合は、観測分散を大きくして影響を弱める。 */
static bool line_ekf_update_encoder(
    line_ekf_t *ekf,
    float encoder_yaw_rate_dps,
    float disagreement_dps)
{
    const float H[EKF_STATE_DIM] = {
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f
    };
    const float measurement =
        encoder_yaw_rate_dps / ODOM_RAD_TO_DEG;
    const float predicted = ekf->x[EKF_OMEGA];

    float sigma_dps = EKF_ENCODER_STD_DPS;
    if (disagreement_dps > 15.0f) {
        const float scale =
            1.0f + clamp_float(disagreement_dps / 30.0f, 0.0f, 3.0f);
        sigma_dps *= scale;
    }

    const float sigma_rad_s =
        sigma_dps / ODOM_RAD_TO_DEG;

    return line_ekf_scalar_update(
        ekf, H, measurement, predicted,
        sigma_rad_s * sigma_rad_s
    );
}

/* 校正済みの左右反射光センサーによるライン観測。
 * 左右の反射光差を、暫定的な横方向偏差に変換する。
 * センサー間隔とライン幅を実測し、幾何学的な観測モデルを追加するまでは、
 * 調整可能な疑似観測として扱う。
 *
 * 両方黒の場合は状態が曖昧なので観測を棄却する。
 * 両方白の場合は、予測した横偏差がセンサー間に近いときだけ、
 * 信頼度の低い「偏差ゼロ」の観測として扱う。 */
static bool line_ekf_update_line(
    line_ekf_t *ekf,
    float normalized_left,
    float normalized_right,
    line_state_t line_state)
{
    const float H[EKF_STATE_DIM] = {
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    const float reflectance_difference =
        normalized_left - normalized_right;
    const float measurement =
        reflectance_difference * LINE_EKF_MM_PER_PERCENT;

    float variance;

    if (line_state == LINE_STATE_BOTH_DARK) {
        return false;
    } else if (line_state == LINE_STATE_LEFT_EDGE ||
               line_state == LINE_STATE_RIGHT_EDGE) {
        variance = LINE_EKF_R_EDGE_MM2;
    } else if (line_state == LINE_STATE_TRANSITION) {
        if (fabsf(reflectance_difference) < LINE_EKF_MIN_DIFF) {
            return false;
        }
        variance = LINE_EKF_R_TRANSITION_MM2;
    } else if (line_state == LINE_STATE_CENTERED) {
        if (fabsf(ekf->x[EKF_EY]) > LINE_EKF_CENTER_GATE_MM) {
            return false;
        }
        variance = LINE_EKF_R_CENTER_MM2;
    } else {
        return false;
    }

    const bool accepted = line_ekf_scalar_update(
        ekf, H, measurement, ekf->x[EKF_EY], variance
    );

    if (!accepted) {
        ekf->rejected_line_updates++;
    }

    return accepted;
}

/* 将来のマップ接続口。読み込んだ経路から、EKFが推定した経路方向の位置に
 * 対応する局所曲率と目標速度を取得できるようにする。 */
static course_map_reference_t course_map_get_reference(float along_track_mm)
{
    course_map_reference_t reference = {0};
    (void)along_track_mm;
    reference.valid = false; /* このバージョンではコースファイルを読み込んでいない */
    reference.curvature_per_m = 0.0f;
    reference.target_speed_deg_s = 0.0f;
    return reference;
}

static bool line_ekf_update_map_curvature(
    line_ekf_t *ekf,
    float curvature_per_m)
{
    const float H[EKF_STATE_DIM] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
    };
    const float variance =
        EKF_MAP_CURVATURE_STD_PER_M * EKF_MAP_CURVATURE_STD_PER_M;

    return line_ekf_scalar_update(
        ekf, H, curvature_per_m, ekf->x[EKF_CURVATURE], variance
    );
}

/* EKFの旋回角速度推定値から機体の運動状態を分類する。
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
    case LINE_STATE_CENTERED:   return "CENTER";
    case LINE_STATE_LEFT_EDGE:  return "LEFT";
    case LINE_STATE_RIGHT_EDGE: return "RIGHT";
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


/* 左右のセンサーでラインを挟む配置を想定する。
 * ラインが中央にある場合、通常は左右とも白で反射光がほぼ等しくなる。
 * 左右どちらかが黒を検出した場合、ラインがそのセンサー側に寄ったと判断する。
 * 両方黒の場合は交差点または幅の広いラインの可能性があり、
 * 左右位置を判断する確かな情報としては扱わない。 */
static line_state_t classify_line_state(
    float normalized_left,
    float normalized_right,
    float error)
{
    const bool left_dark = normalized_left <= LINE_DARK_THRESHOLD;
    const bool right_dark = normalized_right <= LINE_DARK_THRESHOLD;
    const bool left_white = normalized_left >= LINE_WHITE_THRESHOLD;
    const bool right_white = normalized_right >= LINE_WHITE_THRESHOLD;

    if (left_dark && right_dark) {
        return LINE_STATE_BOTH_DARK;
    }
    if (left_dark && right_white) {
        return LINE_STATE_LEFT_EDGE;
    }
    if (right_dark && left_white) {
        return LINE_STATE_RIGHT_EDGE;
    }
    if (left_white && right_white &&
        fabsf(error) <= LINE_CENTER_DIFF) {
        return LINE_STATE_CENTERED;
    }

    return LINE_STATE_TRANSITION;
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
 * 短押しは解放時に通知し、2.5秒以上押し続けた場合は長押しを通知する。 */
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
    return "LINE TRACE";
}

/* 左右ボタンで3つのモードを選び、中央ボタンで決定する。
 * L=ライントレース、C=校正、D=走行・校正データのUSB出力。 */
static app_mode_t select_mode(pup_device_t *start_stop_sensor)
{
    app_mode_t selected = MODE_LINE_TRACE;
    uint32_t previous = 0;
    force_press_state_t force_press = {0};

    show_selected_mode(selected);
    syslog(LOG_NOTICE,
           "PID V2: LEFT/RIGHT=SELECT L/C/D, CENTER=CONFIRM");

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
                   "PID V2: program exit by 2.5-second force-sensor hold");
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
                ? MODE_EXPORT_DATA
                : (app_mode_t)(selected - 1);
            show_selected_mode(selected);
            if (selected != MODE_EXPORT_DATA) {
                syslog(LOG_NOTICE,
                       "PID V2: mode selected %s",
                       selected_mode_name(selected));
            }
        } else if (right_rise) {
            selected = (selected == MODE_EXPORT_DATA)
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

static bool initialize_imu(float gyro_bias[3], float gyro_stddev[3])
{
    pbio_error_t result = PBIO_ERROR_AGAIN;

    /* IMUドライバの初期化はPybricksタスク内で非同期に進む。
     * 通常タスクコンテキストからPBIO_ERROR_AGAINを再試行する。 */
    for (unsigned int i = 0; i < IMU_INIT_RETRY_COUNT; i++) {
        result = hub_imu_init();

        if (result == PBIO_SUCCESS) {
            break;
        }

        if (result != PBIO_ERROR_AGAIN) {
            return false;
        }

        dly_tsk(IMU_INIT_RETRY_US);
    }

    if (result != PBIO_SUCCESS) {
        return false;
    }

    /* ジャイロのゼロ点バイアスを推定している間、機体を静止させておく */
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float sum_sq[3] = {0.0f, 0.0f, 0.0f};

    for (unsigned int i = 0; i < IMU_BIAS_SAMPLES; i++) {
        float angular_velocity[3];

        dly_tsk(IMU_BIAS_SAMPLE_US);
        hub_imu_get_angular_velocity(angular_velocity);

        for (unsigned int axis = 0; axis < 3; axis++) {
            if (!isfinite(angular_velocity[axis])) {
                return false;
            }

            sum[axis] += angular_velocity[axis];
            sum_sq[axis] += angular_velocity[axis] *
                            angular_velocity[axis];
        }
    }

    for (unsigned int axis = 0; axis < 3; axis++) {
        const float mean = sum[axis] / (float)IMU_BIAS_SAMPLES;
        float variance =
            (sum_sq[axis] / (float)IMU_BIAS_SAMPLES) - mean * mean;

        if (variance < 0.0f) {
            variance = 0.0f;
        }

        gyro_bias[axis] = mean;
        gyro_stddev[axis] = sqrtf(variance);
    }

    return true;
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
        syslog(LOG_ERROR, "PID V2: color sensor not found on B/F");
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
           "PID V2: initialized; A/E motors, B/F colors, C force sensor");
    dly_tsk(3000000);

    /* 内蔵LSM6DS3TR-C IMUを初期化し、静止状態でジャイロバイアスを推定する。
     * IMUを利用できない場合も、エンコーダだけを使ってライントレースを続行できる。 */
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
    float gyro_stddev[3] = {0.0f, 0.0f, 0.0f};
    const bool imu_available = initialize_imu(gyro_bias, gyro_stddev);
    bool imu_fusion_enabled = imu_available;

    if (imu_available) {
        syslog(LOG_NOTICE,
               "IMU bias x10 dps X=%ld Y=%ld Z=%ld",
               (long)(gyro_bias[0] * 10.0f),
               (long)(gyro_bias[1] * 10.0f),
               (long)(gyro_bias[2] * 10.0f));

        syslog(LOG_NOTICE,
               "IMU noise x10 dps X=%ld Y=%ld Z=%ld",
               (long)(gyro_stddev[0] * 10.0f),
               (long)(gyro_stddev[1] * 10.0f),
               (long)(gyro_stddev[2] * 10.0f));

        if (gyro_stddev[2] > IMU_BIAS_WARN_STD_DPS ||
            fabsf(gyro_bias[2]) > IMU_BIAS_MAX_ABS_DPS) {
            imu_fusion_enabled = false;
            syslog(LOG_WARNING,
                   "IMU: gyro calibration quality low; encoder-only fusion fallback");
        }
    } else {
        syslog(LOG_WARNING,
               "IMU unavailable; continuing with encoder-only diagnostics");
    }

    hub_speaker_set_volume(100);
    hub_speaker_play_tone(695, 200);

    /* 校正値はモード選択画面に戻った後もRAM上に保持される。
     * 電源を入れ直した場合は、再度校正する必要がある。 */
    while (1) {
        const app_mode_t selected = select_mode(start_stop_sensor);

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
                saved_calibration = candidate;
                saved_calibration.valid = true;

                hub_display_char('O');
                syslog(LOG_NOTICE,
                       "PID V2 CAL: success; return to mode menu");
            } else if (result == CAL_RESULT_CANCELLED) {
                hub_display_char('X');
                syslog(LOG_NOTICE,
                       "PID V2 CAL: cancelled; previous calibration retained");
            } else {
                hub_display_char('E');
                syslog(LOG_WARNING,
                       "PID V2 CAL: failed; previous calibration retained");
            }

            dly_tsk(1000000);
            wait_hub_buttons_released();
            continue;
        }

        if (selected == MODE_EXPORT_DATA) {
            wait_hub_buttons_released();
            hub_display_char('D');
            export_latest_data(
                gyro_bias, gyro_stddev, imu_available, imu_fusion_enabled);
            hub_display_char('O');
            dly_tsk(500000);
            wait_hub_buttons_released();
            continue;
        }

        if (!saved_calibration.valid) {
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

        hub_display_char('L');

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
    float last_steering = 0.0f;

    odometry_t odometry = {0};
    line_ekf_t line_ekf = {0};
    line_ekf_reset(
        &line_ekf,
        imu_fusion_enabled ? gyro_bias[2] : 0.0f
    );

    uint32_t last_tick_hrt = 0;
    bool have_last_tick = false;
    unsigned int imu_log_ticks = 0;
    unsigned int imu_disagree_ticks = 0;
    bool imu_disagree_reported = false;

    syslog(LOG_NOTICE,
           "PID V2: line trace ready; force sensor C starts/stops");

    while (1) {
        if (!wait_control_tick()) {
            syslog(LOG_ERROR, "PID V2: control semaphore wait failed");
            stp_cyc(CYC_CONTROL_TICK);
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            ext_tsk();
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
         * 2.5秒以上の長押しは、どの状態からでもプログラム終了とする。 */
        const force_event = update_force_press(
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
                   "PID V2: program terminated by 2.5-second force-sensor hold");
            ext_tsk();
        }

        if (force_event == FORCE_PRESS_SHORT) {
            if (running) {
                running = false;
                run_finished = true;
                latest_run_end_reason = RUN_END_FORCE_SENSOR;
                latest_run_end_hrt = now_hrt;
                pup_motor_brake(motor_left);
                pup_motor_brake(motor_right);

                integral = 0.0f;
                last_error = 0.0f;
                derivative_filtered = 0.0f;
                last_steering = 0.0f;
                pid_initialized = false;

                syslog(LOG_NOTICE,
                       "PID V2: stopped by short press on force sensor C; "
                       "ODO D=%ld X=%ld Y=%ld H=%ld",
                       (long)odometry.distance_mm,
                       (long)odometry.x_mm,
                       (long)odometry.y_mm,
                       (long)(odometry.heading_rad * ODOM_RAD_TO_DEG));
            } else {
                running = true;
                run_finished = false;
                clear_latest_run_log();
                latest_run_valid = true;
                latest_run_start_hrt = now_hrt;
                latest_run_end_hrt = 0;
                latest_run_end_reason = RUN_END_RUNNING;
                run_start_hrt = now_hrt;
                imu_log_ticks = 0;

                integral = 0.0f;
                last_error = 0.0f;
                derivative_filtered = 0.0f;
                last_steering = 0.0f;
                pid_initialized = false;
                reset_odometry(&odometry);
                line_ekf_reset(
                    &line_ekf,
                    imu_fusion_enabled ? gyro_bias[2] : 0.0f
                );

                syslog(LOG_NOTICE,
                       "PID V2: started by short press on force sensor C");
            }
        }

        if (run_finished) {
            break;
        }

        /* IMUドライバが保持している最新の測定値を取得する。
         * 標準SPIKE-RT APIはドライバのデータを返すため、この制御タスクから
         * アプリケーション側のI2C通信を新たに開始するわけではない。
         * ジャイロバイアスはEKFの推定状態なので、生のジャイロZ値を保持する。 */
        float acceleration[3] = {0.0f, 0.0f, 0.0f};
        float angular_velocity[3] = {0.0f, 0.0f, 0.0f};
        float gyro_z_raw_dps = 0.0f;

        if (imu_available) {
            hub_imu_get_acceleration(acceleration);
            hub_imu_get_angular_velocity(angular_velocity);
            gyro_z_raw_dps = angular_velocity[2];
        }

        /* 車輪エンコーダによるオドメトリは、比較用として独立に保持する */
        update_odometry(&odometry, motor_left, motor_right, dt);

        /* EKFの予測にはエンコーダから求めた前進速度と、直前の旋回角速度・曲率状態を使う。
         * コースファイルが追加されるまではマップ接続口が無効なので、
         * EKF自身が局所的な曲率を推定する。 */
        line_ekf_predict(&line_ekf, odometry.linear_speed_mm_s, dt);

        const course_map_reference_t map_reference =
            course_map_get_reference(line_ekf.x[EKF_S]);

        if (map_reference.valid) {
            line_ekf_update_map_curvature(
                &line_ekf, map_reference.curvature_per_m);
        }

        /* エンコーダによる旋回観測は常に利用できる。
         * ジャイロと大きく食い違う場合は、固定比率で混ぜるのではなく、
         * エンコーダ観測の分散を大きくして影響を弱める。 */
        const float gyro_corrected_for_compare =
            gyro_z_raw_dps -
            line_ekf.x[EKF_GYRO_BIAS] * ODOM_RAD_TO_DEG;
        const float yaw_disagreement_dps = imu_available
            ? fabsf(gyro_corrected_for_compare -
                    odometry.encoder_yaw_rate_deg_s)
            : 0.0f;

        line_ekf_update_encoder(
            &line_ekf,
            odometry.encoder_yaw_rate_deg_s,
            imu_fusion_enabled ? yaw_disagreement_dps : 0.0f
        );

        if (imu_fusion_enabled) {
            line_ekf_update_gyro(
                &line_ekf,
                gyro_z_raw_dps,
                gyro_stddev[2]
            );
        }

        odometry.gyro_yaw_rate_deg_s = imu_available
            ? gyro_z_raw_dps -
              line_ekf.x[EKF_GYRO_BIAS] * ODOM_RAD_TO_DEG
            : 0.0f;
        odometry.yaw_rate_difference_deg_s =
            odometry.gyro_yaw_rate_deg_s -
            odometry.encoder_yaw_rate_deg_s;
        odometry.yaw_rate_deg_s =
            line_ekf.x[EKF_OMEGA] * ODOM_RAD_TO_DEG;

        /* 不一致が継続した場合は、原因調査用の警告を出力する */
        if (imu_fusion_enabled &&
            fabsf(odometry.yaw_rate_difference_deg_s) >
                IMU_ENCODER_DISAGREE_DPS) {
            if (imu_disagree_ticks < IMU_DISAGREE_CONFIRM_TICKS) {
                imu_disagree_ticks++;
            }
            if (imu_disagree_ticks >= IMU_DISAGREE_CONFIRM_TICKS &&
                !imu_disagree_reported) {
                imu_disagree_reported = true;
                syslog(LOG_WARNING,
                       "IMU/ENC yaw mismatch: gyro=%ld encoder=%ld dps",
                       (long)odometry.gyro_yaw_rate_deg_s,
                       (long)odometry.encoder_yaw_rate_deg_s);
            }
        } else {
            imu_disagree_ticks = 0;
            imu_disagree_reported = false;
        }

        const motion_state_t motion_state =
            classify_motion(odometry.yaw_rate_deg_s);

        if (!running) {
            /* 待機中の診断ログを使い、モーターを動かさずに静止時の測定と
             * 手で機体を回したときのジャイロ符号を確認できるようにする。 */
            imu_log_ticks++;
            if (imu_log_ticks >= IMU_LOG_INTERVAL_TICKS) {
                imu_log_ticks = 0;
                syslog(LOG_NOTICE,
                       "IDLE IMU=%d USE=%d GZx10=%ld EZx10=%ld DIFFx10=%ld FZx10=%ld BGx10=%ld",
                       imu_available ? 1 : 0,
                       imu_fusion_enabled ? 1 : 0,
                       (long)(odometry.gyro_yaw_rate_deg_s * 10.0f),
                       (long)(odometry.encoder_yaw_rate_deg_s * 10.0f),
                       (long)(odometry.yaw_rate_difference_deg_s * 10.0f),
                       (long)(odometry.yaw_rate_deg_s * 10.0f),
                       (long)(line_ekf.x[EKF_GYRO_BIAS] *
                              ODOM_RAD_TO_DEG * 10.0f));
                syslog(LOG_NOTICE,
                       "EKF IDLE S=%ld EY=%ld EPSIx1000=%ld Kx1000=%ld SIG_EY=%ld",
                       (long)line_ekf.x[EKF_S],
                       (long)line_ekf.x[EKF_EY],
                       (long)(line_ekf.x[EKF_EPSI] * 1000.0f),
                       (long)(line_ekf.x[EKF_CURVATURE] * 1000.0f),
                       (long)sqrtf(line_ekf.P[EKF_EY][EKF_EY]));
                (void)acceleration;
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
            latest_run_end_hrt = now_hrt;
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

        /* 正規化した左右反射光の差を、ラインの横方向位置に関する観測値として使う。
         * EKFは両方黒の曖昧な状態を棄却し、両方白の場合は弱い観測更新を行う。 */
        const float sensor_difference =
            normalized_left - normalized_right;
        const line_state_t line_state =
            classify_line_state(
                normalized_left, normalized_right, sensor_difference);

        const bool line_observation_accepted =
            line_ekf_update_line(
                &line_ekf,
                normalized_left,
                normalized_right,
                line_state
            );

        /* EKFが推定した横偏差と方位誤差を操舵に使う。
         * 方位誤差に先読み距離を掛け、機体が局所的なライン接線方向に対して
         * どちらを向いているかを操舵量に反映する。
         * 既存PIDの入力尺度に合わせるため、mm単位の偏差を反射光差の尺度に戻す。 */
        float error =
            (line_ekf.x[EKF_EY] +
             LINE_EKF_HEADING_LOOKAHEAD_MM * line_ekf.x[EKF_EPSI]) /
            LINE_EKF_MM_PER_PERCENT;

        if (line_state == LINE_STATE_CENTERED &&
            fabsf(error) < LINE_CENTER_DEADBAND) {
            error = 0.0f;
        }

        if (!pid_initialized) {
            last_error = error;
            derivative_filtered = 0.0f;
            integral = 0.0f;
            last_steering = 0.0f;
            pid_initialized = true;
        }

        const float derivative_raw = (error - last_error) / dt;
        const float filter_alpha =
            dt / (DERIVATIVE_FILTER_TAU_S + dt);

        derivative_filtered +=
            filter_alpha * (derivative_raw - derivative_filtered);

        const float abs_error = fabsf(error);
        int32_t base_speed = calculate_base_speed(
            abs_error, fabsf(odometry.yaw_rate_deg_s));

        const float cross_track_sigma =
            sqrtf(line_ekf.P[EKF_EY][EKF_EY]);
        const float heading_sigma =
            sqrtf(line_ekf.P[EKF_EPSI][EKF_EPSI]);

        if ((cross_track_sigma > LINE_EKF_MAX_SIGMA_MM ||
             heading_sigma > LINE_EKF_MAX_SIGMA_HEADING_RAD) &&
            base_speed > BASE_SPEED_TURN) {
            base_speed = BASE_SPEED_TURN;
        }

        if (map_reference.valid &&
            map_reference.target_speed_deg_s > 0.0f &&
            base_speed > (int32_t)map_reference.target_speed_deg_s) {
            base_speed = (int32_t)map_reference.target_speed_deg_s;
        }

        /* 旋回が検出された場合は、直進時の速度上限を少し下げる。
         * ライン偏差や旋回角速度がさらに大きい場合は、後続処理で追加減速する。 */
        if (motion_state != MOTION_STATE_STRAIGHT &&
            base_speed > BASE_SPEED_STRAIGHT - 15) {
            base_speed = BASE_SPEED_STRAIGHT - 15;
        }

        /* 状態が曖昧な遷移中や、幅の広いライン・交差部では減速する */
        if (line_state == LINE_STATE_BOTH_DARK &&
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
            ODOM_YAW_DAMPING_GAIN * odometry.yaw_rate_deg_s;

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
        if (line_state != LINE_STATE_BOTH_DARK &&
            !pushes_high && !pushes_low) {
            integral = integral_candidate;
        }

        float steering;
        if (line_state == LINE_STATE_BOTH_DARK) {
            /* 幅の広いラインや交差部では直前の操舵方向を引き継ぎ、
             * エンコーダによる旋回角速度のフィードバックで回転を穏やかに抑える。 */
            steering = clamp_float(
                last_steering + odometry_damping,
                -steering_limit,
                steering_limit
            );
        } else {
            steering = clamp_float(
                p_term + KI * integral + d_term + odometry_damping,
                -steering_limit,
                steering_limit
            );
        }


        int32_t left_speed =
            (int32_t)((float)base_speed + steering + 0.5f);
        int32_t right_speed =
            (int32_t)((float)base_speed - steering + 0.5f);

        left_speed = clamp_speed(left_speed);
        right_speed = clamp_speed(right_speed);
        last_steering = steering;

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
            latest_run_end_hrt = now_hrt;
            pid_initialized = false;
            integral = 0.0f;
            derivative_filtered = 0.0f;
            last_steering = 0.0f;
            odometry.initialized = false;

            syslog(LOG_ERROR,
                   "PID V2: motor command failed L=%d R=%d",
                   (int)left_result, (int)right_result);
        }

        /* 5Hzの診断ログ：センサーの生値、ライン・運動状態、
         * エンコーダとジャイロの旋回量、EKFの推定状態、左右モーター指令値。 */
        imu_log_ticks++;
        if (imu_log_ticks >= IMU_LOG_INTERVAL_TICKS) {
            imu_log_ticks = 0;

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
                &line_ekf,
                acceleration,
                angular_velocity,
                line_state,
                motion_state,
                line_observation_accepted
            );

            syslog(LOG_NOTICE,
                   "TRACE t=%ld LS=%s MS=%s RAW=%ld,%ld NORMx10=%ld,%ld ERRx10=%ld STEERx10=%ld",
                   (long)(now_hrt / 1000U),
                   line_state_name(line_state),
                   motion_state_name(motion_state),
                   (long)reflection_left,
                   (long)reflection_right,
                   (long)(normalized_left * 10.0f),
                   (long)(normalized_right * 10.0f),
                   (long)(error * 10.0f),
                   (long)(steering * 10.0f));

            syslog(LOG_NOTICE,
                   "ODOM ENC_Hx10=%ld X=%ld Y=%ld D=%ld V=%ld Kx1000=%ld",
                   (long)(odometry.heading_rad * ODOM_RAD_TO_DEG * 10.0f),
                   (long)odometry.x_mm,
                   (long)odometry.y_mm,
                   (long)odometry.distance_mm,
                   (long)odometry.linear_speed_mm_s,
                   (long)(odometry.curvature_per_m * 1000.0f));

            syslog(LOG_NOTICE,
                   "EKF S=%ld EY=%ld EPSIx1000=%ld WZx10=%ld BGx10=%ld Kx1000=%ld SIGY=%ld SIGTHx1000=%ld LUP=%d",
                   (long)line_ekf.x[EKF_S],
                   (long)line_ekf.x[EKF_EY],
                   (long)(line_ekf.x[EKF_EPSI] * 1000.0f),
                   (long)(line_ekf.x[EKF_OMEGA] * ODOM_RAD_TO_DEG * 10.0f),
                   (long)(line_ekf.x[EKF_GYRO_BIAS] * ODOM_RAD_TO_DEG * 10.0f),
                   (long)(line_ekf.x[EKF_CURVATURE] * 1000.0f),
                   (long)sqrtf(line_ekf.P[EKF_EY][EKF_EY]),
                   (long)(sqrtf(line_ekf.P[EKF_EPSI][EKF_EPSI]) * 1000.0f),
                   line_observation_accepted ? 1 : 0);

            syslog(LOG_NOTICE,
                   "MOTOR CMD L=%ld R=%ld ACCx10=%ld,%ld,%ld GYROx10=%ld,%ld",
                   (long)left_speed,
                   (long)right_speed,
                   (long)(acceleration[0] * 10.0f),
                   (long)(acceleration[1] * 10.0f),
                   (long)(acceleration[2] * 10.0f),
                   (long)(angular_velocity[0] * 10.0f),
                   (long)(angular_velocity[1] * 10.0f));
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
