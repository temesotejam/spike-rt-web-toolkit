/*
 * PID_Line_Tracer_V2.c
 *
 * Standard SPIKE-RT v0.2.0 APIs only. No kernel or device-driver changes.
 *
 * Hardware:
 *   Left motor       A (counterclockwise)
 *   Right motor      E (clockwise)
 *   Left color       B
 *   Right color      F
 *   Start/stop force C
 *
 * Mode selection:
 *   Hub LEFT button   -> LINE TRACE
 *   Hub RIGHT button  -> CALIBRATION
 *   Hub CENTER button -> Confirm
 *
 * Calibration geometry:
 *   Place both color sensors on the white surface, with the robot facing
 *   straight toward a black line that crosses the direction of travel.
 *   The robot drives forward across the transverse line:
 *       white -> black -> white
 *   Each sensor is calibrated independently from its measured white/black
 *   reflectance. After success, the mode menu is shown again.
 *
 * V2:
 *   - 10 ms cyclic notification and semaphore-based control wake-up.
 *   - Actual elapsed-time measurement with fch_hrt().
 *   - Moving white/black/white calibration across the line.
 *   - Per-sensor reflectance normalization.
 *   - Filtered derivative and conditional integral anti-windup.
 *   - Adaptive base speed and bounded wheel commands.
 *   - Built-in IMU initialization and stationary gyro-bias/noise calibration.
 *   - 85/15 IMU-gyro/encoder yaw fusion for heading, motion state and steering.
 *   - 5 Hz logs compare both yaw sources and record fused pose/control state.
 *   - Encoder-only fallback if IMU initialization or stationary calibration
 *     quality is inadequate.
 */

#include <kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
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

/* Port assignment */
#define LEFT_COLOR_SENSOR       PBIO_PORT_ID_B
#define RIGHT_COLOR_SENSOR      PBIO_PORT_ID_F
#define START_STOP_SENSOR       PBIO_PORT_ID_C
#define LEFT_MOTOR              PBIO_PORT_ID_A
#define RIGHT_MOTOR             PBIO_PORT_ID_E

/* Control timing */
#define CONTROL_PERIOD_US       10000U
#define CONTROL_DT_MIN_US       1000U
#define CONTROL_DT_MAX_US       50000U

/* IMU initialization and stationary gyro-bias calibration. */
#define IMU_INIT_RETRY_COUNT    300U
#define IMU_INIT_RETRY_US       10000U
#define IMU_BIAS_SAMPLES        100U
#define IMU_BIAS_SAMPLE_US      10000U
#define IMU_BIAS_WARN_STD_DPS   3.0f
#define IMU_BIAS_MAX_ABS_DPS    8.0f
#define IMU_DISAGREE_CONFIRM_TICKS 5U

/* Keep diagnostic logging slower than the 100 Hz control loop. */
#define IMU_LOG_INTERVAL_TICKS  20U

/* Hub mode-menu polling */
#define MENU_POLL_US            20000U

/* Calibration: robot crosses a transverse black line at low speed. */
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

/* Force-sensor debounce: 3 control ticks = nominally 30 ms. */
#define BUTTON_DEBOUNCE_SAMPLES 3U

/* PID gains retained as initial exercise values. */
#define KP                      1.0f
#define KI                      0.1f
#define KD                      0.1f
#define I_TERM_LIMIT            20.0f
#define DERIVATIVE_FILTER_TAU_S 0.040f

/* Adaptive wheel-speed limits [deg/s]. */
#define BASE_SPEED_STRAIGHT     120
#define BASE_SPEED_TURN         80
#define SLOWDOWN_ERROR          40.0f
#define MIN_WHEEL_SPEED         30
#define MAX_WHEEL_SPEED         200
#define MAX_STEERING            100.0f

/* Differential-drive odometry geometry: M_S6 wheelbase configuration. */
#define ODOM_PI                  3.14159265f
#define WHEEL_DIAMETER_MM        88.0f
#define WHEEL_TRACK_MM           144.0f
#define ODOM_RAD_TO_DEG          57.2957795f
#define ODOM_YAW_RATE_THRESHOLD  8.0f
#define ODOM_YAW_RATE_FULL       60.0f
#define ODOM_YAW_DAMPING_GAIN    0.15f
#define ODOM_YAW_FILTER_TAU_S    0.050f
#define ODOM_GYRO_WEIGHT         0.85f
#define ODOM_ENCODER_WEIGHT      (1.0f - ODOM_GYRO_WEIGHT)
#define IMU_ENCODER_DISAGREE_DPS 35.0f

/* Normalized reflectance state thresholds (black=0, white=100). */
#define LINE_DARK_THRESHOLD      45.0f
#define LINE_WHITE_THRESHOLD     70.0f
#define LINE_CENTER_DIFF         12.0f
#define LINE_CENTER_DEADBAND     4.0f
#define LINE_BOTH_DARK_MAX_SPEED 65
#define LINE_TRANSITION_MAX_SPEED 95

typedef enum {
    MODE_LINE_TRACE = 0,
    MODE_CALIBRATION = 1
} app_mode_t;

typedef enum {
    CAL_SEEK_BLACK = 0,
    CAL_MEASURE_BLACK,
    CAL_SEEK_WHITE_AFTER,
    CAL_DONE
} calibration_phase_t;

typedef enum {
    CAL_RESULT_OK = 0,
    CAL_RESULT_FAILED,
    CAL_RESULT_CANCELLED
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
    float yaw_rate_deg_s; /* fused yaw rate */
    float curvature_per_m;
    bool initialized;
} odometry_t;

static volatile uint32_t control_tick_overruns = 0;
static line_calibration_t saved_calibration = {0};

/* Keep a floating-point value within an inclusive range. */
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

/* Reduce forward speed for either large line error or high measured
 * yaw rate. The odometry term is deliberately conservative. */
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

/* Reset the relative odometry origin at the beginning of each run. */
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

/* Differential-drive odometry from the standard motor encoder API.
 * pup_motor_setup() defines each motor's positive direction as forward,
 * so both encoder increments are positive during straight forward motion. */
/* Fuse differential-drive encoder odometry with the Hub gyro.
 *
 * - Wheel encoder angle increments provide forward distance and an
 *   independent estimate of heading change.
 * - Bias-corrected IMU Z gyro provides the fast body yaw-rate measurement.
 * - The initial complementary blend uses 85% gyro / 15% encoder for yaw.
 *
 * The wheel geometry comes from M_S6.c: 88 mm tire diameter and 144 mm
 * wheel-center spacing. Positive yaw is a left turn for both sources:
 * (right wheel travel - left wheel travel) / track width.
 */
static void update_odometry(
    odometry_t *odometry,
    pup_motor_t *motor_left,
    pup_motor_t *motor_right,
    float dt,
    float gyro_z_deg_s,
    bool imu_available,
    bool imu_fusion_enabled)
{
    const int32_t left_count = pup_motor_get_count(motor_left);
    const int32_t right_count = pup_motor_get_count(motor_right);

    const float mm_per_degree =
        (ODOM_PI * WHEEL_DIAMETER_MM) / 360.0f;

    /* Motor speed is the API's measured encoder-derived speed [deg/s]. */
    const float left_speed_mm_s =
        (float)pup_motor_get_speed(motor_left) * mm_per_degree;
    const float right_speed_mm_s =
        (float)pup_motor_get_speed(motor_right) * mm_per_degree;

    odometry->linear_speed_mm_s =
        (left_speed_mm_s + right_speed_mm_s) * 0.5f;

    const float encoder_yaw_rate =
        ((right_speed_mm_s - left_speed_mm_s) / WHEEL_TRACK_MM) *
        ODOM_RAD_TO_DEG;

    odometry->encoder_yaw_rate_deg_s = encoder_yaw_rate;
    odometry->gyro_yaw_rate_deg_s =
        imu_available ? gyro_z_deg_s : 0.0f;
    odometry->yaw_rate_difference_deg_s =
        imu_available ? gyro_z_deg_s - encoder_yaw_rate : 0.0f;

    const float fused_yaw_rate_raw = imu_fusion_enabled
        ? (ODOM_GYRO_WEIGHT * gyro_z_deg_s +
           ODOM_ENCODER_WEIGHT * encoder_yaw_rate)
        : encoder_yaw_rate;

    if (!odometry->initialized) {
        odometry->previous_left_count = left_count;
        odometry->previous_right_count = right_count;
        odometry->yaw_rate_deg_s = fused_yaw_rate_raw;
        odometry->curvature_per_m = 0.0f;
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
    const float delta_heading_encoder =
        (delta_right_mm - delta_left_mm) / WHEEL_TRACK_MM;

    /* Integrate the gyro and encoder heading increments with the same
     * complementary weights used for yaw-rate fusion. */
    const float delta_heading_gyro =
        gyro_z_deg_s * dt / ODOM_RAD_TO_DEG;
    const float delta_heading_fused = imu_fusion_enabled
        ? (ODOM_GYRO_WEIGHT * delta_heading_gyro +
           ODOM_ENCODER_WEIGHT * delta_heading_encoder)
        : delta_heading_encoder;

    const float midpoint_heading =
        odometry->heading_rad + delta_heading_fused * 0.5f;

    odometry->x_mm += delta_distance * cosf(midpoint_heading);
    odometry->y_mm += delta_distance * sinf(midpoint_heading);
    odometry->distance_mm += delta_distance;
    odometry->heading_rad += delta_heading_fused;

    while (odometry->heading_rad > ODOM_PI) {
        odometry->heading_rad -= 2.0f * ODOM_PI;
    }
    while (odometry->heading_rad < -ODOM_PI) {
        odometry->heading_rad += 2.0f * ODOM_PI;
    }

    /* Smooth the fused rate used by motion classification and steering. */
    if (dt > 0.0f) {
        const float filter_alpha =
            dt / (ODOM_YAW_FILTER_TAU_S + dt);

        odometry->yaw_rate_deg_s +=
            filter_alpha *
            (fused_yaw_rate_raw - odometry->yaw_rate_deg_s);
    } else {
        odometry->yaw_rate_deg_s = fused_yaw_rate_raw;
    }

    const float linear_speed_m_s =
        odometry->linear_speed_mm_s / 1000.0f;

    if (fabsf(linear_speed_m_s) > 0.005f) {
        const float yaw_rate_rad_s =
            odometry->yaw_rate_deg_s / ODOM_RAD_TO_DEG;
        odometry->curvature_per_m =
            yaw_rate_rad_s / linear_speed_m_s;
    } else {
        odometry->curvature_per_m = 0.0f;
    }
}

/* Classify the actual wheel motion estimated from encoder odometry.
 * Positive yaw rate means a left turn; negative means a right turn. */
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


/* The two sensors straddle the line: centered normally means both are
 * white and balanced. A dark left/right sensor identifies which side of
 * the line has moved under the sensor. Both-dark is treated as a crossing
 * or unusually wide line, not as a reliable left/right position cue. */
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

/* The hub API returns a bitmask of all currently pressed built-in buttons. */
static bool read_hub_buttons(uint32_t *pressed_bits)
{
    hub_button_t pressed = 0;

    if (hub_button_is_pressed(&pressed) != PBIO_SUCCESS) {
        return false;
    }

    *pressed_bits = (uint32_t)pressed;
    return true;
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

/* Select one of the two operating modes. LEFT selects trace; RIGHT selects
 * calibration; CENTER confirms. Edge detection prevents a held button from
 * repeatedly changing or confirming the selection. */
static app_mode_t select_mode(void)
{
    app_mode_t selected = MODE_LINE_TRACE;
    uint32_t previous = 0;

    hub_display_char('L');
    syslog(LOG_NOTICE,
           "PID V2: LEFT=LINE TRACE, RIGHT=CALIBRATION, CENTER=SELECT");

    wait_hub_buttons_released();

    while (1) {
        uint32_t current = 0;

        if (!read_hub_buttons(&current)) {
            dly_tsk(MENU_POLL_US);
            continue;
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
            selected = MODE_LINE_TRACE;
            hub_display_char('L');
            syslog(LOG_NOTICE, "PID V2: mode selected LINE TRACE");
        } else if (right_rise) {
            selected = MODE_CALIBRATION;
            hub_display_char('C');
            syslog(LOG_NOTICE, "PID V2: mode selected CALIBRATION");
        }

        previous = current;

        if (center_rise) {
            syslog(LOG_NOTICE, "PID V2: mode confirmed");
            return selected;
        }

        dly_tsk(MENU_POLL_US);
    }
}

/* Start the cyclic release source with an empty binary semaphore. */
static bool start_control_clock(void)
{
    if (ini_sem(SEM_CONTROL_TICK) != E_OK) {
        return false;
    }

    control_tick_overruns = 0;
    return sta_cyc(CYC_CONTROL_TICK) == E_OK;
}

/* Stop the cyclic source before returning to the non-periodic mode menu. */
static void stop_control_clock(void)
{
    stp_cyc(CYC_CONTROL_TICK);
    ini_sem(SEM_CONTROL_TICK);
}

/* Cyclic-handler context: notify only. All sensor and motor operations are
 * performed later by Main in ordinary task context. */
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

    /* The IMU driver initializes asynchronously in the Pybricks task.
     * Retry PBIO_ERROR_AGAIN from ordinary task context. */
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

    /* The robot must remain stationary while estimating the gyro zero bias. */
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


/* Initialize one sensor's crossing detector from its white baseline. */
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

/* Track white -> black -> white independently for one sensor. */
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
            /* A short bright gap was noise or an incomplete crossing. */
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

/* Drive straight slowly while crossing a transverse black line.
 * The robot must start with BOTH sensors on the white surface.
 * A successful result is written only after both sensors complete
 * white -> black -> white and pass contrast validation. */
static calibration_result_t calibrate_across_line(
    pup_motor_t *motor_left,
    pup_motor_t *motor_right,
    pup_device_t *sensor_left,
    pup_device_t *sensor_right,
    line_calibration_t *candidate)
{
    float white_left_sum = 0.0f;
    float white_right_sum = 0.0f;

    sensor_crossing_t left_crossing = {0};
    sensor_crossing_t right_crossing = {0};

    hub_display_char('W');
    syslog(LOG_NOTICE,
           "PID V2 CAL: keep both sensors on white; sampling baseline");

    /* Take a stable white baseline before the robot begins to move. */
    for (unsigned int i = 0; i < CAL_WHITE_SAMPLES; i++) {
        if (!wait_control_tick()) {
            syslog(LOG_ERROR, "PID V2 CAL: control tick wait failed");
            return CAL_RESULT_FAILED;
        }

        if (center_button_pressed()) {
            syslog(LOG_NOTICE, "PID V2 CAL: cancelled before movement");
            return CAL_RESULT_CANCELLED;
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

        /* Continue until BOTH sensors have independently returned to white. */
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

        /* Encoder synchronization keeps the calibration path straight. */
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

/* Convert each sensor's raw reflectance to a common 0..100 scale:
 * 0 = calibrated black, 100 = calibrated white. */
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

/* MAIN_TASK: startup menu, moving calibration, then periodic line tracing. */
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

    /* Initialize the built-in LSM6DS3TR-C IMU and estimate its stationary
     * gyro bias. If unavailable, line tracing remains available using
     * encoder-only motion estimates. */
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

    /* Calibration remains in RAM after returning to the menu.
     * It must be repeated after a power cycle. */
    while (1) {
        const app_mode_t selected = select_mode();

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
                    &candidate
                );

            stop_control_clock();
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);

            if (result == CAL_RESULT_OK) {
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

        break;
    }

    hub_display_char('L');

    bool running = false;
    bool button_armed = !pup_force_sensor_touched(start_stop_sensor);
    bool pid_initialized = false;
    bool overrun_warning_sent = false;

    unsigned int pressed_samples = 0;
    unsigned int released_samples = 0;
    uint32_t reported_overruns = 0;

    float integral = 0.0f;
    float last_error = 0.0f;
    float derivative_filtered = 0.0f;
    float last_steering = 0.0f;

    odometry_t odometry = {0};

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

        /* Force sensor C toggles run/stop. Three consecutive samples are
         * required, and a release must be observed before another press. */
        const bool touched = pup_force_sensor_touched(start_stop_sensor);

        if (button_armed) {
            if (touched) {
                pressed_samples++;

                if (pressed_samples >= BUTTON_DEBOUNCE_SAMPLES) {
                    button_armed = false;
                    pressed_samples = 0;
                    released_samples = 0;

                    if (running) {
                        running = false;
                        pup_motor_brake(motor_left);
                        pup_motor_brake(motor_right);

                        integral = 0.0f;
                        last_error = 0.0f;
                        derivative_filtered = 0.0f;
                        last_steering = 0.0f;
                        pid_initialized = false;

                        syslog(LOG_NOTICE,
                               "PID V2: stopped by force sensor C; "
                               "ODO D=%ld X=%ld Y=%ld H=%ld",
                               (long)odometry.distance_mm,
                               (long)odometry.x_mm,
                               (long)odometry.y_mm,
                               (long)(odometry.heading_rad * ODOM_RAD_TO_DEG));
                    } else {
                        running = true;

                        integral = 0.0f;
                        last_error = 0.0f;
                        derivative_filtered = 0.0f;
                        last_steering = 0.0f;
                        pid_initialized = false;
                        reset_odometry(&odometry);

                        syslog(LOG_NOTICE,
                               "PID V2: started by force sensor C");
                    }
                }
            } else {
                pressed_samples = 0;
            }
        } else {
            if (!touched) {
                released_samples++;

                if (released_samples >= BUTTON_DEBOUNCE_SAMPLES) {
                    button_armed = true;
                    released_samples = 0;
                }
            } else {
                released_samples = 0;
            }
        }

        /* Read the latest cached IMU sample. The standard SPIKE-RT API
         * returns the most recently acquired driver data; it does not start
         * a new I2C transaction from this control task. Read it while idle
         * too, so stationary gyro behavior can be inspected before starting. */
        float acceleration[3] = {0.0f, 0.0f, 0.0f};
        float angular_velocity[3] = {0.0f, 0.0f, 0.0f};
        float gyro_z_corrected = 0.0f;

        if (imu_available) {
            hub_imu_get_acceleration(acceleration);
            hub_imu_get_angular_velocity(angular_velocity);

            for (unsigned int axis = 0; axis < 3; axis++) {
                angular_velocity[axis] -= gyro_bias[axis];
            }

            gyro_z_corrected = angular_velocity[2];
        }

        /* Update wheel odometry and fuse encoder yaw with the bias-corrected
         * IMU Z gyro. If IMU initialization or its stationary quality check
         * failed, encoder-only odometry remains available. */
        update_odometry(
            &odometry,
            motor_left,
            motor_right,
            dt,
            gyro_z_corrected,
            imu_available,
            imu_fusion_enabled
        );

        /* Persistent disagreement is useful evidence of wheel slip,
         * sign/geometry mismatch, or excessive sensor delay. It is logged
         * but does not automatically change the fusion weight. */
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
            /* Idle diagnostics allow a stationary test and a hand-rotation
             * sign check without commanding either motor. */
            imu_log_ticks++;
            if (imu_log_ticks >= IMU_LOG_INTERVAL_TICKS) {
                imu_log_ticks = 0;
                syslog(LOG_NOTICE,
                       "IDLE IMU=%d USE=%d GZx10=%ld EZx10=%ld DIFFx10=%ld FZx10=%ld ACCZx10=%ld",
                       imu_available ? 1 : 0,
                       imu_fusion_enabled ? 1 : 0,
                       (long)(odometry.gyro_yaw_rate_deg_s * 10.0f),
                       (long)(odometry.encoder_yaw_rate_deg_s * 10.0f),
                       (long)(odometry.yaw_rate_difference_deg_s * 10.0f),
                       (long)(odometry.yaw_rate_deg_s * 10.0f),
                       (long)(acceleration[2] * 10.0f));
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

        /* Negative error means the left sensor is darker than the right:
         * the line is under the left sensor; slow the left wheel to steer left. */
        float error = normalized_left - normalized_right;
        const line_state_t line_state =
            classify_line_state(normalized_left, normalized_right, error);

        /* Suppress tiny reflectance imbalance around the centered state. */
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

        /* Any measured turn reduces the straight-line speed ceiling slightly.
         * A stronger line error or yaw rate can reduce it further below. */
        if (motion_state != MOTION_STATE_STRAIGHT &&
            base_speed > BASE_SPEED_STRAIGHT - 15) {
            base_speed = BASE_SPEED_STRAIGHT - 15;
        }

        /* Slow down for ambiguous transitions and wide-line/crossing states. */
        if (line_state == LINE_STATE_BOTH_DARK &&
            base_speed > LINE_BOTH_DARK_MAX_SPEED) {
            base_speed = LINE_BOTH_DARK_MAX_SPEED;
        } else if (line_state == LINE_STATE_TRANSITION &&
                   base_speed > LINE_TRANSITION_MAX_SPEED) {
            base_speed = LINE_TRANSITION_MAX_SPEED;
        }

        /* Encoder-derived yaw-rate feedback damps actual chassis rotation.
         * With this motor sign convention, positive yaw is a left turn and
         * positive steering commands a rightward correction. */
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

        /* Do not integrate while both sensors are over black: the pair no
         * longer provides a trustworthy left/right position difference. */
        if (line_state != LINE_STATE_BOTH_DARK &&
            !pushes_high && !pushes_low) {
            integral = integral_candidate;
        }

        float steering;
        if (line_state == LINE_STATE_BOTH_DARK) {
            /* Carry the last steering direction through a wide line or
             * crossing, while encoder yaw feedback gently damps rotation. */
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
            pid_initialized = false;
            integral = 0.0f;
            derivative_filtered = 0.0f;
            last_steering = 0.0f;
            odometry.initialized = false;

            syslog(LOG_ERROR,
                   "PID V2: motor command failed L=%d R=%d",
                   (int)left_result, (int)right_result);
        }

        /* 5 Hz diagnostics: raw sensor values, line/motion state,
         * wheel-vs-gyro yaw, fused pose, and commanded wheel speeds. */
        imu_log_ticks++;
        if (imu_log_ticks >= IMU_LOG_INTERVAL_TICKS) {
            imu_log_ticks = 0;

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
                   "FUSION IMU=%d USE=%d GZx10=%ld EZx10=%ld DIFFx10=%ld FZx10=%ld Hx10=%ld X=%ld Y=%ld D=%ld V=%ld Kx1000=%ld",
                   imu_available ? 1 : 0,
                   imu_fusion_enabled ? 1 : 0,
                   (long)(odometry.gyro_yaw_rate_deg_s * 10.0f),
                   (long)(odometry.encoder_yaw_rate_deg_s * 10.0f),
                   (long)(odometry.yaw_rate_difference_deg_s * 10.0f),
                   (long)(odometry.yaw_rate_deg_s * 10.0f),
                   (long)(odometry.heading_rad * ODOM_RAD_TO_DEG * 10.0f),
                   (long)odometry.x_mm,
                   (long)odometry.y_mm,
                   (long)odometry.distance_mm,
                   (long)odometry.linear_speed_mm_s,
                   (long)(odometry.curvature_per_m * 1000.0f));

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
}
