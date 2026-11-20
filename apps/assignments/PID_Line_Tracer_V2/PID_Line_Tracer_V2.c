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
 */

#include <kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <t_syslog.h>

#include "PID_Line_Tracer_V2.h"
#include "kernel_cfg.h"

#include "spike/pup/motor.h"
#include "spike/pup/colorsensor.h"
#include "spike/pup/forcesensor.h"
#include "spike/hub/button.h"
#include "spike/hub/display.h"
#include "spike/hub/speaker.h"

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

/* Hub mode-menu polling */
#define MENU_POLL_US            20000U

/* Calibration: robot crosses a transverse black line at low speed. */
#define CAL_WHITE_SAMPLES       8U
#define CAL_WHITE_MIN_REFLECT   20
#define CAL_BLACK_RATIO         0.70f
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

/* Reduce forward speed when the normalized line-position error grows. */
static int32_t calculate_base_speed(float abs_error)
{
    float ratio = clamp_float(abs_error / SLOWDOWN_ERROR, 0.0f, 1.0f);

    return (int32_t)(
        (float)BASE_SPEED_STRAIGHT -
        ratio * (float)(BASE_SPEED_STRAIGHT - BASE_SPEED_TURN) + 0.5f
    );
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
            (delta_left - delta_right) * CAL_SYNC_GAIN;
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

    uint32_t last_tick_hrt = 0;
    bool have_last_tick = false;

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
                        pid_initialized = false;

                        syslog(LOG_NOTICE,
                               "PID V2: stopped by force sensor C");
                    } else {
                        running = true;

                        integral = 0.0f;
                        last_error = 0.0f;
                        derivative_filtered = 0.0f;
                        pid_initialized = false;

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

        if (!running) {
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

        /* The sensors straddle the line. At the centered position both see
         * white; if one moves over black, the normalized difference changes. */
        const float position = normalized_right - normalized_left;
        const float error = -position;

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

        const float abs_error = error >= 0.0f ? error : -error;
        const int32_t base_speed = calculate_base_speed(abs_error);

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

        if (!pushes_high && !pushes_low) {
            integral = integral_candidate;
        }

        const float steering = clamp_float(
            p_term + KI * integral + d_term,
            -steering_limit,
            steering_limit
        );

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
            pid_initialized = false;
            integral = 0.0f;
            derivative_filtered = 0.0f;

            syslog(LOG_ERROR,
                   "PID V2: motor command failed L=%d R=%d",
                   (int)left_result, (int)right_result);
        }

        last_error = error;
    }
}
