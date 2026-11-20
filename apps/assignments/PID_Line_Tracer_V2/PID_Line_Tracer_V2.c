/*
 * PID_Line_Tracer_V2.c
 *
 * V2 keeps the standard SPIKE-RT v0.2.0 motor, color-sensor and force-sensor
 * APIs. No device-driver or kernel-driver changes are required.
 *
 * Hardware:
 *   Left motor       A (counterclockwise)
 *   Right motor      E (clockwise)
 *   Left color       B
 *   Right color      F
 *   Start/stop force C
 *
 * V2 changes:
 *   1. A 10 ms cyclic handler releases a binary semaphore for control updates.
 *   2. fch_hrt() measures the actual interval between control executions.
 *   3. The sensor difference offset is averaged while the robot is stationary.
 *   4. The derivative term uses a first-order low-pass filter.
 *   5. Conditional integration reduces integral windup during output saturation.
 *   6. Base speed is reduced as line-position error grows.
 *   7. Wheel-speed limits and motor-command error handling are explicit.
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
#include "spike/hub/speaker.h"

/* Port assignment */
#define LEFT_COLOR_SENSOR       PBIO_PORT_ID_B
#define RIGHT_COLOR_SENSOR      PBIO_PORT_ID_F
#define START_STOP_SENSOR       PBIO_PORT_ID_C
#define LEFT_MOTOR              PBIO_PORT_ID_A
#define RIGHT_MOTOR             PBIO_PORT_ID_E

/* Nominal control timing */
#define CONTROL_PERIOD_US       10000U
#define CONTROL_PERIOD_S        0.010f
#define CONTROL_DT_MIN_US       1000U
#define CONTROL_DT_MAX_US       50000U

/* Startup calibration */
#define CALIBRATION_SAMPLES     32
#define CALIBRATION_INTERVAL_US 10000U

/* Force-sensor debounce: 3 control ticks = nominally 30 ms */
#define BUTTON_DEBOUNCE_SAMPLES 3

/* PID gains: retain the original exercise values as initial tuning values */
#define KP                      1.0f
#define KI                      0.1f
#define KD                      0.1f

/* Integral contribution limit, in steering-output units */
#define I_TERM_LIMIT            20.0f

/* Filter time constant for the derivative term */
#define DERIVATIVE_FILTER_TAU_S 0.040f

/* Adaptive wheel-speed limits [deg/s] */
#define BASE_SPEED_STRAIGHT     120
#define BASE_SPEED_TURN         80
#define SLOWDOWN_ERROR           40.0f
#define MIN_WHEEL_SPEED          30
#define MAX_WHEEL_SPEED         200
#define MAX_STEERING            100.0f

/* A binary semaphore coalesces pending control ticks instead of building
 * an unbounded backlog when a control iteration takes longer than 10 ms. */
static volatile uint32_t control_tick_overruns = 0;

/* Keep the requested value within an inclusive range. */
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

static int32_t clamp_speed(int32_t value)
{
    if (value < MIN_WHEEL_SPEED) {
        return MIN_WHEEL_SPEED;
    }
    if (value > MAX_WHEEL_SPEED) {
        return MAX_WHEEL_SPEED;
    }
    return value;
}

/* Reduce forward speed when the measured line-position error is large. */
static int32_t calculate_base_speed(float abs_error)
{
    float ratio = abs_error / SLOWDOWN_ERROR;

    ratio = clamp_float(ratio, 0.0f, 1.0f);

    return (int32_t)(
        (float)BASE_SPEED_STRAIGHT -
        ratio * (float)(BASE_SPEED_STRAIGHT - BASE_SPEED_TURN) + 0.5f
    );
}

/* Measure the normal left/right reflection difference while the robot is
 * stationary and centered at its starting position on the line. */
static float calibrate_sensor_offset(
    pup_device_t *sensor_left,
    pup_device_t *sensor_right)
{
    float sum = 0.0f;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int32_t left =
            pup_color_sensor_reflection(sensor_left);
        int32_t right =
            pup_color_sensor_reflection(sensor_right);

        sum += (float)(right - left);
        dly_tsk(CALIBRATION_INTERVAL_US);
    }

    return sum / (float)CALIBRATION_SAMPLES;
}

/* Cyclic-handler context: only send a short notification.
 * Do not read sensors, update motors or write to the display here. */
void control_tick_handler(intptr_t exinf)
{
    if (isig_sem(SEM_CONTROL_TICK) != E_OK) {
        control_tick_overruns++;
    }
}

/* MAIN_TASK: periodic line-tracking controller and force-sensor monitor. */
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

    syslog(LOG_NOTICE, "PID V2: initialized; keep robot centered on line");
    dly_tsk(3000000);

    /* Calibrate the sensor-difference zero while the robot is stationary. */
    const float sensor_offset =
        calibrate_sensor_offset(sensor_left, sensor_right);

    syslog(LOG_NOTICE,
           "PID V2: sensor offset x10=%ld",
           (long)(sensor_offset * 10.0f));

    hub_speaker_set_volume(100);
    hub_speaker_play_tone(695, 200);

    /* Start the 10 ms cyclic notification only after initialization. */
    if (sta_cyc(CYC_CONTROL_TICK) != E_OK) {
        syslog(LOG_ERROR, "PID V2: failed to start control cycle");
        pup_motor_stop(motor_left);
        pup_motor_stop(motor_right);
        ext_tsk();
    }

    bool running = false;
    bool button_armed = true;
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

    syslog(LOG_NOTICE, "PID V2: ready; press force sensor C to start");

    while (1) {
        /* Wait for the next cyclic notification. */
        if (wai_sem(SEM_CONTROL_TICK) != E_OK) {
            syslog(LOG_ERROR, "PID V2: control semaphore wait failed");
            stp_cyc(CYC_CONTROL_TICK);
            pup_motor_brake(motor_left);
            pup_motor_brake(motor_right);
            ext_tsk();
        }

        /* PrimeHub's high-resolution timer is configured at 1 MHz.
         * Unsigned subtraction also handles the 32-bit timer wraparound. */
        uint32_t now_hrt = (uint32_t)fch_hrt();
        uint32_t elapsed_us = CONTROL_PERIOD_US;

        if (have_last_tick) {
            elapsed_us = (uint32_t)(now_hrt - last_tick_hrt);
        } else {
            have_last_tick = true;
        }
        last_tick_hrt = now_hrt;

        /* Use measured elapsed time, with guards for abnormal intervals. */
        if (elapsed_us < CONTROL_DT_MIN_US) {
            elapsed_us = CONTROL_PERIOD_US;
        }
        if (elapsed_us > CONTROL_DT_MAX_US) {
            elapsed_us = CONTROL_DT_MAX_US;
            derivative_filtered = 0.0f;
        }
        const float dt = (float)elapsed_us / 1000000.0f;

        /* Report a timing overrun once, outside the cyclic-handler context. */
        if (!overrun_warning_sent &&
            control_tick_overruns != reported_overruns) {
            reported_overruns = control_tick_overruns;
            overrun_warning_sent = true;
            syslog(LOG_WARNING,
                   "PID V2: control tick overrun detected");
        }

        /* Debounced start/stop toggle. A new press is accepted only after
         * the force sensor has first been released. */
        bool touched = pup_force_sensor_touched(start_stop_sensor);

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

        /* While stopped, the periodic task only monitors the force sensor. */
        if (!running) {
            continue;
        }

        /* Read both sensors and subtract the calibrated zero offset. */
        int32_t reflection_left =
            pup_color_sensor_reflection(sensor_left);
        int32_t reflection_right =
            pup_color_sensor_reflection(sensor_right);

        float raw_difference =
            (float)(reflection_right - reflection_left);
        float error = sensor_offset - raw_difference;

        /* Avoid a derivative kick on the first sample after start/resume. */
        if (!pid_initialized) {
            last_error = error;
            derivative_filtered = 0.0f;
            integral = 0.0f;
            pid_initialized = true;
        }

        float derivative_raw = (error - last_error) / dt;
        float filter_alpha =
            dt / (DERIVATIVE_FILTER_TAU_S + dt);

        derivative_filtered +=
            filter_alpha * (derivative_raw - derivative_filtered);

        float abs_error = (error >= 0.0f) ? error : -error;
        int32_t base_speed = calculate_base_speed(abs_error);

        /* Keep both wheel commands within the configured speed range. */
        float steering_limit = MAX_STEERING;
        float limit_from_min = (float)base_speed - MIN_WHEEL_SPEED;
        float limit_from_max = MAX_WHEEL_SPEED - (float)base_speed;

        if (steering_limit > limit_from_min) {
            steering_limit = limit_from_min;
        }
        if (steering_limit > limit_from_max) {
            steering_limit = limit_from_max;
        }
        if (steering_limit < 0.0f) {
            steering_limit = 0.0f;
        }

        /* Trapezoidal integration with conditional anti-windup. */
        float integral_candidate =
            integral + ((error + last_error) * 0.5f * dt);
        float integral_limit = I_TERM_LIMIT / KI;

        integral_candidate = clamp_float(
            integral_candidate, -integral_limit, integral_limit);

        float p_term = KP * error;
        float d_term = KD * derivative_filtered;
        float candidate_output =
            p_term + KI * integral_candidate + d_term;

        bool pushes_high =
            (candidate_output > steering_limit) && (error > 0.0f);
        bool pushes_low =
            (candidate_output < -steering_limit) && (error < 0.0f);

        if (!pushes_high && !pushes_low) {
            integral = integral_candidate;
        }

        float steering = clamp_float(
            p_term + KI * integral + d_term,
            -steering_limit,
            steering_limit
        );

        /* Positive steering keeps the original left-fast/right-slow sign. */
        int32_t left_speed =
            (int32_t)((float)base_speed + steering + 0.5f);
        int32_t right_speed =
            (int32_t)((float)base_speed - steering + 0.5f);

        left_speed = clamp_speed(left_speed);
        right_speed = clamp_speed(right_speed);

        pbio_error_t left_result =
            pup_motor_set_speed(motor_left, left_speed);
        pbio_error_t right_result =
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
