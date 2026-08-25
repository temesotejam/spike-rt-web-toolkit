// SPDX-License-Identifier: MIT

#include <kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <t_syslog.h>

#include "spike/hub/button.h"
#include "spike/hub/display.h"
#include "spike/hub/light.h"
#include "spike/hub/system.h"
#include "spike/pup/motor.h"

#define MOTOR_PORT              PBIO_PORT_ID_C
#define SAMPLE_INTERVAL_US      50000U
#define SERIAL_EVERY_SAMPLES    4U

static const uint8_t dial_rows[16] = {
  0, 0, 0, 1, 2, 3, 4, 4,
  4, 4, 4, 3, 2, 1, 0, 0
};

static const uint8_t dial_cols[16] = {
  2, 3, 4, 4, 4, 4, 4, 3,
  2, 1, 0, 0, 0, 0, 0, 1
};

static hub_button_t read_buttons(void)
{
  hub_button_t pressed = 0;
  hub_button_is_pressed(&pressed);
  return pressed;
}

static int32_t normalize_one_turn(int32_t angle_deg)
{
  int32_t angle = angle_deg % 360;
  if (angle < 0) {
    angle += 360;
  }
  return angle;
}

static uint8_t angle_to_dial_index(int32_t angle_deg)
{
  int32_t one_turn = normalize_one_turn(angle_deg);

  /* Round to the nearest one of 16 positions (22.5 degrees each). */
  return (uint8_t)(((one_turn * 16 + 180) / 360) % 16);
}

static void draw_angle_dial(int32_t angle_deg)
{
  uint8_t index = angle_to_dial_index(angle_deg);

  hub_display_off();
  hub_display_pixel(2, 2, 20);
  hub_display_pixel(dial_rows[index], dial_cols[index], 100);
}

static pup_motor_t *wait_for_motor(void)
{
  pup_motor_t *motor = NULL;
  bool announced = false;

  while (motor == NULL) {
    motor = pup_motor_get_device(MOTOR_PORT);
    if (motor != NULL) {
      break;
    }

    if (!announced) {
      syslog(LOG_NOTICE, "MOTOR ANGLE: connect a motor to port A");
      announced = true;
    }

    hub_light_on_color(PBIO_COLOR_YELLOW);
    hub_display_char('A');
    dly_tsk(500000U);
  }

  return motor;
}

static bool setup_motor_encoder(pup_motor_t *motor)
{
  pbio_error_t err;

  do {
    err = pup_motor_setup(motor, PUP_DIRECTION_CLOCKWISE, true);
    if (err == PBIO_ERROR_AGAIN) {
      dly_tsk(100000U);
    }
  } while (err == PBIO_ERROR_AGAIN);

  if (err != PBIO_SUCCESS) {
    syslog(LOG_ERROR, "MOTOR ANGLE: motor setup failed (%d)", (int)err);
    return false;
  }

  err = pup_motor_stop(motor);
  if (err != PBIO_SUCCESS) {
    syslog(LOG_ERROR, "MOTOR ANGLE: motor stop failed (%d)", (int)err);
    return false;
  }

  return true;
}

void main_task(intptr_t exinf)
{
  pup_motor_t *motor;
  hub_button_t previous_buttons = 0;
  uint32_t serial_sample = 0;

  hub_display_char('A');
  hub_light_on_color(PBIO_COLOR_YELLOW);
  syslog(LOG_NOTICE, "MOTOR ANGLE VIEWER READY");
  syslog(LOG_NOTICE, "PORT: A");
  syslog(LOG_NOTICE, "CENTER BUTTON: RESET ANGLE TO 0 DEG");

  motor = wait_for_motor();
  if (!setup_motor_encoder(motor)) {
    hub_light_on_color(PBIO_COLOR_RED);
    hub_display_char('E');
    hub_system_shutdown();
  }

  hub_light_on_color(PBIO_COLOR_GREEN);
  draw_angle_dial(0);
  syslog(LOG_NOTICE, "MOTOR ANGLE VIEWER START");

  while (1) {
    hub_button_t buttons;
    int32_t angle_deg;
    int32_t speed_dps;
    int32_t one_turn_deg;

    dly_tsk(SAMPLE_INTERVAL_US);
    buttons = read_buttons();

    if ((buttons & HUB_BUTTON_CENTER) != 0 &&
        (previous_buttons & HUB_BUTTON_CENTER) == 0) {
      pbio_error_t err = pup_motor_reset_count(motor);
      if (err == PBIO_SUCCESS) {
        hub_light_on_color(PBIO_COLOR_YELLOW);
        syslog(LOG_NOTICE, "ANGLE RESET: 0 deg");
      } else {
        hub_light_on_color(PBIO_COLOR_RED);
        syslog(LOG_ERROR, "ANGLE RESET FAILED (%d)", (int)err);
      }
    }

    previous_buttons = buttons;

    angle_deg = pup_motor_get_count(motor);
    speed_dps = pup_motor_get_speed(motor);
    one_turn_deg = normalize_one_turn(angle_deg);

    draw_angle_dial(angle_deg);

    serial_sample++;
    if (serial_sample >= SERIAL_EVERY_SAMPLES) {
      serial_sample = 0;
      syslog(LOG_NOTICE, "ANGLE: %d deg | TURN: %d deg | SPEED: %d deg/s",
             (int)angle_deg, (int)one_turn_deg, (int)speed_dps);

      if ((buttons & HUB_BUTTON_CENTER) == 0) {
        hub_light_on_color(PBIO_COLOR_GREEN);
      }
    }
  }
}
