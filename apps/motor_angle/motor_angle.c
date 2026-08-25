// SPDX-License-Identifier: MIT

#include <kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <t_syslog.h>

#include "motor_angle.h"
#include "spike/hub/button.h"
#include "spike/hub/display.h"
#include "spike/hub/light.h"
#include "spike/hub/system.h"
#include "spike/pup/motor.h"

#define MOTOR_PORT_C            PBIO_PORT_ID_C
#define MOTOR_PORT_D            PBIO_PORT_ID_D
#define SAMPLE_INTERVAL_US      50000U
#define SERIAL_EVERY_SAMPLES    4U
#define DIAL_BRIGHTNESS_C       100U
#define DIAL_BRIGHTNESS_D       45U

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

static void draw_angle_dials(int32_t angle_c_deg, int32_t angle_d_deg)
{
  uint8_t index_c = angle_to_dial_index(angle_c_deg);
  uint8_t index_d = angle_to_dial_index(angle_d_deg);

  hub_display_off();
  hub_display_pixel(2, 2, 20);

  if (index_c == index_d) {
    /* Both motors point to the same dial position. */
    hub_display_pixel(dial_rows[index_c], dial_cols[index_c], DIAL_BRIGHTNESS_C);
    return;
  }

  /* Port C is the brighter point, port D the dimmer point. */
  hub_display_pixel(dial_rows[index_c], dial_cols[index_c], DIAL_BRIGHTNESS_C);
  hub_display_pixel(dial_rows[index_d], dial_cols[index_d], DIAL_BRIGHTNESS_D);
}

static bool wait_for_two_motors(pup_motor_t **motor_c, pup_motor_t **motor_d)
{
  bool announced_c = false;
  bool announced_d = false;

  *motor_c = NULL;
  *motor_d = NULL;

  while (*motor_c == NULL || *motor_d == NULL) {
    if (*motor_c == NULL) {
      *motor_c = pup_motor_get_device(MOTOR_PORT_C);
    }
    if (*motor_d == NULL) {
      *motor_d = pup_motor_get_device(MOTOR_PORT_D);
    }

    if (*motor_c == NULL && !announced_c) {
      syslog(LOG_NOTICE, "MOTOR ANGLE: connect a motor to port C");
      announced_c = true;
    }
    if (*motor_d == NULL && !announced_d) {
      syslog(LOG_NOTICE, "MOTOR ANGLE: connect a motor to port D");
      announced_d = true;
    }

    if (*motor_c != NULL && *motor_d != NULL) {
      break;
    }

    hub_light_on_color(PBIO_COLOR_YELLOW);
    if (*motor_c == NULL) {
      hub_display_char('C');
    } else {
      hub_display_char('D');
    }
    dly_tsk(500000U);
  }

  return true;
}

static bool setup_motor_encoder(pup_motor_t *motor, char port_name)
{
  pbio_error_t err;

  do {
    err = pup_motor_setup(motor, PUP_DIRECTION_CLOCKWISE, true);
    if (err == PBIO_ERROR_AGAIN) {
      dly_tsk(100000U);
    }
  } while (err == PBIO_ERROR_AGAIN);

  if (err != PBIO_SUCCESS) {
    syslog(LOG_ERROR, "MOTOR %c: setup failed (%d)", port_name, (int)err);
    return false;
  }

  err = pup_motor_stop(motor);
  if (err != PBIO_SUCCESS) {
    syslog(LOG_ERROR, "MOTOR %c: stop failed (%d)", port_name, (int)err);
    return false;
  }

  return true;
}

static void reset_both_encoders(pup_motor_t *motor_c, pup_motor_t *motor_d)
{
  pbio_error_t err_c = pup_motor_reset_count(motor_c);
  pbio_error_t err_d = pup_motor_reset_count(motor_d);

  if (err_c == PBIO_SUCCESS && err_d == PBIO_SUCCESS) {
    hub_light_on_color(PBIO_COLOR_YELLOW);
    syslog(LOG_NOTICE, "ANGLE RESET: C=0 deg | D=0 deg");
  } else {
    hub_light_on_color(PBIO_COLOR_RED);
    syslog(LOG_ERROR, "ANGLE RESET FAILED: C=%d | D=%d", (int)err_c, (int)err_d);
  }
}

void main_task(intptr_t exinf)
{
  pup_motor_t *motor_c;
  pup_motor_t *motor_d;
  hub_button_t previous_buttons = 0;
  uint32_t serial_sample = 0;

  hub_display_char('2');
  hub_light_on_color(PBIO_COLOR_YELLOW);
  syslog(LOG_NOTICE, "DUAL MOTOR ANGLE VIEWER READY");
  syslog(LOG_NOTICE, "PORTS: C + D");
  syslog(LOG_NOTICE, "DISPLAY: C=bright | D=dim");
  syslog(LOG_NOTICE, "CENTER BUTTON: RESET BOTH ANGLES TO 0 DEG");

  wait_for_two_motors(&motor_c, &motor_d);

  if (!setup_motor_encoder(motor_c, 'C') || !setup_motor_encoder(motor_d, 'D')) {
    hub_light_on_color(PBIO_COLOR_RED);
    hub_display_char('E');
    hub_system_shutdown();
  }

  hub_light_on_color(PBIO_COLOR_GREEN);
  draw_angle_dials(0, 0);
  syslog(LOG_NOTICE, "DUAL MOTOR ANGLE VIEWER START");

  while (1) {
    hub_button_t buttons;
    int32_t angle_c_deg;
    int32_t angle_d_deg;
    int32_t speed_c_dps;
    int32_t speed_d_dps;
    int32_t turn_c_deg;
    int32_t turn_d_deg;

    dly_tsk(SAMPLE_INTERVAL_US);
    buttons = read_buttons();

    if ((buttons & HUB_BUTTON_CENTER) != 0 &&
        (previous_buttons & HUB_BUTTON_CENTER) == 0) {
      reset_both_encoders(motor_c, motor_d);
    }

    previous_buttons = buttons;

    angle_c_deg = pup_motor_get_count(motor_c);
    angle_d_deg = pup_motor_get_count(motor_d);
    speed_c_dps = pup_motor_get_speed(motor_c);
    speed_d_dps = pup_motor_get_speed(motor_d);
    turn_c_deg = normalize_one_turn(angle_c_deg);
    turn_d_deg = normalize_one_turn(angle_d_deg);

    draw_angle_dials(angle_c_deg, angle_d_deg);

    serial_sample++;
    if (serial_sample >= SERIAL_EVERY_SAMPLES) {
      serial_sample = 0;
      syslog(LOG_NOTICE,
             "C ANGLE:%d TURN:%d SPEED:%d | D ANGLE:%d TURN:%d SPEED:%d",
             (int)angle_c_deg, (int)turn_c_deg, (int)speed_c_dps,
             (int)angle_d_deg, (int)turn_d_deg, (int)speed_d_dps);

      if ((buttons & HUB_BUTTON_CENTER) == 0) {
        hub_light_on_color(PBIO_COLOR_GREEN);
      }
    }
  }
}
