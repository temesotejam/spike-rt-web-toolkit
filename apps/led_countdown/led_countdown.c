/*
 * SPDX-License-Identifier: MIT
 *
 * Derived from the SPIKE-RT v0.2.0 LED sample.
 */

#include <kernel.h>
#include <kernel_cfg.h>
#include <t_syslog.h>

#include "led_countdown.h"
#include "spike/hub/display.h"
#include "spike/hub/light.h"

void main_task(intptr_t exinf)
{
  (void)exinf;
  char digit = '9';
  syslog(LOG_NOTICE, "LED countdown started.");
  while (1) {
    hub_display_off();
    hub_display_char(digit);
    hub_light_on_color(PBIO_COLOR_YELLOW);
    dly_tsk(1000 * 1000);
    if (digit == '0') digit = '9';
    else digit--;
  }
}
