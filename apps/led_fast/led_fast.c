/*
 * SPDX-License-Identifier: MIT
 *
 * Derived from the SPIKE-RT v0.2.0 LED sample.
 */

#include <kernel.h>
#include <kernel_cfg.h>
#include <t_syslog.h>

#include "led_fast.h"
#include "spike/hub/display.h"
#include "spike/hub/light.h"

void main_task(intptr_t exinf)
{
  (void)exinf;
  char digit = '0';
  syslog(LOG_NOTICE, "LED fast counter started.");
  while (1) {
    hub_display_off();
    hub_display_char(digit);
    hub_light_on_color(PBIO_COLOR_YELLOW);
    dly_tsk(250 * 1000);
    digit++;
    if (digit > '9') digit = '0';
  }
}
