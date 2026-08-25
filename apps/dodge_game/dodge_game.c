// SPDX-License-Identifier: MIT

#include <kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "dodge_game.h"

#include "spike/hub/button.h"
#include "spike/hub/display.h"
#include "spike/hub/light.h"

#define DISPLAY_SIZE              5
#define PLAYER_ROW                4
#define INPUT_TICK_US         20000U
#define START_FALL_INTERVAL_US 550000U
#define MIN_FALL_INTERVAL_US   175000U
#define SPEEDUP_PER_POINT_US     25000U

static uint32_t rng_state = 0x5a17c9e3U;

static hub_button_t read_buttons(void)
{
  hub_button_t pressed = 0;
  hub_button_is_pressed(&pressed);
  return pressed;
}

static void wait_for_center_press(void)
{
  while ((read_buttons() & HUB_BUTTON_CENTER) == 0) {
    dly_tsk(INPUT_TICK_US);
  }

  while ((read_buttons() & HUB_BUTTON_CENTER) != 0) {
    dly_tsk(INPUT_TICK_US);
  }
}

static uint8_t next_obstacle_column(void)
{
  rng_state = rng_state * 1664525U + 1013904223U;
  return (uint8_t)((rng_state >> 16) % DISPLAY_SIZE);
}

static void seed_random_from_time(void)
{
  SYSTIM now;
  get_tim(&now);
  rng_state ^= (uint32_t)now;
  if (rng_state == 0) {
    rng_state = 0x5a17c9e3U;
  }
}

static uint32_t fall_interval_for_score(int score)
{
  uint32_t reduction = (uint32_t)score * SPEEDUP_PER_POINT_US;

  if (reduction >= START_FALL_INTERVAL_US - MIN_FALL_INTERVAL_US) {
    return MIN_FALL_INTERVAL_US;
  }

  return START_FALL_INTERVAL_US - reduction;
}

static void draw_game(uint8_t player_column, uint8_t obstacle_row,
                      uint8_t obstacle_column)
{
  hub_display_off();
  hub_display_pixel(obstacle_row, obstacle_column, 45);
  hub_display_pixel(PLAYER_ROW, player_column, 100);
}

static int play_game(void)
{
  uint8_t player_column = 2;
  uint8_t obstacle_row = 0;
  uint8_t obstacle_column = next_obstacle_column();
  uint32_t fall_elapsed_us = 0;
  uint32_t fall_interval_us = START_FALL_INTERVAL_US;
  hub_button_t previous_buttons = 0;
  int score = 0;

  hub_light_on_color(PBIO_COLOR_GREEN);
  syslog(LOG_NOTICE, "DODGE GAME START");
  syslog(LOG_NOTICE, "LEFT/RIGHT: MOVE");

  draw_game(player_column, obstacle_row, obstacle_column);

  while (1) {
    hub_button_t buttons;
    bool moved = false;

    dly_tsk(INPUT_TICK_US);
    fall_elapsed_us += INPUT_TICK_US;
    buttons = read_buttons();

    if ((buttons & HUB_BUTTON_LEFT) != 0 &&
        (previous_buttons & HUB_BUTTON_LEFT) == 0 &&
        (buttons & HUB_BUTTON_RIGHT) == 0 &&
        player_column > 0) {
      player_column--;
      moved = true;
    }

    if ((buttons & HUB_BUTTON_RIGHT) != 0 &&
        (previous_buttons & HUB_BUTTON_RIGHT) == 0 &&
        (buttons & HUB_BUTTON_LEFT) == 0 &&
        player_column < DISPLAY_SIZE - 1) {
      player_column++;
      moved = true;
    }

    previous_buttons = buttons;

    if (moved) {
      draw_game(player_column, obstacle_row, obstacle_column);
    }

    if (fall_elapsed_us < fall_interval_us) {
      continue;
    }

    fall_elapsed_us = 0;

    if (obstacle_row < PLAYER_ROW) {
      obstacle_row++;
    }

    if (obstacle_row == PLAYER_ROW) {
      if (obstacle_column == player_column) {
        syslog(LOG_NOTICE, "GAME OVER");
        syslog(LOG_NOTICE, "FINAL SCORE: %d", score);
        return score;
      }

      score++;
      fall_interval_us = fall_interval_for_score(score);
      syslog(LOG_NOTICE, "SCORE: %d", score);
      syslog(LOG_NOTICE, "FALL INTERVAL: %u ms",
             (unsigned int)(fall_interval_us / 1000U));

      obstacle_row = 0;
      obstacle_column = next_obstacle_column();
    }

    draw_game(player_column, obstacle_row, obstacle_column);
  }
}

static void show_game_over(int score)
{
  int shown_score = score;

  hub_light_on_color(PBIO_COLOR_RED);
  hub_display_char('X');
  dly_tsk(900000U);

  if (shown_score > 99) {
    shown_score = 99;
  }
  hub_display_number((int8_t)shown_score);
}

void main_task(intptr_t exinf)
{
  int score;

  hub_light_on_color(PBIO_COLOR_YELLOW);
  hub_display_char('G');
  syslog(LOG_NOTICE, "DODGE GAME READY");
  syslog(LOG_NOTICE, "PRESS CENTER TO START");
  wait_for_center_press();
  seed_random_from_time();

  while (1) {
    score = play_game();
    show_game_over(score);
    syslog(LOG_NOTICE, "PRESS CENTER TO RESTART");
    wait_for_center_press();
    seed_random_from_time();
  }
}
