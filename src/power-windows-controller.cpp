/**
 * Power Windows Controller - 2013
 * Main sketch file.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <pt.h>
#include <pt-sem.h>
#include "Global.h"
#include "config.h"
#include "Core.h"
#include "LMath.h"
#include "Debug.h"
#include "Storage.h"

struct  ch_ctx_data ch_ctx[CHANNEL_COUNT];
uint8_t ignition_state;
uint8_t lock_state;
uint8_t state_change;

void debug_print_system_state() {
  //return;
  dprintf("\033[1;1H");
  dprintf("CPU: %8d%%  \n",  cpu_util_pct * 100 / 255);
  dprintf("I (c0, c1, c2): %8d, %8d, %8d  \n",  ch_ctx[0].curr >> 4,  ch_ctx[1].curr >> 4,  ch_ctx[2].curr >> 4);
  dprintf("Im (c0, c1, c2): %8d, %8d, %8d  \n",  ch_ctx[0].max_curr >> 4,  ch_ctx[1].max_curr >> 4,  ch_ctx[2].max_curr >> 4);
  dprintf("evt (c0, c1, c2): %8d, %8d, %8d  \n",  ch_ctx[0].evt,  ch_ctx[1].evt,  ch_ctx[2].evt);
  dprintf("lock_state: %8d      \n",  lock_state);
}

void send_message(uint8_t msg) {
}

void setup() {
  cli();
  Board_Init();
  Debug_Init();
  Storage_Init();
  sei();
}

static struct timer_small timer_adc_ctrl   = {0, ADC_CTRL_LOOP_TIME   * TICKS_PER_US};
static struct timer_small timer_input_ctrl = {0, INPUT_CTRL_LOOP_TIME * TICKS_PER_US};
static struct timer_big   timer_service    = {0, SERVICE_LOOP_TIME    * TICKS_PER_US};
static struct timer_big   timer_pwr_delay;
static struct timer_big   timer_auto_close_delay;

static struct pt thread_adc_ctrl_pt;
static struct pt thread_input_ctrl_pt;
static struct pt thread_service_pt;
static struct pt thread_pwr_mgr_pt;
static struct pt thread_auto_close_pt;

#define IDLE_LOOP_PERIOD 33 // measured
#define IDLE_LOOP_CNT (SERVICE_LOOP_TIME / IDLE_LOOP_PERIOD)
static uint16_t main_loop_cnt;

void update_current_limiter(uint8_t ch) {
  if (ch_ctx[ch].curr > ch_ctx[ch].max_curr) {
    ch_ctx[ch].max_curr = ch_ctx[ch].curr;
    ch_ctx[ch].curr_lim = ch_ctx[ch].curr;
    ch_ctx[ch].curr_lim -= ch_ctx[ch].curr >> 2;
  }
}

static PT_THREAD(thread_ch_up(struct pt *pt, uint16_t dt, uint8_t ch)) {
  uint8_t curr_lim;
  PT_BEGIN(pt);
  _up(ch);
  timer_reset(&ch_ctx[ch].timer_rampup, US_TO_TICKS(US_RAMPUP_UP_TIME), dt);
  PT_WAIT_UNTIL(pt, timer_expired(&ch_ctx[ch].timer_rampup, dt));
  if (ch_ctx[ch].evt == BUTTON_STATE_NONE) ch_ctx[ch].evt = BUTTON_STATE_AUTO_UP;
  timer_reset(&ch_ctx[ch].timer_run_timeout, US_TO_TICKS(US_MAX_RUN_TIME), dt);
  PT_WAIT_UNTIL(pt, timer_expired(&ch_ctx[ch].timer_run_timeout, dt) or (curr_lim = (ch_ctx[ch].curr >= ch_ctx[ch].curr_lim)) or (ch_ctx[ch].evt != BUTTON_STATE_UP) and ((ch_ctx[ch].evt != BUTTON_STATE_AUTO_UP)));
  update_current_limiter(ch);
  _stop(ch);
  if (curr_lim) {
    PT_WAIT_UNTIL(pt, (ch_ctx[ch].evt != BUTTON_STATE_UP));
  }
  ch_ctx[ch].evt = BUTTON_STATE_NONE;
  PT_END(pt);
}

static PT_THREAD(thread_ch_down(struct pt *pt, uint16_t dt, uint8_t ch)) {
  uint8_t curr_lim;
  PT_BEGIN(pt);
  _down(ch);
  timer_reset(&ch_ctx[ch].timer_rampup, US_TO_TICKS(US_RAMPUP_DOWN_TIME), dt);
  PT_WAIT_UNTIL(pt, timer_expired(&ch_ctx[ch].timer_rampup, dt));
  if (ch_ctx[ch].evt == BUTTON_STATE_NONE) ch_ctx[ch].evt = BUTTON_STATE_AUTO_DOWN;
  timer_reset(&ch_ctx[ch].timer_run_timeout, US_TO_TICKS(US_MAX_RUN_TIME), dt);
  PT_WAIT_UNTIL(pt, timer_expired(&ch_ctx[ch].timer_run_timeout, dt) or (curr_lim = (ch_ctx[ch].curr >= ch_ctx[ch].curr_lim)) or ((ch_ctx[ch].evt != BUTTON_STATE_DOWN) and (ch_ctx[ch].evt != BUTTON_STATE_AUTO_DOWN)));
  update_current_limiter(ch);
  _stop(ch);
  if (curr_lim) {
    PT_WAIT_UNTIL(pt, (ch_ctx[ch].evt != BUTTON_STATE_DOWN));
  }
  ch_ctx[ch].evt = BUTTON_STATE_NONE;
  PT_END(pt);
}

static PT_THREAD(thread_ch_manager(struct pt *pt, uint16_t dt, uint8_t ch)) {
  PT_BEGIN(pt);
  if ((ch_ctx[ch].evt == BUTTON_STATE_UP) or (ch_ctx[ch].evt == BUTTON_STATE_AUTO_UP)) {
    PT_INIT(&ch_ctx[ch].thread_run_pt);
    PT_SPAWN(pt, &ch_ctx[ch].thread_run_pt, thread_ch_up(&ch_ctx[ch].thread_run_pt, dt, ch));
  } else if ((ch_ctx[ch].evt == BUTTON_STATE_DOWN) or (ch_ctx[ch].evt == BUTTON_STATE_AUTO_DOWN)) {
    PT_INIT(&ch_ctx[ch].thread_run_pt);
    PT_SPAWN(pt, &ch_ctx[ch].thread_run_pt, thread_ch_down(&ch_ctx[ch].thread_run_pt, dt, ch));
  } else if ((ch_ctx[ch].btn_state == BUTTON_STATE_LOCK) and (ignition_state == HIGH)) {
    _lock(ch);
  } else _stop(ch);
  PT_END(pt);
}

static PT_THREAD(thread_adc_ctrl(struct pt *pt, uint16_t dt)) {
  static uint8_t ch = 0;
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_adc_ctrl, dt));
  current_time_us += ADC_CTRL_LOOP_TIME;
  current_time_ms += (ADC_CTRL_LOOP_TIME / 1000);
  if (is_current_adc_finished()) {
    int16_t lpf = ch_ctx[ch].curr;
    lpf += ((get_current_adc() << 4) -  lpf) >> 4;
    ch_ctx[ch].curr = lpf;
    if (++ch >= CHANNEL_COUNT) ch = 0;
    start_current_adc(ch);
  }
  PT_END(pt);
}

static PT_THREAD(thread_input_ctrl(struct pt *pt, uint16_t dt)) {
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_input_ctrl, dt));
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    uint8_t state = _button_state(i);
    if (ch_ctx[i].btn_state != state) {
      ch_ctx[i].evt = state;
      ch_ctx[i].btn_state = state;
      state_change = 1;
    }
  }
  uint8_t state = _ignition_state();
  if (state != ignition_state) {
    ignition_state = state;
    state_change = 1;
  }
  lock_state = _lock_state();
  PT_END(pt);
}

static PT_THREAD(thread_service(struct pt *pt, uint16_t dt)) {
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_service, dt));
  // Calc CPU utilization
  uint16_t delta = main_loop_cnt;
  uint8_t idle_pct = ((uint32_t)delta << 8) / IDLE_LOOP_CNT;
  cpu_util_pct = 255 - idle_pct;
  main_loop_cnt = 0;
  debug_print_system_state();
  StatusLEDToggle();
  PT_END(pt);
}

static PT_THREAD(thread_auto_close(struct pt *pt, uint16_t dt)) {
  uint8_t timeout;
  PT_BEGIN(pt);
  lock_state = _lock_state();
  PT_WAIT_UNTIL(pt, (ignition_state == LOW) && (lock_state == HIGH));
  PT_WAIT_UNTIL(pt, (lock_state == LOW));
  timer_reset(&timer_auto_close_delay, US_TO_TICKS(US_AUTO_CLOSE_DELAY), dt);
  PT_WAIT_UNTIL(pt, ((timeout = timer_expired(&timer_auto_close_delay, dt)) or (_lock_state() == HIGH)));
  if (timeout) {
    _driver_wnd_up();
    state_change = 1;
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
      ch_ctx[i].evt = BUTTON_STATE_AUTO_UP;
    timer_reset(&timer_auto_close_delay, US_TO_TICKS(US_MAX_RUN_TIME), dt);
    PT_WAIT_UNTIL(pt, timer_expired(&timer_auto_close_delay, dt));
    _driver_wnd_stop();
  } else {
    PT_WAIT_UNTIL(pt, timer_expired(&timer_auto_close_delay, dt));
    PT_WAIT_UNTIL(pt, (_lock_state() == LOW));
  }
  PT_END(pt);
}

static PT_THREAD(thread_pwr_mgr(struct pt *pt, uint16_t dt)) {
  uint8_t timeout;
  PT_BEGIN(pt);
  _aux_pwr_on();
  PT_WAIT_UNTIL(pt, ignition_state == LOW);
  timer_reset(&timer_pwr_delay, US_TO_TICKS(US_AUX_PWR_DELAY), dt); state_change = 0;
  PT_WAIT_UNTIL(pt, (timeout = timer_expired(&timer_pwr_delay, dt)) or (state_change != 0) or (_ignition_state() == HIGH));
  if (timeout) {
    _aux_pwr_off();
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
     _stop(i);
     _driver_wnd_stop();
    board_sleep();
  }
  PT_END(pt);
}

void loop() __attribute__ ((noreturn));
void loop() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
    PT_INIT(&ch_ctx[i].thread_mgr_pt);
  PT_INIT(&thread_adc_ctrl_pt);
  PT_INIT(&thread_input_ctrl_pt);
  PT_INIT(&thread_service_pt);
  PT_INIT(&thread_pwr_mgr_pt);
  PT_INIT(&thread_auto_close_pt);
  for (;;) {
    main_loop_cnt++;
    Board_Idle();
    uint16_t current_tick = __systick();
    PT_SCHEDULE(thread_adc_ctrl(&thread_adc_ctrl_pt, current_tick));
    PT_SCHEDULE(thread_input_ctrl(&thread_input_ctrl_pt, current_tick));
    PT_SCHEDULE(thread_service(&thread_service_pt, current_tick));
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
      PT_SCHEDULE(thread_ch_manager(&ch_ctx[i].thread_mgr_pt, current_tick, i));
    PT_SCHEDULE(thread_pwr_mgr(&thread_pwr_mgr_pt, current_tick));
    PT_SCHEDULE(thread_auto_close(&thread_auto_close_pt, current_tick));
  }
}

