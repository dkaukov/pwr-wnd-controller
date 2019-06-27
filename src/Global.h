/**
 * MultiWii NG 0.1 - 2012
 * Global definitions
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

#ifndef WMC_global_h
#define WMC_global_h

////////////////////////////////////////////////////////////////////////////////
/// Setup GCC environment
//  Turn on/off warnings of interest.
//
// These warnings are normally suppressed by the Arduino IDE,
// but with some minor hacks it's possible to have warnings
// emitted.  This helps greatly when diagnosing subtle issues.
//
#pragma GCC diagnostic warning "-Wall"
#pragma GCC diagnostic warning "-Winline"
#pragma GCC diagnostic warning "-Wextra"
#pragma GCC diagnostic warning "-Wlogical-op"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wattributes"

////////////////////////////////////////////////////////////////////////////////
///
//  Preprocessor constants
//
#define _NONE_         000
#define _PROMINI_      100

////////////////////////////////////////////////////////////////////////////////
///
//  Unit conversions
//
#define US_TO_TICKS(x)      ((x) * (uint8_t)TICKS_PER_US)


#define BUTTON_STATE_NONE      0
#define BUTTON_STATE_UP        1
#define BUTTON_STATE_DOWN      2
#define BUTTON_STATE_AUTO_UP   3
#define BUTTON_STATE_AUTO_DOWN 4
#define BUTTON_STATE_LOCK      5

#define ADC_CTRL_LOOP_TIME      (1000L)
#define INPUT_CTRL_LOOP_TIME    (20000L)
#define SERVICE_LOOP_TIME       (250000L)

#define __PACKED__ __attribute__((__packed__))

#include <pt.h>

enum enum_led_patterns {
    LED_PATTERN_OFF               = 0b00000000,
    LED_PATTERN_FAST_BLINK        = 0b00000010,
    LED_PATTERN_SLOW_BLINK        = 0b11001100,
    LED_PATTERN_SHORT_BLINK       = 0b00000001,
    LED_PATTERN_ON                = 0b11111111,
    LED_PATTERN_CALIBRATION_END   = 0b00010101,
    LED_PATTERN_CALIBRATION_START = 0b11011011,
    LED_PATTERN_SHORT_BANK        = 0b11100111,
};

enum enum_beep_patterns {
    BEEP_PATTERN_OFF              = 0b00000000,
    BEEP_PATTERN_FAST_BLINK       = 0b00000010,
    BEEP_PATTERN_SLOW_BLINK       = 0b11001100,
    BEEP_PATTERN_SHORT_BLINK      = 0b00000001,
    BEEP_PATTERN_ON               = 0b11111111,
    BEEP_PATTERN_CALIBRATION_END  = 0b00010101,
    BEEP_PATTERN_VBAT_W1          = 0b01001001,
    BEEP_PATTERN_VBAT_W2          = 0b01010101,
};

static uint32_t current_time_us;
static uint32_t current_time_ms;
static uint8_t  cpu_util_pct;
static uint8_t  sys_param_values_cnt;
static int16_t  batt_voltage;

// Core Function prototypes
// GUI Serial
void GUI_serial_open(uint32_t baud);
void GUI_serial_close();
uint8_t GUI_serial_available();
uint8_t GUI_serial_read();
void GUI_serial_write(uint8_t c);
// CLI/DEBUG Serial
void CLI_serial_open(uint32_t baud);
void CLI_serial_close();
uint8_t CLI_serial_available();
uint8_t CLI_serial_read();
void CLI_serial_write(uint8_t c);
// SysTick
inline uint16_t __systick();
inline uint16_t __interval(uint16_t i_start, uint16_t i_end);
inline uint16_t __interval(uint16_t i_start);
// Init
void Board_Init();

typedef struct timer_small timer_small_t;
struct timer_small { uint16_t elapsed, interval; uint16_t last_systick;};
typedef struct timer_big timer_big_t;
struct timer_big   { uint32_t elapsed, interval; uint16_t last_systick;};

uint8_t timer_expired(timer_small_t *t, uint16_t systick = __systick()) {
  uint16_t dt = __interval(t->last_systick, systick);
  t->last_systick = systick;
  if (t->elapsed < dt) {
    t->elapsed = t->interval;
    return 1;
  } ;
  t->elapsed -= dt;
  return 0;
};

uint8_t timer_expired(timer_big_t *t, uint16_t systick = __systick()) {
  uint16_t dt = __interval(t->last_systick, systick);
  t->last_systick = systick;
  if (t->elapsed < dt) {
    t->elapsed = t->interval;
    return 1;
  } ;
  t->elapsed -= dt;
  return 0;
};

void timer_reset(timer_small_t *t, uint16_t elapsed, uint16_t systick = __systick()) {
  t->elapsed = elapsed;
  t->last_systick = systick;
}

void timer_reset(timer_big_t *t, uint32_t elapsed, uint16_t systick = __systick()) {
  t->elapsed = elapsed;
  t->last_systick = systick;
}


inline void blink_led(uint8_t pattern);
inline void beep(uint8_t pattern);
void read_storage();
void write_storage();

struct ch_ctx_data {
  struct timer_big timer_run_timeout;
  struct timer_big timer_rampup;
  struct pt thread_run_pt;
  struct pt thread_mgr_pt;
  int16_t curr;
  int16_t curr_lim;
  int16_t max_curr;
  uint8_t evt;
  uint8_t btn_state;
};


#endif

