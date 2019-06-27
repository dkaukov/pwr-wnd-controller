// Arduino Pro Mini 16 Mhz, ATMega328

#include "WProgram.h"
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#ifdef PROGMEM
  #undef PROGMEM
  #define PROGMEM __attribute__((section(".progmem.data")))
#endif

#undef PSTR
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];}))

// EEPROM
#include <nvram.h>

// GUI Serial
#define AVR_USART_PORT  0
#define USART_RX_BUFFER_SIZE 32
#define USART_TX_BUFFER_SIZE 32
#include "avr_usart.h"
inline void GUI_serial_open(uint32_t baud) {avr_UsartOpen_0(baud);}
inline void GUI_serial_close() {avr_UsartClose_0();}
inline uint8_t GUI_serial_available() {return avr_UsartAvailable_0();}
inline uint8_t GUI_serial_tx_full() {return avr_UsartTXFull_0();}
inline uint8_t GUI_serial_read() {return avr_UsartRead_0();}
inline void GUI_serial_write(uint8_t c) {avr_UsartWrite_0(c);}

inline void CLI_serial_open(uint32_t baud) {avr_UsartOpen_0(baud);}
inline void CLI_serial_close() {avr_UsartClose_0();}
inline uint8_t CLI_serial_available() {return avr_UsartAvailable_0();}
inline uint8_t CLI_serial_read() {return avr_UsartRead_0();}
inline void CLI_serial_write(uint8_t c) {avr_UsartWrite_0(c);}

#define TICKS_PER_US (F_CPU / 8000000L)

ISR(PCINT1_vect) {
}

// Delay support
void __delay_us(uint16_t __us) __attribute__ ((noinline));
void __delay_us(uint16_t __us) {
#if (TICKS_PER_US == 2)
  __us = (__us << 1) - 4;
#else
  __us = __us  - 4;
#endif
  uint16_t i_start = __systick();
  while (__interval(i_start) < __us) {};
};

void __delay_ms(uint16_t __ms) __attribute__ ((noinline));
void __delay_ms(uint16_t __ms) {
  while (__ms--) __delay_us(1000);
}

inline uint16_t __systick() {
  uint8_t __sreg = SREG;
  cli();
  uint16_t res = TCNT1;
  SREG = __sreg;
  return res;
}

inline uint16_t __interval(uint16_t i_start) {
  uint8_t __sreg = SREG;
  cli();
  uint16_t res = __interval(i_start, TCNT1);
  SREG = __sreg;
  return res;
}

inline uint16_t __interval(uint16_t i_start, uint16_t i_end) {
  return (i_end - i_start);
}

inline void StatusLEDOn() {
  PORTB |= _BV(5);
}

inline void StatusLEDOff() {
  PORTB &= ~_BV(5);
}

inline void StatusLEDToggle() {
  PINB |= _BV(5);
}

inline void start_current_adc(uint8_t ch) {
  uint8_t tmp = _BV(REFS0);
  switch (ch) {
    case 0: tmp |= 0x7; break;
    case 1: tmp |= 0x6; break;
    case 2: tmp |= 0x5; break;
  }
  ADMUX = tmp;
  ADCSRA |= _BV(ADSC);
}

inline uint8_t is_current_adc_finished(){
  return !bit_is_set(ADCSRA, ADSC);
}

int16_t get_current_adc(){
  uint8_t low, high;
  low  = ADCL;  high = ADCH;
  int16_t val = (high << 8) | low;
  return val - 512;
}

void Board_Idle() {
  avr_UsartPollWrite_0();
};

void _up(uint8_t ch) {
  switch (ch) {
    case 0:
      digitalWrite(8, HIGH);
      digitalWrite(7, LOW);
      break;
    case 1:
      digitalWrite(6, HIGH);
      digitalWrite(5, LOW);
      break;
    case 2:
      digitalWrite(4, HIGH);
      digitalWrite(3, LOW);
      break;
  }
}

void _down(uint8_t ch) {
  switch (ch) {
    case 0:
      digitalWrite(8, LOW);
      digitalWrite(7, HIGH);
      break;
    case 1:
      digitalWrite(6, LOW);
      digitalWrite(5, HIGH);
      break;
    case 2:
      digitalWrite(4, LOW);
      digitalWrite(3, HIGH);
      break;
  }
}

void _stop(uint8_t ch) {
  switch (ch) {
    case 0:
      digitalWrite(8, LOW);
      digitalWrite(7, LOW);
      break;
    case 1:
      digitalWrite(6, LOW);
      digitalWrite(5, LOW);
      break;
    case 2:
      digitalWrite(4, LOW);
      digitalWrite(3, LOW);
      break;
  }
}

void _lock(uint8_t ch) {
  switch (ch) {
    case 0:
      digitalWrite(8, HIGH);
      digitalWrite(7, HIGH);
      break;
    case 1:
      digitalWrite(6, HIGH);
      digitalWrite(5, HIGH);
      break;
    case 2:
      digitalWrite(4, HIGH);
      digitalWrite(3, HIGH);
      break;
  }
}

uint8_t _button_state(uint8_t ch) {
  switch (ch) {
    case 0:
      if ((digitalRead(9)  == HIGH) and (digitalRead(10) == HIGH)) return BUTTON_STATE_LOCK;
      if (digitalRead(9)  == HIGH) return BUTTON_STATE_UP;
      if (digitalRead(10) == HIGH) return BUTTON_STATE_DOWN;
      break;
    case 1:
      if ((digitalRead(11)  == HIGH) and (digitalRead(12) == HIGH)) return BUTTON_STATE_LOCK;
      if (digitalRead(11) == HIGH) return BUTTON_STATE_UP;
      if (digitalRead(12) == HIGH) return BUTTON_STATE_DOWN;
      break;
    case 2:
      if ((digitalRead(A0)  == HIGH) and (digitalRead(A1) == HIGH)) return BUTTON_STATE_LOCK;
      if (digitalRead(A0) == HIGH) return BUTTON_STATE_UP;
      if (digitalRead(A1) == HIGH) return BUTTON_STATE_DOWN;
      break;
  }
  return BUTTON_STATE_NONE;
}

void _driver_wnd_up() {
  digitalWrite(A4, LOW);
}

void _driver_wnd_stop() {
  digitalWrite(A4, HIGH);
}

uint8_t _lock_state() {
  return digitalRead(A3);
}

void _aux_pwr_on() {
  digitalWrite(2, HIGH);
}

void _aux_pwr_off() {
  digitalWrite(2, LOW);
}

uint8_t _ignition_state() {
  return digitalRead(A2);
}

void board_sleep() {
  PCMSK1 |= _BV(PCINT10) | _BV(PCINT11); // Lock or Ignition
  PCICR  |= _BV(PCIE1);
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
  power_usart0_disable();
  sleep_bod_disable();
  sei();
  StatusLEDOff();
  sleep_cpu();
  StatusLEDOn();
  PCICR  &= ~ _BV(PCIE1);
  power_adc_enable();
  power_spi_enable();
  power_twi_enable();
  power_usart0_enable();
}


inline void Board_Init() {
  TIMSK0 = 0;
  // Timer1
  TCCR1A = 0;
  TCCR1B = _BV(CS11);                 /* div 8 clock prescaler */
  // Serial Init
  GUI_serial_open(SERIAL_COM_SPEED);
  // LED
  pinMode (13, OUTPUT);
  // CH1
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  // CH2
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  // CH3
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  // Dr. Door
  pinMode(A4, OUTPUT);
  digitalWrite(A4, HIGH);
  // Lock
  pinMode(A3, INPUT);
  // AUX Power
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  // Ignition
  pinMode(A2, INPUT);
  // ADC
  DIDR0 |= _BV(ADC5D) /*| _BV(ADC6D) | _BV(ADC7D)*/;
  ADCSRA |= _BV(ADEN);
  ADCSRA |= _BV(ADSC);
  // Power management
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
}
