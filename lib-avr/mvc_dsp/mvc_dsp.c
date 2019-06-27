#include "mvc_dsp.h"
#include "mvc_dsp_asm.h"

#include <stdio.h>


int32_t __fir_split_16_P(int16_t input, uint8_t ntaps, const int16_t h[], int16_t z[], int8_t *p_state) {
  int8_t ii, end_ntaps, state = *p_state;
  int32_t accum, tmp;
  int16_t const *p_h;
  int16_t *p_z;
  /* setup the filter */
  accum = 0;
  p_h = h;
  /* calculate the end part */
  p_z = z + state;
  *p_z = input;
  end_ntaps = ntaps - state;
  for (ii = 0; ii < end_ntaps; ii++)  {
    MultiS16X16to32(tmp, (int16_t)pgm_read_word(p_h++), *p_z++);
    accum +=  tmp;
  }  
  /* calculate the beginning part */
  p_z = z;
  for (ii = 0; ii < state; ii++) {
    MultiS16X16to32(tmp, (int16_t)pgm_read_word(p_h++), *p_z++);
    accum +=  tmp;
  }
  /* decrement the state, wrapping if below zero */
  if (--state < 0) state += ntaps;
  *p_state = state;       /* pass new state back to caller */
  return accum;
}


int32_t __fir_split_16(int16_t input, uint8_t ntaps, const int16_t h[], int16_t z[], int8_t *p_state) {
  int8_t ii, end_ntaps, state = *p_state;
  int32_t accum, tmp;
  int16_t const *p_h;
  int16_t *p_z;
  /* setup the filter */
  accum = 0;
  p_h = h;
  /* calculate the end part */
  p_z = z + state;
  *p_z = input;
  end_ntaps = ntaps - state;
  for (ii = 0; ii < end_ntaps; ii++) {
    MultiS16X16to32(tmp, *p_h++, *p_z++);
    accum += tmp;
  }  
  /* calculate the beginning part */
  p_z = z;
  for (ii = 0; ii < state; ii++) {
    MultiS16X16to32(tmp, *p_h++, *p_z++);
    accum += tmp;
  }  
  /* decrement the state, wrapping if below zero */
  if (--state < 0) state += ntaps;
  *p_state = state;       /* pass new state back to caller */
  return accum;
}








