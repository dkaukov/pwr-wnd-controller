#ifndef __MVC_DSP_H__
#define __MVC_DSP_H__

#include <stdint.h>
#include <avr/pgmspace.h>

typedef struct fir_filter16 fir_filter16_t;
struct fir_filter16 {
  uint8_t ntaps;
  int8_t  state;
  const int16_t *h;
  int16_t *buff;
};

#ifdef __cplusplus
extern "C"{
#endif
  int32_t __fir_split_16_P(int16_t input, uint8_t ntaps, const int16_t h[], int16_t z[], int8_t *p_state);
  int32_t __fir_split_16  (int16_t input, uint8_t ntaps, const int16_t h[], int16_t z[], int8_t *p_state);
#ifdef __cplusplus
} // extern "C"
#endif

// API
inline int16_t fir_16_P(fir_filter16_t *fir, int16_t sample) {
  return __fir_split_16_P(sample, fir->ntaps, fir->h, fir->buff, &fir->state) >> 10;
}

inline int16_t fir_16(fir_filter16_t *fir, int16_t sample) {
  union {int32_t val; int16_t raw[2]; } tmp;
  tmp.val = __fir_split_16(sample, fir->ntaps, fir->h, fir->buff, &fir->state);
  return tmp.raw[0];
}


#endif
