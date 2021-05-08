#ifndef __AGC_H
#define __AGC_H
#include "sys.h"

#define PEAK_CONTROL PAout(4)

extern volatile u16 ADC_ConvertedValue;

void agc_init(void);
void adc_init(void);

#endif

