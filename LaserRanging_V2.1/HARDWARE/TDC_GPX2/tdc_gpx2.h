#ifndef __TDC_GPX2_H
#define __TDC_GPX2_H
#include "sys.h"

#define false 0
#define true 1

#define  RSTIDX PAout(8)
#define GPIO_INTERRUPT PBin(2)
#define DISABLE PAout(12)
#define PARITY PBin(5)

/* 结果结构体 */
typedef struct result_format{
	int reference_index[4];
	int stopresult[4];
}result,*p_result;


void tdc_init(void);
void tdc_config(void);
void tdc_measure(p_result presult);
unsigned char tdc_measure_group(p_result presult);

#endif
