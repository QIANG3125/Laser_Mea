#ifndef __LASER_H
#define __LASER_H
#include "sys.h"

//#define X5_1 PBin(7)
#define X5_2 PBout(6)
//#define X5_3 PBout(5)
//#define X5_5 PBin(4)
#define X5_8 PBout(3)
#define X5_10 PAout(15)

#define GA PCin(13)

void laser_init(void);
void laser_plus(void);
unsigned char laser_on(void);
void nop(void);
void gpioc_to_io(void);

#endif
