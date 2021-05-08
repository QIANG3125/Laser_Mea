#ifndef __TLV5636_H
#define __TLV5636_H
#include "sys.h"

/* the tlv5636 has a x2 Gain voltage output */
#define REF1 0x01   //1.024V *2
#define REF2 0x02   //2.048V *2
#define REF_EX 0x00 //external ref *2

#define TLV5636_DIN PBout(1)
#define TLV5636_SCLK PBout(7)
#define TLV5636_CS PBout(8)
#define TLV5636_FS PBout(9)

void tlv5636_init(void);
void tlv5636SendData16(u16 data);
int setRefValue(u8 ref);
int setDacValueBin(u16 data);

#endif


