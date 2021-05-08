#ifndef __TPL0202_H
#define __TPL0202_H
#include "sys.h"

#define TPL1_CS PBout(0)
#define TPL1_CLK PAout(6)
#define TPL1_D PAout(7)
#define TPL2_CLK PAout(1)
#define TPL2_D PAout(2)
#define TPL2_CS PAout(3)

#define TPL1_CS_GPIO GPIOB
#define TPL1_CLK_GPIO GPIOA
#define TPL1_D_GPIO GPIOA
#define TPL2_CLK_GPIO GPIOA
#define TPL2_D_GPIO GPIOA
#define TPL2_CS_GPIO GPIOA
#define TPL1_CS_PIN GPIO_Pin_0
#define TPL1_CLK_PIN GPIO_Pin_6
#define TPL1_D_PIN GPIO_Pin_7
#define TPL2_CLK_PIN GPIO_Pin_1
#define TPL2_D_PIN GPIO_Pin_2
#define TPL2_CS_PIN GPIO_Pin_3

extern unsigned int package_buf;

void tpl0202_init(void);
void send_spi_16(unsigned char hdr_byte,unsigned char data_byte,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D);
void write_wa(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D);
void write_wb(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D);
void write_nva(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D);
void write_nvb(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D);
void copy_both_nv_to_wr(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D);
void copy_both_wr_to_nv(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D);


void T_write_wa(unsigned char value);
void T_write_wb(unsigned char value);

#endif
