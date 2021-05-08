/* TPL0202双通道阈值控制驱动 */
 
#include "tpl0202.h"
#include "delay.h"

#define WWRA 0x01
#define WWRB 0x02
#define WWRAB 0x03
#define WNVRA 0x11
#define WNVRB 0x12
#define WNVRAB 0x13
#define CWRATONV 0x21
#define CWRBTONV 0x22
#define CWRABTONV 0x23
#define CNVATOWR 0x31
#define CNVBTOWR 0x32
#define CNVABTOWR 0x33

/* 初始化引脚配置 */
void tpl0202_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	 
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 
	GPIO_ResetBits(GPIOA,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7);		

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 
	GPIO_SetBits(GPIOB,GPIO_Pin_0);	
	GPIO_SetBits(GPIOA,GPIO_Pin_3);	
}

void send_spi_16(unsigned char hdr_byte,unsigned char data_byte,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D)
{
	unsigned int package_buf = 0;
	int data_idx;
	
	package_buf = 0;
	package_buf = ((package_buf|hdr_byte)<<8)|data_byte;
	
	*P_TPL_CS = 1;
	*P_TPL_CS = 0;
	delay_us(1);
	*P_TPL_CLK = 0;
	
	for(data_idx=15;data_idx>=0;data_idx--)
	{
		delay_us(10);
		*P_TPL_D = (package_buf>>data_idx)&0x01;
		delay_us(10);
		*P_TPL_CLK = 1;
		delay_us(10);
		*P_TPL_CLK = 0;
	}
	delay_us(10);
	*P_TPL_CS = 1;
}

void T_send_spi_16(unsigned char hdr_byte,unsigned char data_byte)
{
	unsigned int package_buf = 0;
	int data_idx;
	
	package_buf = 0;
	package_buf = ((package_buf|hdr_byte)<<8)|data_byte;
	
	TPL1_CS = 1;
	TPL1_CS = 0;
	delay_us(1);
	TPL1_CLK = 0;
	
	for(data_idx=15;data_idx>=0;data_idx--)
	{
		delay_us(10);
		TPL1_D = (package_buf>>data_idx)&0x01;
		delay_us(10);
		TPL1_CLK = 1;
		delay_us(10);
		TPL1_CLK = 0;
	}
	delay_us(10);
	TPL1_CS = 1;
}


void T_write_wa(unsigned char value)
{
	T_send_spi_16(WWRA,value);
}

void T_write_wb(unsigned char value)
{
	T_send_spi_16(WWRB,value);
}

/* 设置通道a的阈值电压 */
/* value:0~255 */
/* P_TPL_CS P_TPL_CLK P_TPL_D:引脚地址 */
void write_wa(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D)
{
	send_spi_16(WWRA,value,P_TPL_CS,P_TPL_CLK,P_TPL_D);
}

/* 设置通道b的阈值电压 */
/* value:0~255 */
/* P_TPL_CS P_TPL_CLK P_TPL_D:引脚地址 */
void write_wb(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D)
{
	send_spi_16(WWRB,value,P_TPL_CS,P_TPL_CLK,P_TPL_D);
}

void write_nva(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D)
{
	send_spi_16(WNVRA,value,P_TPL_CS,P_TPL_CLK,P_TPL_D);
}

void write_nvb(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D)
{
	send_spi_16(WNVRB,value,P_TPL_CS,P_TPL_CLK,P_TPL_D);
}

void copy_both_nv_to_wr(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D)
{
	send_spi_16(CNVABTOWR,0,P_TPL_CS,P_TPL_CLK,P_TPL_D);
}

void copy_both_wr_to_nv(unsigned char value,volatile unsigned long *P_TPL_CS,volatile unsigned long *P_TPL_CLK,volatile unsigned long *P_TPL_D)
{
	send_spi_16(CWRABTONV,0,P_TPL_CS,P_TPL_CLK,P_TPL_D);
}
