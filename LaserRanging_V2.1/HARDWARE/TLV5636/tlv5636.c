/* TLV5636驱动程序 */
#include "tlv5636.h"
#include "delay.h"

/* 引脚配置初始化 */
void tlv5636_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 
	GPIO_SetBits(GPIOB,GPIO_Pin_1|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
	
	TLV5636_CS = 0;
	TLV5636_SCLK = 1;
	TLV5636_FS= 0;
	TLV5636_DIN = 0;
	
}

/* 通信时序 */
void tlv5636SendData16(u16 data)
{
	int i;
	TLV5636_CS = 1;
	TLV5636_FS = 0;
	TLV5636_FS = 1;
	delay_us(1);
	TLV5636_CS = 0;
	TLV5636_FS = 0;
	for(i=15;i>=0;i--)
	{
		delay_us(1);
		TLV5636_DIN = (data>>i)&0x0001;
		TLV5636_SCLK = 0;
		delay_us(1);
		TLV5636_SCLK = 1;
	}
	TLV5636_SCLK = 1;
	TLV5636_FS = 1;
	TLV5636_CS = 1;
	delay_us(1);
}

/* 设置参考电压 */
/* the tlv5636 has a x2 Gain voltage output */
/* REF1 0x01   1.024V *2 */
/* REF2 0x02   2.048V *2 */
/* REF_EX 0x00 external ref *2 */
int setRefValue(u8 ref)
{
	if(ref<4)
	{
		tlv5636SendData16(ref|0xD000);
		return 1;
	}
	else
		return 0;
}

/* 设置DAC输出电压0~4095 */
int setDacValueBin(u16 data)
{
	if(data<4096)
	{
		tlv5636SendData16(data|0x4000);
		return 1;
	}
	else
		return 0;
	
}


