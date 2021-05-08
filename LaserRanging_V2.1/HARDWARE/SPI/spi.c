#include "spi.h"
#include "delay.h"

/* 初始化引脚配置 */
void spi_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	 
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_13|GPIO_Pin_14;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 
	GPIO_ResetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_13|GPIO_Pin_14);	//SSN.GPIO_SCK初始化输出低		

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //上拉输入
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOA,GPIO_Pin_8);
	SCK = 0;
}

/* 发送 8byte*/
void send_byte_to_SPI(unsigned char send_data)
{
	int data_idx;
	SCK = 0;
	for(data_idx=7;data_idx>=0;data_idx--)
	{
		delay_us(10);
		MOSI = (send_data>>data_idx)&0x01;
		SCK = 1;
		delay_us(10);
		SCK = 0;
	}
}

/* 接收 8byte*/
void read_byte_from_SPI(unsigned char *read_data)
{
	int data_idx;
	unsigned char *p_read_data = read_data;
	*p_read_data = 0x00;
	for(data_idx=7;data_idx>=0;data_idx--)
	{
		SCK = 1;
		delay_us(10);
		SCK = 0;
		*p_read_data = (*p_read_data|((MISO&0x01)<<data_idx));
		delay_us(10);
	}
}
