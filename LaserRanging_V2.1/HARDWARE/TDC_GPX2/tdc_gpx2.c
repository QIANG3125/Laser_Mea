#include "tdc_gpx2.h"
#include "spi.h"
#include "laser.h"
#include "delay.h"
#include "includes.h"

/* 使能4个通道 */
const char config_register[17] = {
	0x0F, 0x0F, 0x1F, 0x48, 0xE8, 0x01, 0xC0, 0xD3,
	0xA1, 0x13, 0x00, 0x0A, 0xCC, 0xCC, 0x31, 0x8E, 0x04
};

//------------------------------------
//***SPI Opcdes***
//------------------------------------
const char spiopc_power        = 0x30;
const char spiopc_init         = 0x18;
const char spiopc_write_config = 0x80;
const char spiopc_read_config  = 0x40;
const char spiopc_read_results = 0x60;

//-------------------------------------
//***SPI Addresses***
//-------------------------------------
const char reference_index_ch1_byte3 = 8;
const char reference_index_ch1_byte2 = 9;
const char reference_index_ch1_byte1 = 10;
const char stopresult_ch1_byte3 = 11;
const char stopresult_ch1_byte2 = 12;
const char stopresult_ch1_byte1 = 13;
const char reference_index_ch2_byte3 = 14;
const char reference_index_ch2_byte2 = 15;
const char reference_index_ch2_byte1 = 16;
const char stopresult_ch2_byte3 = 17;
const char stopresult_ch2_byte2 = 18;
const char stopresult_ch2_byte1 = 19;
const char reference_index_ch3_byte3 = 20;
const char reference_index_ch3_byte2 = 21;
const char reference_index_ch3_byte1 = 22;
const char stopresult_ch3_byte3 = 23;
const char stopresult_ch3_byte2 = 24;
const char stopresult_ch3_byte1 = 25;
const char reference_index_ch4_byte3 = 26;
const char reference_index_ch4_byte2 = 27;
const char reference_index_ch4_byte1 = 28;
const char stopresult_ch4_byte3 = 29;
const char stopresult_ch4_byte2 = 30;
const char stopresult_ch4_byte1 = 31;


//----------------------------------------
//***Other Variables***
//----------------------------------------
//unsigned char Buffer = 0;
//char i = 0;
//int reference_index[4] = {0,0,0,0};
//int stopresult[4] = {0,0,0,0};
char config_error = false;

//----------------------------------------
//**GPIO Initialize***
//----------------------------------------
void tdc_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	 
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_12;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 
	GPIO_ResetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_12);	//初始化输出低		
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	spi_init();
}

void tdc_config()
{
	int i;
	unsigned char Buffer = 0;
	//---------------------------------------
	//***Power on reset***
	//---------------------------------------
	GPIO_SSN = 1;
	GPIO_SSN = 0;
	
	send_byte_to_SPI(spiopc_power);
	
	//----------------------------------------
	//***wrinting the configuration registers***
	//-----------------------------------------
	GPIO_SSN = 1;
	GPIO_SSN = 0;
	
	config_error = false;
	
	send_byte_to_SPI(spiopc_write_config+00);
	
	for(i=0;i<17;i++)
	{
		send_byte_to_SPI(config_register[i]);
	}
	
	//--------------------------------------------
	//***Verification of config register***
	//--------------------------------------------
	GPIO_SSN = 1;
	GPIO_SSN = 0;
	
	send_byte_to_SPI(spiopc_read_config+00);
	
	for(i=0;i<17;i++)
	{
		read_byte_from_SPI(&Buffer);
		if(config_register[i]!=Buffer)
			config_error = true;
		if(config_error)
			while(1);
	}
	
	//----------------------------------------------
	//***Initialize and start the measurement***
	//----------------------------------------------
	if(config_error == false)
	{
		GPIO_SSN = 1;
		GPIO_SSN = 0;
		
		send_byte_to_SPI(spiopc_init);
	}
}

/* 测量一次，获得一次测量结果 */
void tdc_measure(p_result presult)
{
	unsigned char Buffer = 0;
	int i;
	OS_CPU_SR cpu_sr=0;
	//Initialize to zero
	for(i=0;i<4;i++)
	{
		presult->reference_index[i] = 0;
		presult->stopresult[i] = 0;
		presult->reference_index[i] = 0;
		presult->stopresult[i] = 0;
	}
	
	while(GPIO_INTERRUPT!=0)
	{
		laser_plus();
		delay_ms(1);
	}
	
	GPIO_SSN = 1;
	GPIO_SSN = 0;
	
	OS_ENTER_CRITICAL();
	send_byte_to_SPI(spiopc_read_results+reference_index_ch1_byte3);
	
	for(i=0;i<4;i++)
	{
		read_byte_from_SPI(&Buffer);
		presult->reference_index[i] = presult->reference_index[i] + (Buffer<<16);
		
		read_byte_from_SPI(&Buffer);
		presult->reference_index[i] = presult->reference_index[i] + (Buffer<<8);
		
		read_byte_from_SPI(&Buffer);
		presult->reference_index[i] = presult->reference_index[i] + Buffer;
		
		read_byte_from_SPI(&Buffer);
		presult->stopresult[i] = presult->stopresult[i] + (Buffer<<16);
		
		read_byte_from_SPI(&Buffer);
		presult->stopresult[i] = presult->stopresult[i] + (Buffer<<8);
		
		read_byte_from_SPI(&Buffer);
		presult->stopresult[i] = presult->stopresult[i] + Buffer;
	}
	OS_EXIT_CRITICAL();
	
}

/* 一次脉冲发射读取所有的数据 */
/* presult 结果数组缓冲区首地址 */
/* 返回结果数量 */
INT8U			measureCylce;		//一次测距发射激光数
unsigned char tdc_measure_group(p_result presult)
{
	OS_CPU_SR cpu_sr=0;
	unsigned char Buffer = 0;
	int i;
	unsigned char res_cnt=0;
	int reference_index;
	int stopresult;
	//Initialize to zero

	//必须得保证通道1的第一个为FIFO记录的start信号
	
	OSSchedLock();
	tdc_config();	//清空TDC缓冲区				
	OSSchedUnlock();

	do
	{			
        OS_ENTER_CRITICAL();
		laser_plus();
        OS_EXIT_CRITICAL();
		delay_ms(10);
//		measureCylce++;
	}while(GPIO_INTERRUPT!=0);
	
    OSSchedLock();
	while(GPIO_INTERRUPT==0 && res_cnt<16)//TDC结果FIFO缓冲区中还有数据未读出
	{
		GPIO_SSN = 1;
		GPIO_SSN = 0;
		send_byte_to_SPI(spiopc_read_results+reference_index_ch1_byte3);
		
		for(i=0;i<4;i++)
		{
			reference_index = 0;
			stopresult = 0;
			
			read_byte_from_SPI(&Buffer);
			reference_index = reference_index+(Buffer<<16);
			//presult->reference_index[i] = presult->reference_index[i] + (Buffer<<16);
			
			read_byte_from_SPI(&Buffer);
			reference_index = reference_index+(Buffer<<8);
			//presult->reference_index[i] = presult->reference_index[i] + (Buffer<<8);
			
			read_byte_from_SPI(&Buffer);
			reference_index = reference_index+Buffer;
			//presult->reference_index[i] = presult->reference_index[i] + Buffer;
			
			read_byte_from_SPI(&Buffer);
			stopresult = stopresult+(Buffer<<16);
			//presult->stopresult[i] = presult->stopresult[i] + (Buffer<<16);
			
			read_byte_from_SPI(&Buffer);
			stopresult = stopresult+(Buffer<<8);
			//presult->stopresult[i] = presult->stopresult[i] + (Buffer<<8);
			
			read_byte_from_SPI(&Buffer);
			stopresult = stopresult+Buffer;
			//presult->stopresult[i] = presult->stopresult[i] + Buffer;
			
			presult[res_cnt].reference_index[i] = reference_index;
			presult[res_cnt].stopresult[i] = stopresult;
		}
		
		++res_cnt;
	}
    OSSchedUnlock();	
	return res_cnt;
}

