#include "laser.h"
#include "delay.h"

unsigned int test_var = 0;

void laser_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 
		
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6;				 
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	 GPIO_Init(GPIOB, &GPIO_InitStructure);					 
	 GPIO_ResetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6);	//初始化输出低					 

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	    	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 
	GPIO_ResetBits(GPIOA,GPIO_Pin_15); 						//初始化输出低	 
	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	X5_2 = 0;
	X5_8 = 1;
	//X5_3 = 1;
}

void gpioc_to_io()
{

	GPIO_InitTypeDef GPIO_InitStructure;
	
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE );
    PWR_BackupAccessCmd( ENABLE );/* ????RTC??????*/
    RCC_LSEConfig( RCC_LSE_OFF ); /* ????????,PC14+PC15??????IO*/
    BKP_TamperPinCmd(DISABLE);  /* ????????,PC13??????IO*/

    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    PWR_BackupAccessCmd(DISABLE);/* ????RTC??????*/
		RCC_LSEConfig(RCC_LSE_OFF);
}



void nop()
{}

void laser_plus()
{	
	//static int i;
	X5_10 = 1;
	//for(i=0;i<1;i++){}
	nop();
	X5_10 = 0;
	//nop();
}

unsigned char laser_on()
{
	unsigned int i;
	
	while(1)
	{
		for(i=0;i<20;i++)
		{
			delay_ms(4);
			laser_plus();
		}
		delay_ms(400);
		test_var++;
	}
	
}

