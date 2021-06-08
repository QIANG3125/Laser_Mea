#ifndef app_h
#define app_h


#include "os_cpu.h"
#include "includes.h"
#include "tlv5636.h"
#include "usart.h"
#include "key.h"
#include "agc.h"
#include <math.h>


/*********************************************************************************************/
/* 激光测距任务校准参数 */
#define coe						0.2
#define coe1					2.4
#define coe2					2.2
extern double			d_distance;
extern float			batch_err;			//不同批次测墙补偿
extern float			caps;				//近距离测线补偿
extern INT8U			level_stable;
extern INT8U			no_data_cnt;

//激光测距模式类型
enum 
{
	wall,											//测墙模式
	line											//测线模式
};

//激光测距输出模式类型11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111 
enum
{
	camera,
	UARTbluetooth
};
#define OutModeSet camera		//输出模式设置
extern unsigned char outmode;

/*********************************************************************************************/
////动态增益
#define MAX_LEVEL				13
/* 增益档位表 */
extern const INT16U	LEVEL[MAX_LEVEL];

////固定增益
//#define MAX_LEVEL 1
/////* 增益档位表 */
//extern const INT16U LEVEL[MAX_LEVEL];
extern INT8U			level;


//#define DEBUG
//#define DUG		 //发送原始数据与滤波数据 

/* 校准调试模式参数设置 */
#define STANDARD_DISTANCE		0 //测试目标标准距离
#define INC_STEP				10		   //增益增加步进
#define MIN_GAIN				1000 	  //最小增益 quantify DAC量化值
#define MAX_GAIN				1500	   //最大增益

/**********************************************/
/* 宏定义模式设置 */
/* 通过注释或定义宏选择模式 */
//#define ADJ //校准调试模式 ,定义它将会启动校准模式
#define MULTI_VTH //定义它将会把结果进行多阈值拟合校准

//#define AGC_LOW	//如果定义AGC_LOW,AGC控制会将高于CONTROL_MAX_VOLTAGE的时候减少增益，否则只会增大增益
#define AUTO_LEVEL_MOD //自动档位切换模式 不定义则为AGC模式

//#define ARGV	//是否将缓冲区结果求平均，不然取中值

/*********************************************************************************************/
/****************结构体定义**********************/
/* 将浮点数转换为3个字节表示 */
/* 阈值时间差 */
typedef struct CHANGE_3CHAR_DATA
{
	/* h_data : 整数高字节 */
	/* l_data ：整数低字节 */
	/* f_data : 小数位 */
	INT8U			h_data;
	INT8U			l_data;
	INT8U			f_data; 						//小数位为 小数*256表示 例如0.01*256 = 2 
	INT32U			err_time;
} CH_DATA;

/* INT32U表示的数据 */
/* 阈值时间差 */
typedef struct CHANGE_INT_DATA
{
	INT32U			i_data; 						//距离 低8位小数，其余表示整数
	INT32U			err_time;
} INT_DATA;


/* 飞行时间和阈值时间差 */
typedef struct TIME_PS_DATA
{
	INT32U			time_ps;
	INT32U			err_time1;
	INT32U			err_time2;
	INT32U			err_time3;
} TIME_DATA;

typedef struct GEN_TIME_DATA
{
	TIME_DATA		timedata;
	float			distance;
}GEN_TIME_DATA;

/*********************************************************************************************/
//UCOS 任务设置
//设置开始任务优先级
#define START_TASK_PRIO 		10//一共有10个优先级，开始任务的优先级最低为10
#define START_STAK_SIZE 		64//设置开始任务堆栈大小
extern OS_STK			START_TASK_STAK[START_STAK_SIZE]; //开始任务堆栈

//设置主控线程优先级最高
#define MASTER_TASK_PRIO		5
#define MASTER_STAK_SIZE		512
extern OS_STK			MASTER_TASK_STAK[MASTER_STAK_SIZE];

//设置激光测距生产者任务
#define GEN_TASK_PRIO			6
#define GEN_STAK_SIZE			512*5
#define MEASURE_DATA_ARR_SIZE	16
extern OS_STK			GEN_TASK_STAK[GEN_STAK_SIZE];

//设置数据处理消费者任务
#define HANDLE_TASK_PRIO		7
#define HANDLE_STAK_SIZE		128
extern OS_STK			HANDLE_TASK_STAK[HANDLE_STAK_SIZE]; //使用printf打印浮点数到串口，需要任务堆栈8字节对齐

//设置AGC控制任务
#define AGC_TASK_PRIO			8
#define AGC_STAK_SIZE			64
extern OS_STK			AGC_TASK_STAK[AGC_STAK_SIZE];

#ifdef ADJ

//设置调试校准任务 
#define ADJ_TASK_PRIO			4
#define ADJ_STAK_SIZE			64
extern OS_STK			ADJ_TASK_STAK[ADJ_STAK_SIZE];

#endif

void start_task(void * pdata);
void master_task(void * pdata);
void gen_task(void * pdata);
void handle_task(void * pdata);
void agc_task(void * pdata);
void adj_task(void * pdata);

/*********************************************************************************************/
//功能函数相关设置
#define DATA_GROUP_SIZE 		5 //每一组接收数据长度
#define QSTART_SIZE 			256    //消息队列指针数组缓存区大小
extern void *			Qstart[QSTART_SIZE]; //消息队列指针数组
extern OS_EVENT *		q_msg; //消息队列

#define ANS_BUF_SIZE			8	 //输出结果缓冲区大小

//INT_DATA ans_buf[ANS_BUF_SIZE];//输出结果缓冲区
extern TIME_DATA		ans_buf[ANS_BUF_SIZE]; //输出结果缓冲区

//INT32U err_buf[ANS_BUF_SIZE];//输出阈值时间差缓冲区
extern INT8U			ans_ite; //当前写入位置
extern INT8U			ans_cnt; //缓冲区中结果数量
extern INT8U 			first_measure;		//是否是第一次测量


/* 位操作宏 */
#define setbit(x, y)			(x)|=(1<<(y))
#define clrbit(x, y)			(x)&=~(1<<(y))
#define reversebit(x, y)		(x)^=(1<<(y))
#define getbit(x, y)			((x)>>(y)&1)

/* 初始化多阈值参数 */
#define QUANTIFY_INIT			1100  //初始化增益值 DAC
#define START_THREAD			80			
#define STOP_THREAD1			60		//原60
#define STOP_THREAD2			210			//判断是否饱和
#define STOP_THREAD3			80		//原80

/* 数据筛选校准参数 */
#define GAIN_TH 				1100	  //增益限值
#define DISTANCE_TH 			21	  //距离限制
#define TIME_PS_TH				100000 //15米的时间 
#define TIME_PS_TH2				500000 //75米时间
#define MAX_TIME				10000000 //最大飞行时间 1500米的时间 ps
#define TIME_1000M				6666667		//1000米飞行时间
#define MIN_TIME				0		 //最小飞行时间  
#define DETECT_MIN_TIME 		300000 //探测阈值最小飞行时间 45m
#define MAX_ERR_TIME			10000 //最大阈值时间差
#define MIN_ERR_TIME			0	 //最小阈值时间差

/* AGC 变量 */
extern unsigned char	AGC_EN;
extern unsigned short	quantify; //DAC 量化值
extern INT32U			detect_val; //侦察阈值检测到的值 time_ps

/* AGC Control parameter	 */
#define CONTROL_MAX_VOLTAGE 	3
#define FLOW_RANGE				0.2
#define CONTROL_DIFF_TIME		2000
#define TIME_FLOW_RANGE 		200

#define MAX_NO_DATA_CNT 		5

extern const int		refclk_divisions; //8M晶振  ,TDC参考时钟


void out_mode_set(void);
unsigned char Mode_Judge(void);
double ch_data_dou(CH_DATA * pch_data);
CH_DATA dou_ch_data(double * d_data, INT32U err_time);
INT32U ch_data_u32(CH_DATA * pch_data);
double u32_dou(INT32U data);
INT32U dou_u32(double * d_data);
void quick_sort_int_data(INT_DATA * arr, INT32U size); //快速排序
void quick_sort_time_data(TIME_DATA * arr, INT32U size);
void print_ch_data(CH_DATA * ch_data);
void print_int_data(INT32U int_data);
double getvar_orgdata(TIME_DATA* array,INT8U length);
double my_log(double a);
void agc_control(OS_CPU_SR cpu_sr);
void create_time_data(TIME_DATA * ptime_data, INT32U time_ps, INT32U err_time1, INT32U err_time2, INT32U err_time3);
double multi_thread_adj(double * org, unsigned int * time);
INT8U data_filter(TIME_DATA* arr, unsigned char len, unsigned int diff_time, TIME_DATA *ans);
INT8U data_2_filter(GEN_TIME_DATA (*timedata)[2],INT8U maxnum,TIME_DATA *ans);

#include "mto.h"
#endif

