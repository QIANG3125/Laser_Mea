#include "app.h"

//测距任务相关参数
double			d_distance = 0;
float			batch_err = 0;			//不同批次测墙补偿
float			caps	  = 0;
INT8U			level_stable = 0;
INT8U			no_data_cnt = 0;

//数据输出模式
unsigned char outmode= OutModeSet; //数据输出模式 camera:输出到照相机的格式  UARTbluetooth:打印到串口调试字符格式

/*********************************************************************************************/
//动态增益
/* 增益档位表 */

const INT16U	LEVEL[MAX_LEVEL] =
{
	1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200,2300
};

//////固定增益
///* 增益档位表 */
//const INT16U LEVEL[MAX_LEVEL] = 
//{
//	1800
//};
INT8U			level = 0; //档位

/*********************************************************************************************/
//UCOS II 任务相关变量定义
OS_STK			START_TASK_STAK[START_STAK_SIZE]; //开始任务堆栈
__align(8) OS_STK			MASTER_TASK_STAK[MASTER_STAK_SIZE];
OS_STK			GEN_TASK_STAK[GEN_STAK_SIZE];
__align(8) OS_STK			HANDLE_TASK_STAK[HANDLE_STAK_SIZE]; //使用printf打印浮点数到串口，需要任务堆栈8字节对齐
OS_STK			AGC_TASK_STAK[AGC_STAK_SIZE];
#ifdef ADJ
__align(8) OS_STK			ADJ_TASK_STAK[ADJ_STAK_SIZE];
#endif

/*********************************************************************************************/
//功能函数相关变量定义
void *			Qstart[QSTART_SIZE]; //消息队列指针数组
OS_EVENT *		q_msg; //消息队列

//INT_DATA ans_buf[ANS_BUF_SIZE];//输出结果缓冲区
TIME_DATA		ans_buf[ANS_BUF_SIZE]; //输出结果缓冲区

//INT32U err_buf[ANS_BUF_SIZE];//输出阈值时间差缓冲区
INT8U			ans_ite = 0; //当前写入位置
INT8U			ans_cnt = 0; //缓冲区中结果数量
INT8U 			first_measure = 0;		//是否是第一次测量

/* AGC 变量 */
unsigned char	AGC_EN;
unsigned short	quantify = QUANTIFY_INIT; //DAC 量化值
INT32U			detect_val = 0; //侦察阈值检测到的值 time_ps

const int		refclk_divisions = 125000; //8M晶振  ,TDC参考时钟


/* 设置数据输出格式 */
/* outmode为UARTbluetooth时输出字符串格式 */
/* outmode为camera时输出字节流格式 */
/* MODE为0时测墙模式 */
/* MODE为1时测线模式 */
void out_mode_set()
{ /* 输出模式设定 */

	if(outmode==UARTbluetooth)		//蓝牙输出调试
		{
		//		硬件判断测线与测墙模式		加跳帽测墙，不加测线
			if (MODE == 1)
				{
				mcb.measure_mode=wall;
				}
			else 
				{
				mcb.measure_mode=line;
				}
		}
	else 	//上位机输出数据
	{
//		//上位机选择测线测墙模式
//		while(Mode_Judge());		上位机协议未写好，只能硬件选择2021.1.25

		//		硬件判断测线与测墙模式
			if (MODE == 1)
				{
				mcb.measure_mode=wall;
				}
			else 
				{
				mcb.measure_mode=line;
				}
	}
} /* CH_DATA转double */

unsigned char Mode_Judge()
{
	INT16U pos=0;

	if(USART_RX_STA>2)	//协议指令长度为3,指令接受完整才处理
	{
		if(USART_RX_BUF[pos]==0xCD)	// 接受到帧头
			{
				switch (USART_RX_BUF[pos+1])
				{
					case 0X4F:
						if(USART_RX_BUF[pos+2] == 0X11)
							mcb.measure_mode=line;
						else if(USART_RX_BUF[pos+2] == 0x33)
							mcb.measure_mode=wall;
						break;
				//协议可扩展
						default:break;
				}
				//接受到指令
				return 0;
			}
		else
			{
				pos++;
			}
	}

    //未接受到指令
    return 1;
	
}


double ch_data_dou(CH_DATA * pch_data)
{
	double			ans;

	INT16U			i_temp = 0;

	ans 				= (double)
	pch_data->f_data / 256;
	i_temp				= ((INT16U) (pch_data->h_data) << 8) +pch_data->l_data;
	return ans + (double)
	i_temp;
} /* double转CH_DATA */

 
CH_DATA dou_ch_data(double * d_data, INT32U err_time)
{
	CH_DATA 		ch_data;

	INT32U			i_temp;

	i_temp				= *d_data;					//获取整数位
	ch_data.h_data		= (i_temp >> 8) & 0xFF;
	ch_data.l_data		= i_temp & 0xFF;
	ch_data.f_data		= (INT8U) ((*d_data - i_temp) * 256);
	ch_data.err_time	= err_time;
	return ch_data;
} /* CH_DATA转u32 */


INT32U ch_data_u32(CH_DATA * pch_data)
{
	INT32U			ans = 0;

	ans 				+= pch_data->h_data;
	ans 				= ans << 8;
	ans 				+= pch_data->l_data;
	ans 				= ans << 8;
	ans 				+= pch_data->f_data;
	return ans;
} /* u32转double */


double u32_dou(INT32U data)
{
	double			ans;

	ans 				= (double) (data & 0xFF) / 256; //取出小数
	ans 				= ans + (data >> 8);		//加上整数位
	return ans;
} /* double转u32 */


INT32U dou_u32(double * d_data)
{
	INT32U			ans;

	CH_DATA 		temp;

	temp				= dou_ch_data(d_data, 0);
	ans 				= ch_data_u32(&temp);
	return ans;



} /* 将CH_DATA格式的数据打印成浮点数显示 */
/* 显示小数点后两位 */


void print_ch_data(CH_DATA * ch_data)
{
	printf("%d.%02d ", 
		((INT16U) ch_data->h_data << 8) +ch_data->l_data, ch_data->f_data * 100 / 256);




} /* 将U32格式的数据打印成浮点数显示(3个字节合并) */
/* 显示小数点后两位 */


void print_int_data(INT32U int_data)
{
	double			res;

	res 				= u32_dou(int_data);

	//printf("%d.%.2f ",int_data>>8,((double)(int_data&0xFF))/256);
	printf("%.2f ", res);
} /* 快速排序算法 */


int partiton_int_data(INT_DATA * arr, int low, int high)
{
	int 			pivotkey;

	INT_DATA		d_temp;

	pivotkey			= arr[low].i_data;

	while (low < high)
		{
		while (low < high && arr[high].i_data >= pivotkey)
			high--;

		d_temp				= arr[low];
		arr[low]			= arr[high];
		arr[high]			= d_temp;				//swap(arr,low,high);

		while (low < high && arr[low].i_data <= pivotkey)
			low++;

		d_temp				= arr[low];
		arr[low]			= arr[high];
		arr[high]			= d_temp;				//swap(arr,low,high);
		}

	return low;
}


void q_sort_int_data(INT_DATA * arr, int low, int high)
{
	int 			pivot;

	if (low < high)
		{
		pivot				= partiton_int_data(arr, low, high);
		q_sort_int_data(arr, low, pivot - 1);
		q_sort_int_data(arr, pivot + 1, high);
		}
}


void quick_sort_int_data(INT_DATA * arr, INT32U size)
{
	q_sort_int_data(arr, 0, size - 1);
} /* 快速排序算法 */


int partiton_time_data(TIME_DATA * arr, int low, int high)
{
	int 			pivotkey;

	TIME_DATA		d_temp;

	pivotkey			= arr[low].time_ps;

	while (low < high)
		{
		while (low < high && arr[high].time_ps >= pivotkey)
			high--;

		d_temp				= arr[low];
		arr[low]			= arr[high];
		arr[high]			= d_temp;				//swap(arr,low,high);

		while (low < high && arr[low].time_ps <= pivotkey)
			low++;

		d_temp				= arr[low];
		arr[low]			= arr[high];
		arr[high]			= d_temp;				//swap(arr,low,high);
		}

	return low;
}


void q_sort_time_data(TIME_DATA * arr, int low, int high)
{
	int 			pivot;

	if (low < high)
		{
		pivot				= partiton_time_data(arr, low, high);
		q_sort_time_data(arr, low, pivot - 1);
		q_sort_time_data(arr, pivot + 1, high);
		}
}


void quick_sort_time_data(TIME_DATA * arr, INT32U size)
{
	q_sort_time_data(arr, 0, size - 1);
} 

double getvar_orgdata(TIME_DATA* array,INT8U length)
{
	double result=0;
	double distance[DATA_GROUP_SIZE]={0};
	INT8U i=0;
	INT32S sum=0,average=0;
	
	for(i=0;i<length;i++)
		{
			distance[i]=(1.5 * (double) array[i].time_ps) / 10000;
			sum+=distance[i];
		}
	average= sum/length;
	
	for(i=0;i<length;i++)
	{
		result+= pow(distance[i]-average,2)/length;
	}
	
	return result;
}


/*********对数函数Ln(x)**********/
double my_log(double a)
{
	int 			N	= 100000;

	long			k, nk;

	double			x, xx, y;

	if (a < 0)
		{
		a					= -a;
		}

	x					= (a - 1) / (a + 1);
	xx					= x * x;
	nk					= 2 * N + 1;
	y					= 1.0 / nk;

	for (k = N; k > 0; k--)
		{
		nk					= nk - 2;
		y					= 1.0 / nk + xx * y;
		}

	return 2.0 * x * y;
} 
/*******************************/
/*********AGC 控制函数**********/
/*********基于峰值电压**********/

void agc_control(OS_CPU_SR cpu_sr)
{
	static double	ADC_Value;						//ADC value

	static double	err;							//增益控制差值

	/* Get ADC_Value */
	ADC_Value			= (double)ADC_ConvertedValue / 4096 * 3.3;

	/* AGC Control Code */

	/* 增量控制 */
	if (ADC_Value < CONTROL_MAX_VOLTAGE)
		{
		err 				= CONTROL_MAX_VOLTAGE - ADC_Value;
		}
	else if (ADC_Value >= CONTROL_MAX_VOLTAGE)
		{
		err 				= ADC_Value - CONTROL_MAX_VOLTAGE;
		}

	if (ADC_Value < CONTROL_MAX_VOLTAGE - FLOW_RANGE)
		{
		if (quantify > 2000)
			quantify = 2000;
		else if (err > 1.0)
			{
			quantify			+= 10;
			}
		else if (err > 0.8)
			{
			quantify			+= 8;
			}
		else if (err > 0.6)
			{
			quantify			+= 4;
			}
		else if (err > 0.4)
			{
			quantify			+= 2;
			}
		else 
			{
			quantify			+= 1;
			}

		OS_ENTER_CRITICAL();
		setDacValueBin(quantify);
		OS_EXIT_CRITICAL();
		}

#ifdef AGC_LOW
	else if (ADC_Value > CONTROL_MAX_VOLTAGE + FLOW_RANGE)
		{
		if (quantify <= 10)
			quantify = 10;
		else if (err > 1.0)
			quantify -= 10;
		else if (err > 0.8)
			quantify -= 8;
		else if (err > 0.6)
			quantify -= 4;
		else if (err > 0.4)
			quantify -= 2;
		else 
			quantify -= 1;

		OS_ENTER_CRITICAL();
		setDacValueBin(quantify);
		OS_EXIT_CRITICAL();
		}

#endif
} /* 构建TIME_DATA */


void create_time_data(TIME_DATA * ptime_data, INT32U time_ps, INT32U err_time1, INT32U err_time2, 
	INT32U err_time3)
{
	ptime_data->time_ps = time_ps;

	ptime_data->err_time1 = err_time1;
	ptime_data->err_time2 = err_time2;
	ptime_data->err_time3 = err_time3;
} /* 多阈值数据拟合 */


double multi_thread_adj(double * org, unsigned int * time)
{
	double			ans;

	if (*time > 1100 && *time < 2000)
		{
		ans 				= *org - 0.8541 * my_log(-10.9372 * (*time) + 11430) + 3.019 - coe;
		}
	else if ((*time) >= 2000)
		{
		ans 				= *org - 0.0004 * (*time) - 4.0737 - coe;
		}
	else 
		{
		ans 				= *org - coe1;
		}

	return ans;
}



/* 数据分类筛选 */
/* arr: 有序数组 */
/* len: 数组长度 */
/* diff_time: 分类范围ps */
INT8U data_filter(TIME_DATA* arr, unsigned char len, unsigned int diff_time, TIME_DATA *ans)
{
	unsigned char cur_cnt;
	unsigned char max_ele=0,max_pos = 0;
	int i = 0;
	unsigned int cur_category;

	/* 进行数据分类 */
	cur_category = arr[0].time_ps;
	cur_cnt = 1;
	for (i = 1; i < len; i++)
	{
		if (arr[i].time_ps - cur_category < diff_time)//属于当前分类
		{
			cur_cnt++;//分类值计数加一
		}
		else//新的分类
		{
			if (cur_cnt > max_ele)//更新最大计数值
			{
				max_ele = cur_cnt;//最大计数
				max_pos = i - max_ele;//最大分类值在原始数组中的位置
				
			}				
			/* 更新分类数组 */
			cur_category = arr[i].time_ps;
			cur_cnt = 1;
		}
	}
	//处理边界情况
	if(cur_cnt > max_ele)
	{
		max_ele = cur_cnt;//最大计数
		max_pos = i - max_ele;//最大分类值在原始数组中的位置

	}
	// 取最大计数分类数据中的中值
	if(max_ele<3)
		return NON_ERR;
	else
		*ans= arr[((max_pos << 1) + max_ele) / 2];
	return SUN_OK;
}


INT8U data_2_filter(GEN_TIME_DATA (*timedata)[2],INT8U maxnum,TIME_DATA *ans)
{
	float sum_valid1=0,sum_valid2=0;
	float var1=0,var2=0;
	INT8U i;
	
	for(i=0;i<maxnum;i++)
		{
			sum_valid1 += timedata[i][0].distance;
			sum_valid2 += timedata[i][1].distance;
		}
	sum_valid1=sum_valid1/maxnum;
	sum_valid2=sum_valid2/maxnum;

	for(i=0; i<maxnum; i++)
		{
			var1+= pow(timedata[i][0].distance-sum_valid1,2);
			var2+= pow(timedata[i][1].distance-sum_valid2,2);
		}
	var1= var1/maxnum;
	var2= var1/maxnum;

	if(var1<10 && var2<10)	//两个有效数据方差都稳定在一定范围，说明测量有效
		{
			if(timedata[1][1].distance <200) //第一个数据有效
				{
					if(quantify> 1100)
						{
							return Q_OUT;
						}
					else
						{
							*ans= timedata[1][0].timedata;
							return VALID_MEAS;
						}
				}
			else	//第二个数据有效
				{
					*ans= timedata[1][1].timedata;
				
					return VALID_MEAS;
				}
		}
	else
		return DATA_ERR;
}
