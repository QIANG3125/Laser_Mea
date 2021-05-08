#ifndef __MTO_H
#define __MTO_H
#include "app.h"
#include "tpl0202.h"

/* MTO相关 */
/*******************************/
enum
{
	NON_ERR,
	NULL_PTR,
	REPEATED_ACCESS,
	NO_EMPTY_MTO,
	NOT_FOUND,
	UNDEFINE_MODE,
	LINE_OUT,
	WALL_OUT,
	Q_OUT,
	DATA_ERR,
	VALID_MEAS,
	SUN_OK
};


#define SINGLE_LIST 0
#define CIRCULAR_LIST 1

#define MAX_MTO_SIZE 32 //空闲链表最大节点数

/* 多阈值操作结构体 MTO */
typedef struct MULTI_THRESHOLD_OPREATION
{
	/* 三个阈值 */
	u8 th1;
	u8 th2;
	u8 th3;	
	u8 mode;//wall , line
	/* 拟合函数 */
	int (*adj)(TIME_DATA*,double*);//飞行时间数据，结果缓存区
	struct MULTI_THRESHOLD_OPREATION* next;
}MTO;

extern MTO MTO_MEM[MAX_MTO_SIZE];

/* MTO控制块 MCB */
typedef struct MTO_CONTROL_BLOCK
{
	MTO* empty_mto_head;			//空闲MTO链表头
	MTO* use_mto_head_line;		//使用的MTO链表头，线
	MTO* use_mto_head_wall;		//使用的MTO链表头，墙
	MTO* cur_mto_line;				//当前使用的MTO,线
	MTO* cur_mto_wall;				//当前使用的MTO，墙
	
	MTO* use_mto_rear_line;		//当前使用的MTO尾节点
	MTO* use_mto_rear_wall;
	u8 wall_circular_flag;         //是否循环
	u8 line_circular_flag;
	u8 measure_mode;					//测量模式
}MCB;

extern MCB mcb;

int mto_init(MCB*);
int mto_add(u8 th1,u8 th2,u8 th3,u8 mode,int (*adj)(TIME_DATA*,double*));
int mto_erase(u8 th1,u8 th2,u8 th3,u8 mode);
int link_to_circular_list(MTO* mto_list);
int unlink_to_circular_list(MTO* mto_list);
void print_list(MTO* mto_list);
int mto_start(u8 wall_list_mode,u8 line_list_mode);
int mto_switch(void);
int mto_adj(TIME_DATA* time_data,double* ansbuf);
MTO* mto_find(u8 th1,u8 th2,u8 th3,u8 mode);
int mto_switch_p(u8 th1,u8 th2,u8 th3,u8 mode);
int set_measure_mode(u8 _measure_mode);
u8 get_measure_mode(void);
MTO* get_cur_mto(void);

int adj_wall_60_70_80(TIME_DATA *data, double *ans);
int adj_line_60_70_80(TIME_DATA *data,double *ans);
int adj_line_80_90_100(TIME_DATA *data,double *ans);
int adj_wall_80_90_100(TIME_DATA *data,double *ans);




#endif


