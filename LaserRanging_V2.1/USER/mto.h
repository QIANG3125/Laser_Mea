#ifndef __MTO_H
#define __MTO_H
#include "app.h"
#include "tpl0202.h"

/* MTO��� */
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

#define MAX_MTO_SIZE 32 //�����������ڵ���

/* ����ֵ�����ṹ�� MTO */
typedef struct MULTI_THRESHOLD_OPREATION
{
	/* ������ֵ */
	u8 th1;
	u8 th2;
	u8 th3;	
	u8 mode;//wall , line
	/* ��Ϻ��� */
	int (*adj)(TIME_DATA*,double*);//����ʱ�����ݣ����������
	struct MULTI_THRESHOLD_OPREATION* next;
}MTO;

extern MTO MTO_MEM[MAX_MTO_SIZE];

/* MTO���ƿ� MCB */
typedef struct MTO_CONTROL_BLOCK
{
	MTO* empty_mto_head;			//����MTO����ͷ
	MTO* use_mto_head_line;		//ʹ�õ�MTO����ͷ����
	MTO* use_mto_head_wall;		//ʹ�õ�MTO����ͷ��ǽ
	MTO* cur_mto_line;				//��ǰʹ�õ�MTO,��
	MTO* cur_mto_wall;				//��ǰʹ�õ�MTO��ǽ
	
	MTO* use_mto_rear_line;		//��ǰʹ�õ�MTOβ�ڵ�
	MTO* use_mto_rear_wall;
	u8 wall_circular_flag;         //�Ƿ�ѭ��
	u8 line_circular_flag;
	u8 measure_mode;					//����ģʽ
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


