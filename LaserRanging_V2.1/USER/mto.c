#include "mto.h"

/************MTO相关***************/

/* MTO初始化存储块 */
MTO MTO_MEM[MAX_MTO_SIZE] = {0};
MCB mcb = {0};

/* 初始化MTO */
int mto_init(MCB* p_mcb)
{
	static u8 access = 0;
	u8 i;
	MTO* p_mto;
	if(p_mcb==NULL) return NULL_PTR;
	if(access==1)//单例，防止多次调用
	{
		return REPEATED_ACCESS;
	}
	access = 1;
	/* 初始化空闲链表 */
	p_mcb->empty_mto_head = &MTO_MEM[0];
	p_mto = p_mcb->empty_mto_head;
	for(i=1;i<MAX_MTO_SIZE;i++)
	{
		p_mto->next = &MTO_MEM[i];
		p_mto = &MTO_MEM[i];
	}
	return NON_ERR;
}

/* 添加MTO */
int mto_add(u8 th1,u8 th2,u8 th3,u8 mode,int (*adj)(TIME_DATA*,double*))
{
	if(mcb.empty_mto_head==NULL) return NO_EMPTY_MTO;
	/* 从空闲链表分配资源 */
	mcb.empty_mto_head->th1 	= th1;
	mcb.empty_mto_head->th2 	= th2;
	mcb.empty_mto_head->th3 	= th3;
	mcb.empty_mto_head->mode 	= mode;
	mcb.empty_mto_head->adj 	= adj;
	
	if(mode==wall)//分配到测墙拟合的链表
	{
		if(mcb.use_mto_head_wall==NULL)//当前链表为空
		{
			mcb.use_mto_rear_wall = mcb.empty_mto_head;
			mcb.use_mto_head_wall = mcb.empty_mto_head;
			mcb.cur_mto_wall 			= mcb.empty_mto_head;
		}
		else//不为空时，在当前链表下连接
		{
			mcb.use_mto_rear_wall->next = mcb.empty_mto_head;
			mcb.use_mto_rear_wall 			= mcb.empty_mto_head;
		}
		mcb.empty_mto_head = mcb.empty_mto_head->next;
		mcb.use_mto_rear_wall->next = NULL;
	}
	
	if(mode==line)//分配到测线拟合的链表
	{
		if(mcb.use_mto_head_line==NULL)//当前链表为空
		{
			mcb.use_mto_rear_line = mcb.empty_mto_head;
			mcb.use_mto_head_line = mcb.empty_mto_head;
			mcb.cur_mto_line 			= mcb.empty_mto_head;
		}
		else//不为空时，在当前链表下连接
		{
			mcb.use_mto_rear_line->next = mcb.empty_mto_head;
			mcb.use_mto_rear_line 			= mcb.empty_mto_head;
		}
		mcb.empty_mto_head = mcb.empty_mto_head->next;
		mcb.use_mto_rear_line->next = NULL;
	}	
	/* 链接成循环链表 */
	if(mode==wall&&mcb.wall_circular_flag)
		link_to_circular_list(mcb.use_mto_head_wall);
	if(mode==line&&mcb.line_circular_flag)
		link_to_circular_list(mcb.use_mto_head_line);
	return NON_ERR;
}


/* 删除链表中的节点 */
int mto_erase(u8 th1,u8 th2,u8 th3,u8 mode)
{
	MTO *mto_list,*pre_node,*p_mto;
	p_mto = mto_find(th1,th2,th3,mode);//找到对应节点地址
	if(p_mto==NULL) return NOT_FOUND;
	mto_list = mode==wall ? mcb.use_mto_head_wall:mcb.use_mto_head_line;
	/* 先展开成单链表操作 */
	if(mode==wall)
	{
		unlink_to_circular_list(mcb.use_mto_head_wall);
		if(p_mto==mcb.cur_mto_wall)//删除的节点是当前正在使用的节点
		{
			mcb.cur_mto_wall = mcb.cur_mto_wall->next;//切换下一个
		}
	}
		
	if(mode==line)
	{
		unlink_to_circular_list(mcb.use_mto_head_line);
		if(p_mto==mcb.cur_mto_line)//删除的节点是当前正在使用的节点
		{
			mcb.cur_mto_line = mcb.cur_mto_line->next;//切换下一个
		}
	}
		
	
	/* 删除的头节点情况 */
	if(mto_list==p_mto)
	{
		if(mode==wall)
		{
			mcb.use_mto_head_wall = mto_list->next;
			if(mcb.use_mto_head_wall==NULL)//当头节点删除后链表空时，还要更新尾节点
			{
				mcb.use_mto_rear_wall = NULL;
				mcb.cur_mto_wall = NULL;
			}				
		}			
		if(mode==line)
		{
			mcb.use_mto_head_line = mto_list->next;
			if(mcb.use_mto_head_line==NULL)//当头节点删除后链表空时，还要更新尾节点
			{
				mcb.use_mto_rear_line = NULL;
				mcb.cur_mto_line = NULL;
			}				
		}

		p_mto->th1 = 0;
		p_mto->th2 = 0;
		p_mto->th3 = 0;
		p_mto->mode = 0;
		p_mto->adj = NULL;
		p_mto->next = mcb.empty_mto_head;//释放资源回归空闲链表
		mcb.empty_mto_head = p_mto;
		/* 如果是循环链表再链接回去 */
		if(mode==wall&&mcb.wall_circular_flag)
			link_to_circular_list(mcb.use_mto_head_wall);
		if(mode==line&&mcb.line_circular_flag)
			link_to_circular_list(mcb.use_mto_head_line);
		return NON_ERR;
	}
	pre_node = mto_list;
	mto_list = mto_list->next;
	while(mto_list!=NULL)
	{
		if(mto_list==p_mto)
		{
			/* 更新尾节点 */
			if(mode==wall)
			{
				if(p_mto==mcb.use_mto_rear_wall)
					mcb.use_mto_rear_wall = pre_node;
			}
			else if(mode==line)
			{
				if(p_mto==mcb.use_mto_rear_line)
					mcb.use_mto_rear_line = pre_node;
			}
			
			pre_node->next = p_mto->next;//删除节点
			p_mto->next = mcb.empty_mto_head;//释放资源回归空闲链表
			mcb.empty_mto_head = p_mto;
			p_mto->th1 = 0;
			p_mto->th2 = 0;
			p_mto->th3 = 0;
			p_mto->mode = 0;
			p_mto->adj = NULL;
			/* 如果是循环链表再链接回去 */
			if(mode==wall&&mcb.wall_circular_flag)
				link_to_circular_list(mcb.use_mto_head_wall);
			if(mode==line&&mcb.line_circular_flag)
				link_to_circular_list(mcb.use_mto_head_line);
			return NON_ERR;
		}
		
		pre_node = mto_list;
		mto_list = mto_list->next;
	}
	/* 如果是循环链表再链接回去 */
	if(mode==wall&&mcb.wall_circular_flag)
		link_to_circular_list(mcb.use_mto_head_wall);
	if(mode==line&&mcb.line_circular_flag)
		link_to_circular_list(mcb.use_mto_head_line);
	return NOT_FOUND;
}

/* 将MTO链表链接成循环链表 */
int link_to_circular_list(MTO* mto_list)
{
	u8 mode;
	if(mto_list==NULL)
		return NULL_PTR;
	mode = mto_list->mode;
	if(mode==wall)
	{
		mcb.use_mto_rear_wall->next = mcb.use_mto_head_wall;
	}
	if(mode==line)
	{
		mcb.use_mto_rear_line->next = mcb.use_mto_head_line;
	}
	return NON_ERR;
}

/* 奖MTO循环链表断开成单链表 */
int unlink_to_circular_list(MTO* mto_list)
{
	u8 mode;
	if(mto_list==NULL)
		return NULL_PTR;
	mode = mto_list->mode;
	if(mode==wall)
	{
		mcb.use_mto_rear_wall->next = NULL;
	}
	if(mode==line)
	{
		mcb.use_mto_rear_line->next = NULL;
	}
	return NON_ERR;
}

/* 调试打印链表 */
void print_list(MTO* mto_list)
{
	while(mto_list!=NULL)
	{
		/* 打印调试 */
		printf("%d %d %d %d\n",mto_list->th1,mto_list->th2,mto_list->th3,mto_list->mode);
		mto_list = mto_list->next;
	}
}

/* 启动mto */
/* 硬件设置具体的电压值 */
/* 参数：是否连接成循环链表 */
/* wall_list_mode ： SINGLE_LIST or CIRCULAR_LIST */
/* line_list_mode ： SINGLE_LIST or CIRCULAR_LIST */
int mto_start(u8 wall_list_mode,u8 line_list_mode)
{
	u8 t1,t2,t3;
	/* 是否连接成循环链表 */
	if(wall_list_mode==SINGLE_LIST)
	{
		mcb.wall_circular_flag = 0;
		unlink_to_circular_list(mcb.use_mto_head_wall);
	}		
	if(wall_list_mode==CIRCULAR_LIST)
	{
		mcb.wall_circular_flag = 1;
		link_to_circular_list(mcb.use_mto_head_wall);
	}
	if(line_list_mode==SINGLE_LIST)
	{
		mcb.line_circular_flag = 0;
		unlink_to_circular_list(mcb.use_mto_head_line);
	}		
	if(line_list_mode==CIRCULAR_LIST)
	{
		mcb.line_circular_flag = 1;
		link_to_circular_list(mcb.use_mto_head_line);
	}

	// 对数控10202 设置电压阈值
	if(mcb.measure_mode==wall)
	{
		t1 = mcb.cur_mto_wall->th1;
		t2 = mcb.cur_mto_wall->th2;
		t3 = mcb.cur_mto_wall->th3;
	}
	if(mcb.measure_mode==line)
	{
		t1 = mcb.cur_mto_line->th1;
		t2 = mcb.cur_mto_line->th2;
		t3 = mcb.cur_mto_line->th3;
	}
	
	write_wb(START_THREAD, &TPL1_CS, &TPL1_CLK, &TPL1_D); //set voltage thread of start pulse
	write_wa(t1 ,&TPL1_CS, &TPL1_CLK, &TPL1_D); //set voltage thread of stop1 pulse
	write_wa(t2, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop2 pulse
	write_wb(t3, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop3 pulse
	return NON_ERR;
}

/* 动态阈值切换 */
int mto_switch()
{
	u8 t1,t2,t3;
	if(mcb.measure_mode==wall)
	{
		if(mcb.cur_mto_wall==NULL)
			return NULL_PTR;
		mcb.cur_mto_wall = mcb.cur_mto_wall->next;
		t1 = mcb.cur_mto_wall->th1;
		t2 = mcb.cur_mto_wall->th2;
		t3 = mcb.cur_mto_wall->th3;
		if(mcb.cur_mto_wall==NULL)
			return NULL_PTR;
	}
	if(mcb.measure_mode==line)
	{
		if(mcb.cur_mto_line==NULL)
			return NULL_PTR;
		mcb.cur_mto_line = mcb.cur_mto_line->next;
		t1 = mcb.cur_mto_line->th1;
		t2 = mcb.cur_mto_line->th2;
		t3 = mcb.cur_mto_line->th3;
		if(mcb.cur_mto_line==NULL)
			return NULL_PTR;
	}
	write_wa(t1 ,&TPL1_CS, &TPL1_CLK, &TPL1_D); //set voltage thread of stop1 pulse
	write_wa(t2, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop2 pulse
	write_wb(t3, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop3 pulse
	return NON_ERR;
}

/* 多阈值拟合校准 */
int mto_adj(TIME_DATA* time_data,double* ansbuf)
{
	if(mcb.measure_mode==wall)
		return mcb.cur_mto_wall->adj(time_data,ansbuf);
	if(mcb.measure_mode==line)
		return mcb.cur_mto_line->adj(time_data,ansbuf);
	return UNDEFINE_MODE;
}

/* 测试回调函数 */
int adj_test(TIME_DATA* time_data,double* ansbuf)
{
	return 0;
}

/* 找到指定的mto块 */
MTO* mto_find(u8 th1,u8 th2,u8 th3,u8 mode)
{
	MTO* ret = NULL;
	
	if(mode==wall)
	{
		unlink_to_circular_list(mcb.use_mto_head_wall);
		ret = mcb.use_mto_head_wall;
		while(ret!=NULL)
		{
			if(ret->th1==th1&&ret->th2==th2&&ret->th3==th3&&ret->mode==mode)
				break;
			ret = ret->next;
		}
		if(mcb.wall_circular_flag)
			link_to_circular_list(mcb.use_mto_head_wall);
	}
	if(mode==line)
	{
		unlink_to_circular_list(mcb.use_mto_head_line);
		ret = mcb.use_mto_head_line;
		while(ret!=NULL)
		{
			if(ret->th1==th1&&ret->th2==th2&&ret->th3==th3&&ret->mode==mode)
				break;
			ret = ret->next;
		}
		if(mcb.line_circular_flag)
			link_to_circular_list(mcb.use_mto_head_line);
	}
	return ret;
}

/* 切换到指定阈值 */
int mto_switch_p(u8 th1,u8 th2,u8 th3,u8 mode)
{
	MTO* p_mto;
	p_mto = mto_find(th1,th2,th3,mode);
	if(p_mto==NULL)
		return NOT_FOUND;
	else if(mode==wall)
	{
		mcb.cur_mto_wall = p_mto;
		if(mcb.measure_mode==wall)
		{
			write_wa(th1 ,&TPL1_CS, &TPL1_CLK, &TPL1_D); //set voltage thread of stop1 pulse
			write_wa(th2, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop2 pulse
			write_wb(th3, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop3 pulse
		}
	}		
	else if(mode==line)
	{
		mcb.cur_mto_line = p_mto;
		if(mcb.measure_mode==line)
		{
			write_wa(th1 ,&TPL1_CS, &TPL1_CLK, &TPL1_D); //set voltage thread of stop1 pulse
			write_wa(th2, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop2 pulse
			write_wb(th3, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop3 pulse
		}
	}
	
	return NON_ERR;
}

/* 获得当前测量模式 */
u8 get_measure_mode()
{
	return mcb.measure_mode;
}

/* 设置测量模式 ： wall line */
int set_measure_mode(u8 _measure_mode)
{
	u8 t1,t2,t3;
	if(_measure_mode==wall)
	{
		mcb.measure_mode = wall;
		t1 = mcb.cur_mto_wall->th1;
		t2 = mcb.cur_mto_wall->th2;
		t3 = mcb.cur_mto_wall->th3;
	}
	else if(_measure_mode==line)
	{
		mcb.measure_mode = line;
		t1 = mcb.cur_mto_line->th1;
		t2 = mcb.cur_mto_line->th2;
		t3 = mcb.cur_mto_line->th3;
	}
	else
		return UNDEFINE_MODE;
	write_wa(t1 ,&TPL1_CS, &TPL1_CLK, &TPL1_D); //set voltage thread of stop1 pulse
	write_wa(t2, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop2 pulse
	write_wb(t3, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop3 pulse
	return NON_ERR;
}


/* 获得当前的多阈值MTO块 */
MTO* get_cur_mto()
{
	if(get_measure_mode()==wall)
		return mcb.cur_mto_wall;
	if(get_measure_mode()==line)
		return mcb.cur_mto_line;
}

int adj_line_60_70_80(TIME_DATA *data,double *ans)
{
	if(data->err_time3 <=1600 || data->time_ps< TIME_PS_TH*1.2)
	{
		*ans =((1.5 * (double) data->time_ps)/10000) - (4.561*sin(0.005508*((1.5 * (double) data->time_ps)/10000)+0.5009)) -batch_err;
		
		printf("A_\r\n");

	}
	else if(data->err_time3 >1600 && data->err_time3< 2000)
	{
		*ans =  ((1.5 * (double) data->time_ps)/10000) - 0.8541 * my_log(-10.9372 * (data->err_time3) + 11430) + 3.019 - coe -0.3 -batch_err;

	
		printf("B_\r\n");

	}
	else if(data->err_time3 >= 2000)
	{
		*ans = ((1.5 * (double) data->time_ps)/10000) - 0.0004 * (data->err_time3) - 4.0737 - coe -0.3 -batch_err;


		printf("C_\r\n");
	}
	else
	{
		return LINE_OUT;
	}

	return 0;
}

int adj_wall_60_70_80(TIME_DATA *data, double *ans)
{
	
		if(data->err_time3 <=1600)
		{
			*ans =((1.5 * (double) data->time_ps)/10000) - (4.561*sin(0.005508*((1.5 * (double) data->time_ps)/10000)+0.5009)) -batch_err;
		}
		else if(data->err_time3 >1600 && data->err_time3< 2000)
		{
			*ans =  ((1.5 * (double) data->time_ps)/10000) - 0.8541 * my_log(-10.9372 * (data->err_time3) + 11430) + 3.019 - coe -0.3 -batch_err;
		}
		else if(data->err_time3 >= 2000)
		{
			*ans = ((1.5 * (double) data->time_ps)/10000) - 0.0004 * (data->err_time3) - 4.0737 - coe -0.3 -batch_err;
		}
		else
		{
			return LINE_OUT;
		}


	
	
	return 0;
}

int adj_line_80_90_100(TIME_DATA *data,double *ans)
{
	

	if (data->err_time3 > 1100 && data->err_time3 < 2000)
		{
		*ans 				=  ((1.5 * (double) data->time_ps)/10000) - 0.8541 * my_log(-10.9372 * (data->err_time3) + 11430) + 3.019 - coe -0.3 -batch_err;
		}
	else if ((data->err_time3) >= 2000)
		{
		*ans 				= ((1.5 * (double) data->time_ps)/10000) - 0.0004 * (data->err_time3) - 4.0737 - coe -0.3 -batch_err;
		}
	else 
		{
		return LINE_OUT;
		}

	return 0;
}

int adj_wall_80_90_100(TIME_DATA *data, double *ans)
{
	if(data->err_time3 < 2000)
		{	
			*ans =((1.5 * (double) data->time_ps)/10000) - (4.561*sin(0.005508*((1.5 * (double) data->time_ps)/10000)+0.5009)) -batch_err;
		}
	else
		{
			return WALL_OUT;
		}

	return 0;
}



