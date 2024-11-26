/* 包含头文件 ----------------------------------------------------------------*/
//Oneself
#include "Speed_Tracker.h"
/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 扩展函数定义 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/**
  * 速度跟踪器初始化
  * @param  tracker		速度跟踪器实例
  * @retval NULL
**/
void Speed_Tracker_Init( Speed_Tracker_Typedef  *handle,
	                     uint16_t               CtrlFreqHZ,
                         SPEENTRACKER_TYPE	    up_acc,
	                     SPEENTRACKER_TYPE	    down_acc )
{
	//配置
	handle->CtrlFreqHZ = CtrlFreqHZ;
	handle->up_acc = up_acc;
	handle->down_acc = down_acc;
	//计算过程数据
	handle->course_mut = 0;
	handle->course = 0;
	//输出跟踪量
	handle->go_speed = 0;
}
/**
  * 速度跟踪器开始新任务
  * @param  tracker			速度跟踪器实例
  * @param  real_speed		实时速度
  * @retval NULL
**/
void Speed_Tracker_NewTask(Speed_Tracker_Typedef  *handle, SPEENTRACKER_TYPE real_speed)
{
	//更新计算过程数据
	handle->course_mut = 0;		//过程加速度积分
	handle->course = real_speed;//过程速度
}
/**
  * 速度跟踪器速度积分
  * @param  value	加速度
**/
#if 1
    inline static void Speed_Course_Integral(Speed_Tracker_Typedef  *handle, int32_t value)
    {
        handle->course_mut += value;									\
	    handle->course += (handle->course_mut / handle->CtrlFreqHZ);	\
	    handle->course_mut = handle->course_mut % handle->CtrlFreqHZ;	\
    }
#else
    #define Speed_Course_Integral(handle, value)	\
    {	\
	    handle->course_mut += value;									\
	    handle->course += (handle->course_mut / handle->CtrlFreqHZ);	\
	    handle->course_mut = handle->course_mut % handle->CtrlFreqHZ;	\
    }
#endif
/**
  * 速度跟踪器获得立即速度
  * @param  tracker			速度跟踪器实例
  * @param  goal_speed		目标速度
  * @retval NULL
**/
SPEENTRACKER_TYPE Speed_Tracker_Capture_Goal(Speed_Tracker_Typedef *handle, SPEENTRACKER_TYPE goal_speed)
{
	//整形速度差
	SPEENTRACKER_TYPE speed_sub = goal_speed - handle->course;
	
	if(speed_sub == 0)
	{//速度到达目标速度
		handle->course = goal_speed;		//进入稳速状态
	}
	else if(speed_sub > 0)
	{//矢量速度小于目标
		if(handle->course >= 0)
			Speed_Course_Integral(handle, handle->up_acc);
		else
			Speed_Course_Integral(handle, handle->down_acc);
		if(handle->course >= goal_speed)
		{
			handle->course_mut = 0;
			handle->course = goal_speed;
		}
	}
	else if(speed_sub < 0)
	{//矢量速度大于目标
		if(handle->course <= 0)
			Speed_Course_Integral(handle, -handle->up_acc);
		else
			Speed_Course_Integral(handle, -handle->down_acc);
		if(handle->course <= goal_speed)
		{
			handle->course_mut = 0;
			handle->course = goal_speed;
		}
	}
	
	//输出
	handle->go_speed = (SPEENTRACKER_TYPE)handle->course;
	
	return handle->go_speed;
}
