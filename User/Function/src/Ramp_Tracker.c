/* 包含头文件 ----------------------------------------------------------------*/
//Oneself
#include "Ramp_Tracker.h"
/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 扩展函数定义 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/**
  * 跟踪器初始化
  * @param  tracker		跟踪器实例
  * @retval CtrlFreqHZ  控制频率
  * @retval up_acc      增流梯度
  * @retval down_acc    减流梯度
**/
void Ramp_Tracker_Init( Ramp_Tracker_Typedef  *handle,
	                    uint16_t              CtrlFreqHZ,
                        SPEENTRACKER_TYPE	  up_acc,
	                    SPEENTRACKER_TYPE	  down_acc )
{
	//配置
	handle->CtrlFreqHZ = CtrlFreqHZ;
	handle->up_acc = up_acc;
	handle->down_acc = down_acc;
	//计算过程数据
	handle->course_mut = 0;
	handle->course = 0;
	//输出跟踪量
	handle->go_value = 0;
}
/**
  * 跟踪器开始新任务
  * @param  tracker			跟踪器实例
  * @param  real_value		实时值
  * @retval NULL
**/
void Ramp_Tracker_NewTask(Ramp_Tracker_Typedef  *handle, SPEENTRACKER_TYPE real_value)
{
	//更新计算过程数据
	handle->course_mut = 0;		//过程值积分
	handle->course = real_value;//过程值
}
/**
  * 跟踪器积分
  * @param  value	梯度
**/
#if 1
    inline static void Ramp_Course_Integral(Ramp_Tracker_Typedef  *handle, int32_t value)
    {
        handle->course_mut += value;									\
	    handle->course += (handle->course_mut / handle->CtrlFreqHZ);	\
	    handle->course_mut = handle->course_mut % handle->CtrlFreqHZ;	\
    }
#else
    #define Ramp_Course_Integral(handle, value)	\
    {	\
	    handle->course_mut += value;									\
	    handle->course += (handle->course_mut / handle->CtrlFreqHZ);	\
	    handle->course_mut = handle->course_mut % handle->CtrlFreqHZ;	\
    }
#endif
/**
  * 跟踪器获得立即值
  * @param  tracker			跟踪器实例
  * @param  goal_value		目标值
  * @retval NULL
**/
SPEENTRACKER_TYPE Ramp_Tracker_Capture_Goal(Ramp_Tracker_Typedef *handle, SPEENTRACKER_TYPE goal_value)
{
	//整形值差
	SPEENTRACKER_TYPE speed_sub = goal_value - handle->course;
	
	if(speed_sub == 0)
	{//值到达目标值
		handle->course = goal_value;		//进入稳速状态
	}
	else if(speed_sub > 0)
	{//矢量值小于目标
		if(handle->course >= 0)
		{
			Ramp_Course_Integral(handle, handle->up_acc);
			if(handle->course >= goal_value)
			{
				handle->course_mut = 0;
				handle->course = goal_value;
			}
		}
		else
		{
			Ramp_Course_Integral(handle, handle->down_acc);
			if(handle->course >= 0)
			{
				handle->course_mut = 0;
				handle->course = 0;
			}
		}
		
	}
	else if(speed_sub < 0)
	{//矢量值大于目标
		if(handle->course <= 0)
		{
			Ramp_Course_Integral(handle, -handle->up_acc);
			if(handle->course <= goal_value)
			{
				handle->course_mut = 0;
				handle->course = goal_value;
			}
		}
		else
		{
			Ramp_Course_Integral(handle, -handle->down_acc);
			if(handle->course <= 0)
			{
				handle->course_mut = 0;
				handle->course = 0;
			}
		}
	}
	
	//输出
	handle->go_value = (SPEENTRACKER_TYPE)handle->course;
	
	return handle->go_value;
}
