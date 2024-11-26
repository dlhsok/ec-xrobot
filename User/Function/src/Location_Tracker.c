/* 包含头文件 ----------------------------------------------------------------*/
//Oneself
#include "Location_Tracker.h"
/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 扩展函数定义 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/**
  * 位置跟踪器速度积分
  * @param  value	加速度
**/
inline static void Speed_Course_Integral(Location_Tracker_Typedef *handle, LOCATIONTRACKER_TYPE value)
{
	handle->course_acc_integral += value;
	handle->course_speed += handle->course_acc_integral / handle->CtrlFreqHZ;
	handle->course_acc_integral = handle->course_acc_integral % handle->CtrlFreqHZ;
}
/**
  * 位置跟踪器位置积分
  * @param  value	速度
**/
inline static void Location_Course_Integral(Location_Tracker_Typedef *handle, LOCATIONTRACKER_TYPE value)
{
	handle->course_speed_integral += value;
	handle->course_location += handle->course_speed_integral / handle->CtrlFreqHZ;
	handle->course_speed_integral = handle->course_speed_integral % handle->CtrlFreqHZ;
}
/**
  * 位置跟踪器初始化
  * @param  tracker		位置跟踪器实例
  * @retval NULL
**/
void Location_Tracker_Init( Location_Tracker_Typedef *handle,
	                        uint16_t                 CtrlFreqHZ,
	                        LOCATIONTRACKER_TYPE	 max_speed,
                            LOCATIONTRACKER_TYPE	 up_acc,
	                        LOCATIONTRACKER_TYPE	 down_acc,
                            LOCATIONTRACKER_TYPE	 speed_locking_stop )
{
	//配置(控制周期)
	handle->CtrlFreqHZ = CtrlFreqHZ;
	//配置(速度)
	handle->max_speed = max_speed;
	//配置(加速加速度)
	handle->up_acc = up_acc;
	//配置(减速加速度)
	handle->down_acc = down_acc;
	//快速运算数		1.0f / (2.0f * down_acc)
	handle->down_acc_quick = 0.5f / (float)handle->down_acc;
	//静态配置的跟踪参数
	handle->speed_locking_stop = speed_locking_stop;//允许直接抱死停车的速度
	//计算过程数据
	handle->course_acc_integral = 0;	//过程加速度积分
	handle->course_speed = 0;			//过程速度
	handle->course_speed_integral = 0;	//过程速度积分
	handle->course_location = 0;		//过程位置
	//输出跟踪控制量
	handle->go_location = 0;	        //立即位置
	handle->go_speed = 0;			    //立即速度
}
/**
  * 位置跟踪器开始新任务
  * @param  tracker			位置跟踪器实例
  * @param  real_location	实时位置
  * @param  real_speed		实时速度
  * @retval NULL
**/
void Location_Tracker_NewTask( Location_Tracker_Typedef *handle,
	                           LOCATIONTRACKER_TYPE     real_location,
                               LOCATIONTRACKER_TYPE     real_speed )
{
	//更新计算过程数据
	handle->course_acc_integral = 0;			//过程加速度积分
	handle->course_speed = real_speed;			//过程速度
	handle->course_speed_integral = 0;			//过程速度积分
	handle->course_location = real_location;	//过程位置
}
/**
  * 位置跟踪器获得立即位置和立即速度
  * @param  tracker			位置跟踪器实例
  * @param  goal_location	目标位置
  * @retval NULL
**/
void Location_Tracker_Capture_Goal( Location_Tracker_Typedef *handle, 
	                                LOCATIONTRACKER_TYPE     goal_location )
{
    //整形位置差
	LOCATIONTRACKER_TYPE location_sub;
	int32_t need_down_location;
	
	location_sub = goal_location - handle->course_location;
	
	if(location_sub == 0)
	{//到达目标
		if( ( handle->course_speed >= (-handle->speed_locking_stop) ) && 
			( handle->course_speed <= handle->speed_locking_stop ) )
		{//速度小于刹停速度
			//进入静止->取整浮点数据
			handle->course_acc_integral = 0;
			handle->course_speed = 0;
			handle->course_speed_integral = 0;
		}
		else if(handle->course_speed > 0)
		{//速度 > 0
			//正向减速到0
			Speed_Course_Integral(handle, -handle->down_acc);
			if(handle->course_speed <= 0)
			{
				handle->course_acc_integral = 0;
				handle->course_speed = 0;
			}
		}
		else if(handle->course_speed < 0)
		{//速度 < 0
			//反向减速到0
			Speed_Course_Integral(handle, handle->down_acc);
			if(handle->course_speed >= 0)
			{
				handle->course_acc_integral = 0;
				handle->course_speed = 0;
			}
		}
	}
	else
	{//未到达目标
		if( handle->course_speed == 0 )
		{//速度为0
			if(location_sub > 0)
			{//开始正向加速
				Speed_Course_Integral(handle, handle->up_acc);
			}
			else
			{//开始反向加速
				Speed_Course_Integral(handle, -handle->up_acc);
			}
		}
		else if( handle->course_speed > 0 )
		{//速度 > 0
			if( location_sub > 0 )
			{//速度与位移方向同向(正方向)
				if( handle->course_speed <= handle->max_speed )
				{
					//计算需要的减速位移(浮点运算)
					//位移等于二分之一的加速度乘以时间的平方
					need_down_location = (int32_t)( (float)handle->course_speed \
						                          * (float)handle->course_speed \
											      * (float)handle->down_acc_quick );
				    if(location_sub > need_down_location)
					{
						if(handle->course_speed < handle->max_speed)
						{//正向加速到最大速度
							Speed_Course_Integral(handle, handle->up_acc);
						    if(handle->course_speed >= handle->max_speed)
						    {
							    handle->course_acc_integral = 0;
							    handle->course_speed = handle->max_speed;
						    }
						}
					}
					else
					{//正向减速到0
					    Speed_Course_Integral(handle, -handle->down_acc);
					    if(handle->course_speed <= 0)
					    {
						    handle->course_acc_integral = 0;
						    handle->course_speed = 0;
					    }
					}
				}
				else
				{//正向减速到0
				    Speed_Course_Integral(handle, -handle->down_acc);
				    if(handle->course_speed <= 0)
				    {
					    handle->course_acc_integral = 0;
					    handle->course_speed = 0;
				    }
				}
			}
			else
			{//速度与位移方向反向(正方向)
				//正向减速到0
			    Speed_Course_Integral(handle, -handle->down_acc);
			    if(handle->course_speed <= 0)
			    {
				    handle->course_acc_integral = 0;
				    handle->course_speed = 0;
			    }
			}
		}
		else if( handle->course_speed < 0 )
		{//速度 < 0
			if( location_sub < 0 )
			{//速度与位移方向同向(负方向)
				if( handle->course_speed >= (-handle->max_speed) )
			    {
					//计算需要的减速位移(有溢出风险的运算)(浮点运算)
				    need_down_location = (int32_t)( (float)handle->course_speed \
						                          * (float)handle->course_speed \
											      * (float)handle->down_acc_quick );
					if((-location_sub) > need_down_location)
					{
					    if(handle->course_speed > (-handle->max_speed))
					    {//反向加速到最大速度
							Speed_Course_Integral(handle, -handle->up_acc);
						    if(handle->course_speed <= -handle->max_speed)
						    {
							    handle->course_acc_integral = 0;
							    handle->course_speed = -handle->max_speed;
						    }
						}
					}
				    else
					{//反向减速到0
						Speed_Course_Integral(handle, handle->down_acc);
						if(handle->course_speed >= 0)
						{
							handle->course_acc_integral = 0;
							handle->course_speed = 0;
						}	
					}
				}
				else
                {//反向减速到0
					Speed_Course_Integral(handle, handle->down_acc);
					if(handle->course_speed >= 0)
					{
						handle->course_acc_integral = 0;
						handle->course_speed = 0;
					}
				}
			}
			else
			{//速度与位移方向反向(负方向)
				//反向减速到0
				Speed_Course_Integral(handle, handle->down_acc);
				if(handle->course_speed >= 0)
				{
					handle->course_acc_integral = 0;
					handle->course_speed = 0;
				}
			}
		}
	}
	
	//位置积分
	Location_Course_Integral(handle, handle->course_speed);
	//输出
	handle->go_location = (int32_t)handle->course_location;
	handle->go_speed = (int32_t)handle->course_speed;
}
