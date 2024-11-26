#ifndef __LOCATION_TRACKER_H__
#define __LOCATION_TRACKER_H__

#ifdef __cplusplus
extern "C" {
#endif
/* 包含头文件 ----------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
/* 宏定义 --------------------------------------------------------------------*/
#define LOCATIONTRACKER_TYPE int32_t
/* 类型定义 ------------------------------------------------------------------*/
/**
  * Location_Tracker类结构体定义
**/
typedef struct{
	//配置(控制周期)
	uint16_t                CtrlFreqHZ;
	//配置(速度)
	LOCATIONTRACKER_TYPE	max_speed;
	//配置(加速加速度)
	LOCATIONTRACKER_TYPE	up_acc;
	//配置(减速加速度)
	LOCATIONTRACKER_TYPE	down_acc;
	//快速运算数		1.0f / (2.0f * down_acc)
	float	down_acc_quick;
	//静态配置的跟踪参数
	LOCATIONTRACKER_TYPE	speed_locking_stop;		//允许直接抱死停车的速度
	//计算过程数据
	LOCATIONTRACKER_TYPE	course_acc_integral;	//过程加速度积分(放大CONTROL_FREQ_HZ倍)
	LOCATIONTRACKER_TYPE	course_speed;			//过程速度
	LOCATIONTRACKER_TYPE	course_speed_integral;	//过程速度积分(放大CONTROL_FREQ_HZ倍)
	LOCATIONTRACKER_TYPE	course_location;		//过程位置
	//输出跟踪控制量
	LOCATIONTRACKER_TYPE	go_location;	        //立即位置
	LOCATIONTRACKER_TYPE	go_speed;			    //立即速度
}Location_Tracker_Typedef;
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Location_Tracker_Init( Location_Tracker_Typedef *handle,
	                        uint16_t                 CtrlFreqHZ,
	                        LOCATIONTRACKER_TYPE	 max_speed,
                            LOCATIONTRACKER_TYPE	 up_acc,
	                        LOCATIONTRACKER_TYPE	 down_acc,
                            LOCATIONTRACKER_TYPE	 speed_locking_stop );
void Location_Tracker_NewTask( Location_Tracker_Typedef *handle,
	                           LOCATIONTRACKER_TYPE     real_location,
                               LOCATIONTRACKER_TYPE     real_speed );
void Location_Tracker_Capture_Goal( Location_Tracker_Typedef *handle, 
	                                LOCATIONTRACKER_TYPE     goal_location );


#ifdef __cplusplus
}
#endif

#endif //__LOCATION_TRACKER_H__
