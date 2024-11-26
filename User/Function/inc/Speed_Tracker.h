#ifndef __SPEED_TRACKER_H__
#define __SPEED_TRACKER_H__

#ifdef __cplusplus
extern "C" {
#endif
/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
/* 宏定义 --------------------------------------------------------------------*/
#define SPEENTRACKER_TYPE int16_t
/* 类型定义 ------------------------------------------------------------------*/
/**
  * Speed_Tracker类结构体定义
**/
typedef struct{
	//配置(控制周期)
	uint16_t            CtrlFreqHZ;
	SPEENTRACKER_TYPE	up_acc;
	SPEENTRACKER_TYPE	down_acc;
	//计算过程数据
	SPEENTRACKER_TYPE	course_mut;	 //过程加速度积分(放大CONTROL_FREQ_HZ倍)
	SPEENTRACKER_TYPE	course;		 //过程速度
	//输出跟踪量
	SPEENTRACKER_TYPE	go_speed;	 //立即速度
}Speed_Tracker_Typedef;
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Speed_Tracker_Init( Speed_Tracker_Typedef  *handle,
	                     uint16_t               CtrlFreqHZ,
                         SPEENTRACKER_TYPE	    up_acc,
	                     SPEENTRACKER_TYPE	    down_acc ); //速度跟踪器初始化
void Speed_Tracker_NewTask(Speed_Tracker_Typedef *handle, SPEENTRACKER_TYPE real_speed);			//速度控制器开始新任务
SPEENTRACKER_TYPE Speed_Tracker_Capture_Goal(Speed_Tracker_Typedef *handle, SPEENTRACKER_TYPE goal_speed);	//速度跟踪器获得立即速度

#ifdef __cplusplus
}
#endif

#endif //__SPEED_TRACKER_H__
