#ifndef __RAMP_TRACKER_H__
#define __RAMP_TRACKER_H__
#ifdef __cplusplus
extern "C" {
#endif
/* 包含头文件 ----------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
/* 宏定义 --------------------------------------------------------------------*/
#define SPEENTRACKER_TYPE int32_t
/* 类型定义 ------------------------------------------------------------------*/
/**
  * Ramp_Tracker类结构体定义
**/
typedef struct{
	//配置(控制周期)
	uint16_t            CtrlFreqHZ;
	SPEENTRACKER_TYPE	up_acc;
	SPEENTRACKER_TYPE	down_acc;
	//计算过程数据
	SPEENTRACKER_TYPE	course_mut;	 //过程积分(放大CONTROL_FREQ_HZ倍)
	SPEENTRACKER_TYPE	course;		 //过程值
	//输出跟踪量
	SPEENTRACKER_TYPE	go_value;	 //立即速度
}Ramp_Tracker_Typedef;
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Ramp_Tracker_Init( Ramp_Tracker_Typedef *handle,
	                    uint16_t             CtrlFreqHZ,
                        SPEENTRACKER_TYPE	 up_acc,
	                    SPEENTRACKER_TYPE	 down_acc ); //速度跟踪器初始化
void Ramp_Tracker_NewTask( Ramp_Tracker_Typedef *handle, SPEENTRACKER_TYPE real_value );			       //速度控制器开始新任务
SPEENTRACKER_TYPE Ramp_Tracker_Capture_Goal( Ramp_Tracker_Typedef *handle, SPEENTRACKER_TYPE goal_value ); //速度跟踪器获得立即速度

#ifdef __cplusplus
}
#endif

#endif //__RAMP_TRACKER_H__
