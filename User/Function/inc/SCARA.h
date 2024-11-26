#ifndef __SCARA_H__
#define __SCARA_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
/* 宏定义 --------------------------------------------------------------------*/
/****************机械臂宏定义***********************/
#define X_AXIS   0
#define Y_AXIS   1
#define Z_AXIS   2
// Length of inner support arm
#define LINKAGE1        (130)      //mm
// Length of outer support arm    
#define LINKAGE2        (130)      //mm

#define CENCER_OFFSET   (28.25f)    //mm
#define HEAD_OFFSET     (85.62f)    //mm

#define SCARA_OFFSET_X  (0)    //mm   
#define SCARA_OFFSET_Y  (0)    //mm   
#define SCARA_OFFSET_Z  (-145.0f)  //mm

#define SCARA_RAD2DEG   (57.2957795f)  // to convert RAD to degrees

#define BIGARM_INIT_ANGLE   (90.0f)    //大臂复位角度
#define FOREARM_INIT_ANGLE  (0.0f)   //小臂复位角度
#define ROTAT_INIT_ANGLE    (0.0f)   //旋转电机复位角度

#define SERVO_BIGARM_ZERO   1330  //大臂，靠近底部选择舵机
#define SERVO_FOREARM_ZERO  1340  //小臂，靠近手爪的臂
#define SERVO_YAW_ZERO      2120  //底部旋转舵机

#define SERVO_ANGLE (135.0f)
/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
	float Linkage_1;       //主臂长度
	float Linkage_2;       //副臂长度
	float CencerOffset;    //旋转轴偏移量
	float HeadOffset;      //头部偏移量
	float axis_scaling[3]; //缩放比例
	float SCARA_offset[3]; //机械臂笛卡尔坐标系偏移量
	float Angle_BIGARM;    //计算角度（主臂）
	float Angle_FOREARM;   //计算角度（副臂）
	float Angle_ROTAT;     //计算角度（旋转轴）
	float cartesian[3];    //笛卡尔坐标系（设置值）
	float axis[3];         //机械臂正解得出的笛卡尔坐标
	float Angle[3];        //机械臂反解得出的角度
} _s_SCARA;
/* 扩展变量 ------------------------------------------------------------------*/
extern _s_SCARA SCARA_S;
/* 函数声明 ------------------------------------------------------------------*/
void calculate_SCARA_forward_Transform(float Angle_BIGARM,float Angle_FOREARM,float Angle_ROTAT); //机械臂正解函数
void calculate_delta(float cartesian[3]);//机械臂反解函数
void Servo_AngleToPWM(float Angle[3],
	                 uint16_t *qAngleValue_ARM1,
					 uint16_t *qAngleValue_ARM2,
					 uint16_t *qAngleValue_YAW);//舵机角度转换位PWM
void Servo_PWMToAngle(uint16_t AngleValue_ARM1,
					  uint16_t AngleValue_ARM2,
					  uint16_t AngleValue_YAW,
	                  float Angle[3]);//舵机PWM转换位角度








#endif  // __SCARA_H__

