#ifndef __DATA_H
#define __DATA_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include "mytype.h"
#include "bsp.h"
#include "RecognitionModule.h"
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#ifndef  FALSE
    #define  FALSE    0
#endif
#ifndef  TRUE
    #define  TRUE     1
#endif

#define DataStorageFlg_Leisure 0   //空闲
#define DataStorageFlg_Start   1   //进行存储
#define DataStorageFlg_ERR     2   //失败
#define DataStorageFlg_ERRADDR 3   //地址错误

#ifdef __SLAVE_DCMotorMiniwatt_H__
#define DCMotorMiniwatt1_S DCMotorMiniwatt_S[0]
#define DCMotorMiniwatt2_S DCMotorMiniwatt_S[1]
#define DCMotorMiniwatt3_S DCMotorMiniwatt_S[2]
#define DCMotorMiniwatt4_S DCMotorMiniwatt_S[3]
#endif
#ifdef __SLAVE_Tracking_H__
#define Tracking_Device1 Tracking_Device[0]
#define Tracking_Device2 Tracking_Device[1]
#define Tracking_Device3 Tracking_Device[2]
#define Tracking_Device4 Tracking_Device[3]
#endif
#ifdef __SLAVE_SteeringEngine_6CH_H__
#define SERVO_NUM_MAX 6
#define SERVO_CH1     0
#define SERVO_CH2     1
#define SERVO_CH3     2
#define SERVO_CH4     3
#define SERVO_CH5     4
#define SERVO_CH6     5

#define SERVO_CLAW_J        2300
#define SERVO_CLAW_S        1800
#define SERVO_CLAW_S_MAX    1400

#define SERVO_CLAW SERVO_CH1 //手爪舵机
#define SERVO_ARM2 SERVO_CH2 //臂2舵机，靠近手爪的臂
#define SERVO_ARM1 SERVO_CH3 //臂1舵机，靠近底部选择舵机
#define SERVO_YAW  SERVO_CH4 //底部旋转舵机
#endif
/******************************************************************************
 *              底盘速度设置                                             *
 ******************************************************************************/
#define MAX_WHEEL_RPM               (200.0f)
#define RC_CHASSIS_MAX_SPEED_X      (500.0f)     //X轴方向最大速度(mm/s)
#define RC_CHASSIS_MAX_SPEED_Y      (500.0f)     //Y轴方向最大速度(mm/s)
#define RC_CHASSIS_MAX_SPEED_R      (150.0f)      //旋转最大速度(deg/s)
#define RC_GIMBAL_MOVE_RATIO_YAW    (0.12f)        //yaw移动比例
/******************************************************************************
 *              舵机参数设置                                             *
 ******************************************************************************/
#define SERVO_CH_NUM            3
#define Recognition_RX_LEN  	20
#define Rotation_Claw_Init      FlashData_Struct.Servo_RotationClawInit
#define Claw_S                  FlashData_Struct.Servo_Claw_S
#define Claw_J                  FlashData_Struct.Servo_Claw_J
/******************************************************************************
 *              机械臂参数设置                                             *
 ******************************************************************************/
#define DEVICE_ROBOTARM_ID      2
#define DEVICE_ROBOTARM_MAX_SPEED_X      (100.0f)     //X轴方向最大速度(mm/s)
#define DEVICE_ROBOTARM_MAX_SPEED_Y      (100.0f)     //Y轴方向最大速度(mm/s)
#define DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO (100.0f) // 位置跟踪器放大倍数
/******************************************************************************
 *              电机旋转方向设置                                             *
 ******************************************************************************/
#define FWD                         (0) //正转
#define REV                         (1) //反转
#define DCMOTOR1_DIR                FWD
#define DCMOTOR2_DIR                REV
#define DCMOTOR3_DIR                REV
#define DCMOTOR4_DIR                FWD


/* Exported types ------------------------------------------------------------*/
extern DCMotorMiniwattDef_t DCMotorMiniwatt_S[4];
typedef struct
{
	uint8_t  FH1;
	uint8_t  FH2;
    uint16_t Servo_RotationClawInit;
	uint16_t Servo_Claw_S;
	uint16_t Servo_Claw_J;
	uint16_t FlashData_Len;    //存储数据的长度
	uint16_t Bit16CRC;
}FlashData_Typedef; //必须是4个字节的倍数
typedef struct
{
	uint8_t  DataStorage_Flg;    //数据存储标记
}AppData_Typedef;
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t yaw;
	bool rec_flg; // 接受到新数据标志位
}BrickData_TypeDef;
typedef struct
{
	__IO uint16_t ServoPwmDuty[SERVO_CH_NUM];	//舵机PWM脉冲宽度
	__IO uint16_t ServoTime[SERVO_CH_NUM];	    //舵机时间
	__IO uint16_t ID;    //ID
	__IO union
	{//机械臂控制标记
		uint16_t WORD;
		struct
		{
			uint16_t YV1                  :1;
			uint16_t YV2                  :1;
			uint16_t YV3                  :1;
			uint16_t YV4                  :1;
			uint16_t DazzleLight          :1;
			uint16_t retain0              :1;
			uint16_t AirCtrl              :1;  //气泵控制
			uint16_t SCARAReset_Enabled   :1;  //复位使能
			uint16_t AngleCtrl_Enabled    :1;  //角度控制使能
			uint16_t AS5600_Enabled       :1;  //AS5600使能
			uint16_t XYZ_Enabled          :1;  //坐标系使能
			uint16_t Scram_Enabled        :1;  //急停
			uint16_t retain1              :4;
		}bit;
	}SCARAflg_U;
	__IO union
	{//机械臂状态标志
		uint16_t WORD;
		struct
		{
			uint16_t IN1                  :1;
			uint16_t IN2                  :1;
			uint16_t IN3                  :1;
			uint16_t IN4                  :1;
			uint16_t Motor1_RunSta        :1;
			uint16_t Motor2_RunSta        :1;
			uint16_t Motor3_RunSta        :1;
			uint16_t Motor1_EnabledSta    :1;
			uint16_t Motor2_EnabledSta    :1;
			uint16_t Motor3_EnabledSta    :1;
			uint16_t AS5600_1_InitSta     :1;
			uint16_t AS5600_2_InitSta     :1;
			uint16_t AS5600_3_InitSta     :1;
			uint16_t MotorCtrl_flg        :1; //电机控制模式下电机控制标记
			uint16_t W25QxxErr_flg        :1;
			uint16_t retain               :1;
		}bit;
	}UploadData_U;
	float MotorAngle_Target[3]; //角度控制值
	float SCARA_Cartesian[3];//机械臂坐标系
}RobotArmData_Typedef;
typedef struct 
{
	uint8_t  mark;
	int16_t stcAcc[3];  //加速度
	int16_t stcGyro[3]; //角速度
	int16_t stcAngle[3];//角度
	int16_t  stcTemp;
	float temp;
	float ax;
	float ay;
	float az;
	float wx;
	float wy;
	float wz;
	float Roll;
	float Pitch;
	float Yaw;
}GyroData_Typedef;
typedef struct
{
	uint8_t sta;
	/* 拨杆 */
    uint8_t sw;
	/* 摇杆 */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
	/* 遥控取值范围 做转化用 */
	__IO float RC_Resolution;
	/* 姿态 */
	__IO struct
    {
        float vx;
        float vy;
        float vyaw;
		float vpitch;
    } sPosition;
}_s_RemoteCtrl_Info;
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern FlashData_Typedef FlashData_Struct;
extern AppData_Typedef   AppData_Struct;
/*******直流电机驱动模块数据声明*******/
#ifdef __SLAVE_DCMotorMiniwatt_H__
extern DCMotorMiniwattDef_t DCMotorMiniwatt_S[4];
#endif
/*******舵机驱动模块数据声明*******/
#ifdef __SLAVE_SteeringEngine_6CH_H__
extern SteeringEngine6CHDef_t Servo_S;
extern uint16_t AngleValue[6];
extern uint16_t AngleTime[6];
#endif
/*******巡线条模块数据声明*******/
#ifdef __SLAVE_Tracking_H__
extern TrackingDef_t Tracking_Device[4];
#endif
/*******超声波模块数据声明*******/
#ifdef __SLAVE_UltrasonicRanging_H__
extern UltrasonicRangingDef_t UltrasonicRanging_S;
#endif
/*******陀螺仪数据声明*******/
extern GyroData_Typedef GyroData_Struct;
/*******遥控数据数据声明*******/
extern _s_RemoteCtrl_Info sRemoteCtrl_Info;
/*******机械臂数据数据声明*******/
extern RobotArmData_Typedef RobotArmData_Struct;
/*******识别模块数据声明*******/
extern RecognitionModule_s RecognitionModule_t;
extern uint8_t Recognition_Buffer[Recognition_RX_LEN];
extern uint8_t bool_recognitionflag;
/*******砖数据声明*******/
extern BrickData_TypeDef BrickData_Struct;
/* Exported functions --------------------------------------------------------*/
int AppData_Init(void);
uint8_t Application_DataFlash_Modification(void);




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
