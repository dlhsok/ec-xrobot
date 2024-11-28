#ifndef __CHASSIS_LINETRACKER_H__
#define __CHASSIS_LINETRACKER_H__
/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
#include "Speed_Tracker.h"
#include "mytype.h"
/* 宏定义 --------------------------------------------------------------------*/
#define LINE_PATROL_DELAY(x) My_mDelay(x)

//定义 弧度 转换到 角度的比例
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE (57.295779513082320876798154814105)
#endif

#define ORIENTATION_INNER   0
#define ORIENTATION_OUTER   1
#define PH2_0_ORIENTATION   ORIENTATION_INNER

#define LINETRACKER_ADC_THRESHOLD          230   // 循迹判断黑线的ADC阈值
#define LINETRACKER_LINE_REFERENCE         5    //横线条件
#define LINETRACKER_SPEED_MAX              1000 //最大速度
#define LINETRACKER_SPEED_START            200  //起步速度
#define LINETRACKER_SPEED_STOP             200  //止步速度
#define LINETRACKER_SPEED_BEFORE_TURNING   400  //转弯前前进速度
#define LINETRACKER_SPEED_UP_ACC           (LINETRACKER_SPEED_MAX/3)
#define LINETRACKER_SPEED_DOWN_ACC         (LINETRACKER_SPEED_MAX/3)

#define LINETRACKER_ANGULAR_SPEED_MAX      150  //最大角速度
#define LINETRACKER_ANGULAR_SPEED_START    25   //起步角速度
#define LINETRACKER_ANGULAR_SPEED_STOP     25   //止步角速度
#define LINETRACKER_ANGULAR_SPEED_UP_ACC   (LINETRACKER_ANGULAR_SPEED_MAX*2)
#define LINETRACKER_ANGULAR_SPEED_DOWN_ACC (LINETRACKER_ANGULAR_SPEED_MAX*2)
#define LINETRACKER_ANGLE_OFFSET_STOP      10    //转弯90°时如果发现转弯角度大于此值时里面停止

#define LINETRACKER_WHEELTRACK  (338.5f) //轮距(mm)
#define LINETRACKER_WHEELBASE   (300)    //轴距(mm)
//#define WHEEL_D                 (141.0f) //轮子直径，单位mm
//#define CALC_C                  (1750)   //计算系数，(电机一圈脉冲数*10/Π)

#if 0
#define RL_DISTANCE_FAC         (16.57f)     //2个光敏电阻之间的距离(mm)
#define RL_LENGTH_FAC           (115.99f)    //循迹条长度
#else
#define RL_DISTANCE_FAC         (13.0f)     //2个光敏电阻之间的距离(mm)
#define RL_LENGTH_FAC           (91.0f)    //循迹条长度
#endif
#define A_LENGTH                (265.0f)     //循迹条A距离（x方向）
#define B_LENGTH                (435.0f)     //循迹条B距离（y方向）


#define X_FDBCK_P    5.0f
#define X_FDBCK_I    0.0f
#define X_FDBCK_D    5.0f
#define YAW_FDBCK_P  6.0f
#define YAW_FDBCK_I  0.0f
#define YAW_FDBCK_D  6.0f
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
  CorrectiveMode_Slash_A = 0,  //单斜线修正
  CorrectiveMode_Slash_Double, //双斜线修正（这里的双线指的是十字交叉线）
} CorrectiveMode_e;
typedef enum
{
  CarDirection_Head = 0,
  CarDirection_Tail,
  CarDirection_Left,
  CarDirection_Right,
} DirectionDef_e;
typedef enum
{
  Mode_Stop = 0,               //停止
  Mode_Await,                  //等待
  Mode_OpenLoopWalk_Delay,     //延时开环控制
  Mode_LineTracker_LineNum,    //线条巡线
  Mode_LineTracker_Delay,      //延时巡线
  Mode_LineTracker_Condition,  //条件巡线
  Mode_LineTracker_SituAdjust, //原地矫正
  Mode_LineTracker_Wheel_90,   //90度转弯
  Mode_Rotate_Angle,           //旋转角度
  Mode_LineTracker_Encoder,    //编码器巡线
  Mode_LineSingleTracker,		//单线循迹
} ModeDef_e;
typedef union
{
	/*ls8 ——> ls1*/
		uint8_t adc_byte[8];
    uint8_t byte;
    struct
    {
    #if (PH2_0_ORIENTATION == ORIENTATION_OUTER)
		//接口朝外
		uint8_t  ls8 :1;
        uint8_t  ls7 :1;
        uint8_t  ls6 :1;
        uint8_t  ls5 :1;
        uint8_t  ls4 :1;
        uint8_t  ls3 :1;
        uint8_t  ls2 :1;
        uint8_t  ls1 :1;
	#else
		//接口朝内
				uint8_t  adc;
        uint8_t  ls1 :1;
        uint8_t  ls2 :1;
        uint8_t  ls3 :1;
        uint8_t  ls4 :1;
        uint8_t  ls5 :1;
        uint8_t  ls6 :1;
        uint8_t  ls7 :1;
        uint8_t  ls8 :1;
				
	#endif
    }bit;
} SignalDef_u;
typedef struct
{
  uint8_t  run_state; //状态
  uint8_t  line_num;  //目标条数
  uint8_t  CorrectiveCtrl_Flag;
  DirectionDef_e Car_Direction;
  int16_t  y_axis_set; //y设置值
  int16_t  x_axis_set; //x设置值
  int16_t  yaw_set;    //旋转设置值
  int32_t  TimeMem;    //时间存储器
  int32_t  SetVal;     //设置存储器
  int32_t  *pTotal_ecd;//编码器读取数据指针
  uint16_t (*ReadData_func)(void);//数据读取函数指针
} LineTracker_ParaTypeDef;

typedef struct
{
  uint8_t  run_state; //状态

} SingleLine_ParaTypeDef;

typedef struct
{
  uint8_t   active_level; //信号有效电平储存器
  uint8_t   line_cnt;     //计数器
  ModeDef_e run_mode;     //运动模式
  uint16_t  CtrlFreqHZ;    //配置(控制周期)
  int16_t   y_axis_out;   //y输出值
  int16_t   x_axis_out;   //x输出值
  int16_t   yaw_out;      //旋转输出值
  int16_t   y_axis_PracticalOut;//Y实际输出值
  int16_t   x_axis_PracticalOut;//X实际输出值
  int16_t   yaw_PracticalOut;   //旋转实际输出值
  Speed_Tracker_Typedef Speed_TrackerStruct;
  float   Line1;
  float   Line2;
  float   D_Offs;
  float   Alpha;
  SignalDef_u *pHeadSignal;    //相对方向头信号
  SignalDef_u *pSignal1;       //信号1
  SignalDef_u *pSignal2;       //信号2
  SignalDef_u *pSignal3;       //信号3
  SignalDef_u *pSignal4;       //信号4
  SingleLine_ParaTypeDef	*pSingleLineTracker_Struct;
  LineTracker_ParaTypeDef *pLineTracker_ParaStruct;
} LineTracker_TypeDef;
/* 扩展变量 ------------------------------------------------------------------*/
extern LineTracker_TypeDef LineTracker_Struct;
/* 函数声明 ------------------------------------------------------------------*/
void LineTracker_Init( uint8_t active_level,
                       uint16_t CtrlFreqHZ,
                       uint8_t *pSignal1,
                       uint8_t *pSignal2,
                       uint8_t *pSignal3,
                       uint8_t *pSignal4 );
void LineTracker_Scan(void);
uint16_t ReadSignal1_TransverseLine_func(void);
uint16_t ReadSignal2_TransverseLine_func(void);
uint16_t ReadSignal3_TransverseLine_func(void);
uint16_t ReadSignal4_TransverseLine_func(void);
void LineTracker_WaitCarToStop(void);
void LineTracker_Execute_OpenLoopWalk_Delay(DirectionDef_e Car_Direction, int16_t y_val, int16_t x_val, int16_t yaw_val, uint16_t time);
void LineTracker_Execute_LineNum(DirectionDef_e Car_Direction, uint16_t spd, uint8_t line_num, uint8_t mode, uint8_t CorrectiveCtrl_Flag);
void LineTracker_Execute_Delay(DirectionDef_e Car_Direction, uint16_t spd, uint16_t time, uint8_t CorrectiveCtrl_Flag);
void LineTracker_Execute_Condition(DirectionDef_e Car_Direction, uint16_t spd, uint16_t (*ReadData_func)(void), uint16_t TargetState, uint8_t CorrectiveCtrl_Flag);
void LineTracker_Execute_Condition1(DirectionDef_e Car_Direction, uint16_t spd, uint16_t (*ReadData_func)(void), uint16_t TargetState, uint8_t CorrectiveCtrl_Flag);
void LineTracker_Execute_Wheel_90(DirectionDef_e Car_Direction, int16_t spd, uint16_t distance, uint8_t CorrectiveCtrl_Flag);
void LineTracker_Execute_RotateAngle(DirectionDef_e Car_Direction, int16_t spd, int16_t angle);
void LineTracker_Execute_Encoder(DirectionDef_e Car_Direction, uint16_t spd, int32_t *pTotal_ecd, uint16_t distance, uint8_t CorrectiveCtrl_Flag);
void LineTracker_Execute_SituAdjust(DirectionDef_e Car_Direction, uint8_t mode, uint16_t OutTime);
void LineTracker_ChassisPostureCalc(SignalDef_u *pHeadSignal, SignalDef_u *pTailSignal);
void LineTracker_CorrectiveCtrl(CorrectiveMode_e mode);
#endif  // __CHASSIS_LINETRACKER_H__

