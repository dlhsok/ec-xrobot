/* Includes ------------------------------------------------------------------*/
#include "thread_rccu.h"
#include "bsp.h"
#include "data.h"
#include "chassis_function.h"
#include "pid.h"
#include "Location_Tracker.h"
#include "thread_user.h"
#include "thread_gui.h"
#ifdef __SLAVE_SteeringEngine_6CH_H__
#include "SCARA.h"
#endif
/* Private macros ------------------------------------------------------------*/
//控制频率_hz
#define CONTROL_FREQ_HZ			  (100)
//YAW PID参数
#define De_YAW_KP                 2.0f
#define De_YAW_KI                 0.01f
#define De_YAW_KD                 0.0f
//X PID参数
#define De_LOCX_KP                4.5f
#define De_LOCX_KI                0.1f
#define De_LOCX_KD                3.0f
//Y PID参数
#define De_LOCY_KP                4.5f
#define De_LOCY_KI                0.1f
#define De_LOCY_KD                3.0f
/* Private types -------------------------------------------------------------*/
typedef enum
{
  CHASSIS_RELAX = 0,          //安全模式
  CHASSIS_STOP,               //底盘停止
  CHASSIS_NORMAL,             //底盘正常模式（带陀螺仪）
  CHASSIS_COORD,              //底盘坐标模式
  CHASSIS_TRACKING,           //底盘循迹模式
} ChassisCtrlMode_TypeDef;
typedef struct
{
  //硬目标位置和速度
  float		goal_yaw;
  float		goal_x;
  float		goal_y;
  //软目标位置和速度
  float		soft_yaw;
  float		soft_x;
  float		soft_y;

  Location_Tracker_Typedef Yaw_Tracker_Struct;
  Location_Tracker_Typedef X_Tracker_Struct;
  Location_Tracker_Typedef Y_Tracker_Struct;
} ChassisCoord_CtrlTypeDef;
typedef struct
{
  ChassisCtrlMode_TypeDef mode_order;
  ChassisCtrlMode_TypeDef mode_run;
  float Gyro_YawAngle_zero;
  float Gyro_YawAngle_Calc;
  float Gyro_YawAngle_Chassis;
  float Gyro_YawAngle_Coord;
  ChassisHandle_TypeDef chassis_struct;
  ChassisCoord_CtrlTypeDef ChassisCoord_CtrlStruct;
  pid_t YawAngle_pid;
  pid_t LocationX_pid;
  pid_t LocationY_pid;
  volatile float *qGyro_YawAngle_New;
} RCCUStruct_TypeDef;
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 定义线程控制块指针 */
rt_thread_t thread_rccu = RT_NULL;
/* 底盘控制 */
RCCUStruct_TypeDef rccu_struct;
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  电机模式配置
  * @param  _mode
  * @retval NULL
**/
void ChassisStop(void)
{
  if( rccu_struct.mode_order != CHASSIS_STOP )
  {
    rccu_struct.mode_order = CHASSIS_STOP;
    My_mDelay(1000);
  }
}


static void rccu_setmode( RCCUStruct_TypeDef *rccu_handle, ChassisCtrlMode_TypeDef _mode )
{
  rccu_handle->mode_order = _mode;
}
static float Read_RealYawAngle( RCCUStruct_TypeDef* rccu_handle )
{
  return (*rccu_handle->qGyro_YawAngle_New);
//  return (rccu_handle->chassis_struct.position.angle_deg);
}
static float Read_GyroYawAngleCalc( RCCUStruct_TypeDef* rccu_handle )
{
  //读取陀螺仪值
  float YawAngleCalc;

  YawAngleCalc = Read_RealYawAngle(rccu_handle) - rccu_handle->Gyro_YawAngle_zero;
  if(YawAngleCalc < 0)
    YawAngleCalc += 360;
  if(YawAngleCalc > 360)
    YawAngleCalc -= 360;

  return YawAngleCalc;
}
static void rccu_set_gyro_offset_to_current(void)
{
  rccu_struct.Gyro_YawAngle_zero = Read_RealYawAngle(&rccu_struct);
}

static void rccu_chassisctrl( RCCUStruct_TypeDef* rccu_handle )
{
  //小车底盘控制
  int16_t speed1buff;
  int16_t speed2buff;
  int16_t speed3buff;
  int16_t speed4buff;

  int32_t distance11buff;
  int32_t distance12buff;
  int32_t distance13buff;
  int32_t distance14buff;

  if(rccu_struct.chassis_struct.Chassis_CtrlFunc != 0)
    rccu_struct.chassis_struct.Chassis_CtrlFunc(&rccu_struct.chassis_struct);
  /**************************底盘速度输出到电机上*******************************/
  speed1buff = rccu_handle->chassis_struct.wheel_rpm[0];
  speed2buff = rccu_handle->chassis_struct.wheel_rpm[1];
  speed3buff = rccu_handle->chassis_struct.wheel_rpm[2];
  speed4buff = rccu_handle->chassis_struct.wheel_rpm[3];
#if (DCMOTOR1_DIR == REV)
  speed1buff = -speed1buff;
#endif
#if (DCMOTOR2_DIR == REV)
  speed2buff = -speed2buff;
#endif
#if (DCMOTOR3_DIR == REV)
  speed3buff = -speed3buff;
#endif
#if (DCMOTOR4_DIR == REV)
  speed4buff = -speed4buff;
#endif
  if(lcd_page == 1)
  {
#if 1
    SLAVE_DCMotorMiniwatt_SpeedSet(&DCMotorMiniwatt1_S, speed1buff);
    SLAVE_DCMotorMiniwatt_SpeedSet(&DCMotorMiniwatt2_S, speed2buff);
    SLAVE_DCMotorMiniwatt_SpeedSet(&DCMotorMiniwatt3_S, speed3buff);
    SLAVE_DCMotorMiniwatt_SpeedSet(&DCMotorMiniwatt4_S, speed4buff);
#else
    SLAVE_DCMotorMiniwatt_Digital_SpeedSet(&DCMotorMiniwatt1_S, speed1buff);
    SLAVE_DCMotorMiniwatt_Digital_SpeedSet(&DCMotorMiniwatt2_S, speed2buff);
    SLAVE_DCMotorMiniwatt_Digital_SpeedSet(&DCMotorMiniwatt3_S, speed3buff);
    SLAVE_DCMotorMiniwatt_Digital_SpeedSet(&DCMotorMiniwatt4_S, speed4buff);
#endif
  }
  /**************************电机数据计算到底盘上*******************************/
  rccu_handle->chassis_struct.yaw_gyro_angle = rccu_handle->Gyro_YawAngle_Calc;
  speed1buff = SLAVE_DCMotorMiniwatt_SpeedRead(&DCMotorMiniwatt1_S);
  speed2buff = SLAVE_DCMotorMiniwatt_SpeedRead(&DCMotorMiniwatt2_S);
  speed3buff = SLAVE_DCMotorMiniwatt_SpeedRead(&DCMotorMiniwatt3_S);
  speed4buff = SLAVE_DCMotorMiniwatt_SpeedRead(&DCMotorMiniwatt4_S);
  distance11buff = SLAVE_DCMotorMiniwatt_PlaceRead(&DCMotorMiniwatt1_S);
  distance12buff = SLAVE_DCMotorMiniwatt_PlaceRead(&DCMotorMiniwatt2_S);
  distance13buff = SLAVE_DCMotorMiniwatt_PlaceRead(&DCMotorMiniwatt3_S);
  distance14buff = SLAVE_DCMotorMiniwatt_PlaceRead(&DCMotorMiniwatt4_S);
#if (DCMOTOR1_DIR == REV)
  speed1buff = -speed1buff;
  distance11buff = -distance11buff;
#endif
#if (DCMOTOR2_DIR == REV)
  speed2buff = -speed2buff;
  distance12buff = -distance12buff;
#endif
#if (DCMOTOR3_DIR == REV)
  speed3buff = -speed3buff;
  distance13buff = -distance13buff;
#endif
#if (DCMOTOR4_DIR == REV)
  speed4buff = -speed4buff;
  distance14buff = -distance14buff;
#endif
  rccu_handle->chassis_struct.Chassis_Calc_ForwardTransform( &rccu_handle->chassis_struct,
      speed1buff,
      speed2buff,
      speed3buff,
      speed4buff,
      distance11buff,
      distance12buff,
      distance13buff,
      distance14buff );
}
#if 0
static void rccu_chassisctrl_normalmode( RCCUStruct_TypeDef* rccu_handle )
{
  //有陀螺仪的底盘控制
  float YawAngle_Diff;
#if 0
  rccu_handle->chassis_struct.vw = sRemoteCtrl_Info.sPosition.vyaw;
#else
#if 0
  YawAngle_Diff = rccu_handle->Gyro_YawAngle_Calc - rccu_handle->ChassisCoord_CtrlStruct.soft_yaw;
  while(YawAngle_Diff < -180)
    YawAngle_Diff += 360;
  while(YawAngle_Diff > 180)
    YawAngle_Diff -= 360;
#else
  if(sRemoteCtrl_Info.sPosition.vyaw != 0)
  {
    rccu_handle->Gyro_YawAngle_Chassis = rccu_handle->Gyro_YawAngle_Calc + sRemoteCtrl_Info.sPosition.vyaw;

    if(rccu_handle->Gyro_YawAngle_Chassis < 0)
      rccu_handle->Gyro_YawAngle_Chassis += 360;
    if(rccu_handle->Gyro_YawAngle_Chassis > 360)
      rccu_handle->Gyro_YawAngle_Chassis -= 360;
  }
  YawAngle_Diff = rccu_handle->Gyro_YawAngle_Calc - rccu_handle->Gyro_YawAngle_Chassis;

  if(YawAngle_Diff < -180)
    YawAngle_Diff += 360;
  if(YawAngle_Diff > 180)
    YawAngle_Diff -= 360;
#endif
  rccu_handle->chassis_struct.vw = rccu_handle->YawAngle_pid.f_pid_calc( &rccu_handle->YawAngle_pid, YawAngle_Diff, 0 );
#endif
  rccu_handle->chassis_struct.gimbal_yaw_ecd_angle = 0;
  rccu_handle->chassis_struct.vx = sRemoteCtrl_Info.sPosition.vx;
  rccu_handle->chassis_struct.vy = sRemoteCtrl_Info.sPosition.vy;
}
#else
static void rccu_chassisctrl_normalmode( RCCUStruct_TypeDef* rccu_handle )
{
  //有陀螺仪的底盘控制
  float YawAngle_Diff;

  YawAngle_Diff = rccu_handle->Gyro_YawAngle_Calc - rccu_handle->ChassisCoord_CtrlStruct.soft_yaw;
  while(YawAngle_Diff < -180)
    YawAngle_Diff += 360;
  while(YawAngle_Diff > 180)
    YawAngle_Diff -= 360;

  rccu_handle->chassis_struct.vw = rccu_handle->YawAngle_pid.f_pid_calc( &rccu_handle->YawAngle_pid, YawAngle_Diff, 0 );
  rccu_handle->chassis_struct.gimbal_yaw_ecd_angle = 0;
  rccu_handle->chassis_struct.vx = sRemoteCtrl_Info.sPosition.vx;
  rccu_handle->chassis_struct.vy = sRemoteCtrl_Info.sPosition.vy;
  
    rt_kprintf(
    "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
    (int)(rccu_handle->chassis_struct.vx * 1000),
    (int)(rccu_handle->chassis_struct.vy * 1000),
    (int)(rccu_handle->chassis_struct.vw * 1000),
    (int)(rccu_handle->chassis_struct.wheel_rpm[0] * 1000),
    (int)(rccu_handle->chassis_struct.wheel_rpm[1] * 1000),
    (int)(rccu_handle->chassis_struct.wheel_rpm[2] * 1000),
    (int)(rccu_handle->chassis_struct.wheel_rpm[3] * 1000),
    (int)(rccu_handle->chassis_struct.position.position_x_mm * 1000),
    (int)(rccu_handle->chassis_struct.position.position_y_mm * 1000),
    (int)(rccu_handle->chassis_struct.position.angle_deg * 1000),
    (int)(rccu_handle->ChassisCoord_CtrlStruct.soft_x * 1000),
    (int)(rccu_handle->ChassisCoord_CtrlStruct.soft_y * 1000),
    (int)(rccu_handle->ChassisCoord_CtrlStruct.soft_yaw * 1000),      
    (int)(rccu_handle->chassis_struct.position.total_ecd[0]* 1000),
    (int)(rccu_handle->chassis_struct.position.total_ecd[1]* 1000),
    (int)(rccu_handle->chassis_struct.position.total_ecd[2]* 1000),
    (int)(rccu_handle->chassis_struct.position.total_ecd[3]* 1000)

  );
  
}
#endif
static void rccu_chassisctrl_coordmode( RCCUStruct_TypeDef* rccu_handle )
{
  float YawAngle_Diff;

//  YawAngle_Diff = rccu_handle->chassis_struct.position.angle_deg - rccu_handle->ChassisCoord_CtrlStruct.soft_yaw;
  YawAngle_Diff = rccu_handle->Gyro_YawAngle_Calc - rccu_handle->ChassisCoord_CtrlStruct.soft_yaw;
  while(YawAngle_Diff < -180)
    YawAngle_Diff += 360;
  while(YawAngle_Diff > 180)
    YawAngle_Diff -= 360;

  rccu_handle->chassis_struct.vw = rccu_handle->YawAngle_pid.f_pid_calc( &rccu_handle->YawAngle_pid, YawAngle_Diff, 0 );
  rccu_handle->chassis_struct.vx = rccu_handle->LocationX_pid.f_pid_calc( &rccu_handle->LocationX_pid, \
                                   rccu_handle->chassis_struct.position.position_x_mm, \
                                   rccu_handle->ChassisCoord_CtrlStruct.soft_x );

  rccu_handle->chassis_struct.vy = rccu_handle->LocationY_pid.f_pid_calc( &rccu_handle->LocationY_pid, \
                                   rccu_handle->chassis_struct.position.position_y_mm, \
                                   rccu_handle->ChassisCoord_CtrlStruct.soft_y );

  rt_kprintf(
    "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
    (int)(rccu_handle->chassis_struct.vx * 1000),
    (int)(rccu_handle->chassis_struct.vy * 1000),
    (int)(rccu_handle->chassis_struct.vw * 1000),
    (int)(rccu_handle->chassis_struct.wheel_rpm[0] * 1000),
    (int)(rccu_handle->chassis_struct.wheel_rpm[1] * 1000),
    (int)(rccu_handle->chassis_struct.wheel_rpm[2] * 1000),
    (int)(rccu_handle->chassis_struct.wheel_rpm[3] * 1000),
    (int)(rccu_handle->chassis_struct.position.position_x_mm * 1000),
    (int)(rccu_handle->chassis_struct.position.position_y_mm * 1000),
    (int)(rccu_handle->chassis_struct.position.angle_deg * 1000),
    (int)(rccu_handle->ChassisCoord_CtrlStruct.soft_x * 1000),
    (int)(rccu_handle->ChassisCoord_CtrlStruct.soft_y * 1000),
    (int)(rccu_handle->ChassisCoord_CtrlStruct.soft_yaw * 1000),      
    (int)(rccu_handle->chassis_struct.position.total_ecd[0]* 1000),
    (int)(rccu_handle->chassis_struct.position.total_ecd[1]* 1000),
    (int)(rccu_handle->chassis_struct.position.total_ecd[2]* 1000),
    (int)(rccu_handle->chassis_struct.position.total_ecd[3]* 1000)

  );

  /************************************方向计算********************************************/
  YawAngle_Diff = rccu_handle->Gyro_YawAngle_Coord - rccu_handle->Gyro_YawAngle_Calc;
  if(YawAngle_Diff < 0)
    YawAngle_Diff += 360;
  if(YawAngle_Diff > 360)
    YawAngle_Diff -= 360;
  rccu_handle->chassis_struct.gimbal_yaw_ecd_angle = YawAngle_Diff;
}
static void rccu_chassisctrl_trackingmode( RCCUStruct_TypeDef* rccu_handle )
{
  rccu_handle->chassis_struct.gimbal_yaw_ecd_angle = 0;
  rccu_handle->chassis_struct.vx = rccu_handle->ChassisCoord_CtrlStruct.soft_x;
  rccu_handle->chassis_struct.vy = rccu_handle->ChassisCoord_CtrlStruct.soft_y;
  rccu_handle->chassis_struct.vw = rccu_handle->ChassisCoord_CtrlStruct.soft_yaw;
}
/**
  * @brief  电机控制初始化
  * @param  NULL
  * @retval NULL
**/
static void rccu_init( RCCUStruct_TypeDef* rccu_handle )
{
  memset(rccu_handle, 0, sizeof(RCCUStruct_TypeDef));
  rccu_setmode( rccu_handle, CHASSIS_RELAX );
  rccu_handle->qGyro_YawAngle_New = &GyroData_Struct.Yaw;
  Chassis_Init( &rccu_handle->chassis_struct,
                FOUR_DRIVE_McNamara,
                RC_CHASSIS_MAX_SPEED_X,   //最大X轴速度
                RC_CHASSIS_MAX_SPEED_Y,   //最大Y轴速度
                RC_CHASSIS_MAX_SPEED_R,   //最大中心旋转轴速度
                MAX_WHEEL_RPM );          //轮子最大速度
  //YAW PID
  PID_struct_init(&rccu_handle->YawAngle_pid, POSITION_PID, RC_CHASSIS_MAX_SPEED_R, 3.0f, De_YAW_KP, De_YAW_KI, De_YAW_KD);
  rccu_handle->YawAngle_pid.output_deadband = 0;
  //X PID
  PID_struct_init(&rccu_handle->LocationX_pid, POSITION_PID, RC_CHASSIS_MAX_SPEED_X, 10.0f, De_LOCX_KP, De_LOCX_KI, De_LOCX_KD);
  rccu_handle->LocationX_pid.output_deadband = 0;
  //Y PID
  PID_struct_init(&rccu_handle->LocationY_pid, POSITION_PID, RC_CHASSIS_MAX_SPEED_Y, 10.0f, De_LOCY_KP, De_LOCY_KI, De_LOCY_KD);
  rccu_handle->LocationY_pid.output_deadband = 0;
  //YAW 跟随器
  Location_Tracker_Init( &rccu_handle->ChassisCoord_CtrlStruct.Yaw_Tracker_Struct,
                         CONTROL_FREQ_HZ,
                         RC_CHASSIS_MAX_SPEED_R,
                         RC_CHASSIS_MAX_SPEED_R * 2 / 3,
                         RC_CHASSIS_MAX_SPEED_R * 2 / 3,
                         RC_CHASSIS_MAX_SPEED_R );
  //X 跟随器
  Location_Tracker_Init( &rccu_handle->ChassisCoord_CtrlStruct.X_Tracker_Struct,
                         CONTROL_FREQ_HZ,
                         RC_CHASSIS_MAX_SPEED_X,
                         RC_CHASSIS_MAX_SPEED_X / 4,
                         RC_CHASSIS_MAX_SPEED_X / 4,
                         RC_CHASSIS_MAX_SPEED_X );
  //Y 跟随器
  Location_Tracker_Init( &rccu_handle->ChassisCoord_CtrlStruct.Y_Tracker_Struct,
                         CONTROL_FREQ_HZ,
                         RC_CHASSIS_MAX_SPEED_Y,
                         RC_CHASSIS_MAX_SPEED_Y / 4,
                         RC_CHASSIS_MAX_SPEED_Y / 4,
                         RC_CHASSIS_MAX_SPEED_Y );
  //循迹跟随器
  LineTracker_Init( 1,
                    CONTROL_FREQ_HZ,
                    Tracking_Device1.Tracking_UploadADCData.DATE.adc_val,   //前
                    Tracking_Device2.Tracking_UploadADCData.DATE.adc_val,   //左
                    Tracking_Device3.Tracking_UploadADCData.DATE.adc_val,   //后
                    Tracking_Device4.Tracking_UploadADCData.DATE.adc_val ); //右
}
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#ifdef __SLAVE_SteeringEngine_6CH_H__
/*******舵机控制*******/
void ServoSetPluseAndTime(uint8_t mode, uint16_t pwmval, uint16_t time)
{
  if(mode >= SERVO_NUM_MAX)
    mode = SERVO_NUM_MAX - 1;
  if(pwmval < 500)
    pwmval = 500;
  else if(pwmval > 2500)
    pwmval = 2500;
  if(time < 20)
    time = 20;
  else if(time > 30000)
    time = 30000;
  AngleTime[mode] = time;
  AngleValue[mode] = pwmval;
}
void SCARA_CoordinateCtrl(float Cartesian_X, float Cartesian_Y, float Cartesian_Z)
{
  SCARA_S.cartesian[0] = Cartesian_X;
  SCARA_S.cartesian[1] = Cartesian_Y;
  SCARA_S.cartesian[2] = Cartesian_Z;
  calculate_delta(SCARA_S.cartesian);
  Servo_AngleToPWM( SCARA_S.Angle, &AngleValue[SERVO_ARM1], &AngleValue[SERVO_ARM2], &AngleValue[SERVO_YAW] );
}
#else
void ServoSetPluseAndTime(uint8_t mode, uint16_t pwmval, uint16_t time)
{
  if(mode >= SERVO_CH_NUM)
    mode = SERVO_CH_NUM - 1;
  if(pwmval < 500)
    pwmval = 500;
  else if(pwmval > 2500)
    pwmval = 2500;
  if(time < 20)
    time = 20;
  else if(time > 30000)
    time = 30000;
  RobotArmData_Struct.ServoPwmDuty[mode] = time;
  RobotArmData_Struct.ServoTime[mode] = pwmval;
}
#endif
void ChassisCoord_Set(float _x_diff, float _y_diff, float _yaw_diff)
{
  if( rccu_struct.mode_order != CHASSIS_COORD )
  {
    rccu_struct.mode_order = CHASSIS_COORD;
    My_mDelay(100);
  }
  rccu_struct.ChassisCoord_CtrlStruct.goal_y += _y_diff;
  rccu_struct.ChassisCoord_CtrlStruct.goal_x += _x_diff;
  rccu_struct.ChassisCoord_CtrlStruct.goal_yaw += _yaw_diff;
#if 0
  while( ( rccu_struct.ChassisCoord_CtrlStruct.goal_yaw < 0 ) || \
         ( rccu_struct.ChassisCoord_CtrlStruct.goal_yaw > 360 ) )
  {
    if(rccu_struct.ChassisCoord_CtrlStruct.goal_yaw < 0)
      rccu_struct.ChassisCoord_CtrlStruct.goal_yaw += 360;
    if(rccu_struct.ChassisCoord_CtrlStruct.goal_yaw >= 360)
      rccu_struct.ChassisCoord_CtrlStruct.goal_yaw -= 360;
  }
#endif
}
void ChassisCoord_WaitStop(void)
{
  while( ( ABS(rccu_struct.ChassisCoord_CtrlStruct.soft_y - rccu_struct.ChassisCoord_CtrlStruct.goal_y) > 1.0f ) || \
         ( ABS(rccu_struct.LocationY_pid.err[NOW]) > 10 ) || \
         ( ABS(rccu_struct.ChassisCoord_CtrlStruct.soft_x - rccu_struct.ChassisCoord_CtrlStruct.goal_x) > 1.0f ) || \
         ( ABS(rccu_struct.LocationX_pid.err[NOW]) > 10 ) || \
         ( ABS(rccu_struct.ChassisCoord_CtrlStruct.soft_yaw - rccu_struct.ChassisCoord_CtrlStruct.goal_yaw) > 0.5f) || \
         ( ABS(rccu_struct.YawAngle_pid.err[NOW]) > 0.5f ) )
  {
    My_mDelay(10);
  }
}
void ChassisCoord_WaitYawStop(void)
{
  while( ( ABS(rccu_struct.ChassisCoord_CtrlStruct.soft_yaw - rccu_struct.ChassisCoord_CtrlStruct.goal_yaw) > 1.3f) || \
         ( ABS(rccu_struct.YawAngle_pid.err[NOW]) > 0.5f )
       )
  {
    My_mDelay(10);
  }

}
void ChassisSpeed_WaitCarToStop(void)
{
  while(  rccu_struct.mode_order != CHASSIS_STOP  )
    LINE_PATROL_DELAY(2);
}
void ChassisSpeed_Set(float _x_spd, float _y_spd)
{
  sRemoteCtrl_Info.sPosition.vx = 0;
  sRemoteCtrl_Info.sPosition.vy = 0;
  if( rccu_struct.mode_order != CHASSIS_NORMAL )
  {
    rccu_struct.mode_order = CHASSIS_NORMAL;
    My_mDelay(100);
  }
  sRemoteCtrl_Info.sPosition.vx = _x_spd;
  sRemoteCtrl_Info.sPosition.vy = _y_spd;
}

void ChassisSpeed_Set1(float _x_spd, float _y_spd, float _yaw)
{
  if( rccu_struct.mode_order != CHASSIS_NORMAL )
  {
    rccu_struct.mode_order = CHASSIS_NORMAL;
    My_mDelay(100);
  }
  sRemoteCtrl_Info.sPosition.vx = _x_spd;
  sRemoteCtrl_Info.sPosition.vy = _y_spd;
  rccu_struct.ChassisCoord_CtrlStruct.soft_yaw = _yaw;
}
void rccu_setmode_to_tracking( void )
{
  rccu_struct.mode_order = CHASSIS_TRACKING;
}
void rccu_setmode_to_coord( void )
{
  rccu_struct.mode_order = CHASSIS_COORD;
}
void rccu_setmode_to_normal( void )
{
  rccu_struct.mode_order = CHASSIS_NORMAL;
}
int32_t Read_Position_x_mm(void)
{
  return rccu_struct.chassis_struct.position.position_x_mm;
}
int32_t Read_Position_y_mm(void)
{
  return rccu_struct.chassis_struct.position.position_y_mm;
}
int32_t Read_Angle_deg(void)
{
  return rccu_struct.chassis_struct.position.angle_deg;
}
void rccu_task(void *pvParameters)
{
  //延时等待车子稳定
  My_mDelay(500);
  //读取当前参数
  rccu_struct.chassis_struct.position.position_x_mm = 0;
  rccu_struct.chassis_struct.position.position_y_mm = 0;
  rccu_struct.ChassisCoord_CtrlStruct.goal_x = 0;
  rccu_struct.ChassisCoord_CtrlStruct.goal_y = 0;
  rccu_struct.Gyro_YawAngle_zero = Read_RealYawAngle(&rccu_struct);
  rccu_struct.Gyro_YawAngle_Calc = Read_GyroYawAngleCalc(&rccu_struct);
  rccu_struct.Gyro_YawAngle_Chassis = rccu_struct.Gyro_YawAngle_Calc;
  rccu_struct.Gyro_YawAngle_Coord = rccu_struct.Gyro_YawAngle_Calc;

  rccu_setmode( &rccu_struct, CHASSIS_RELAX );
//	rccu_setmode( &rccu_struct, CHASSIS_NORMAL );
#ifdef __SLAVE_SteeringEngine_6CH_H__
  SCARA_CoordinateCtrl(SCARA_S.cartesian[0], SCARA_S.cartesian[1], SCARA_S.cartesian[2]);
#endif

  Task_User_create();

  while(1)
  {
    /************************ 数据采集 ************************************/
    /************************ 数据采集 ************************************/
    rccu_struct.Gyro_YawAngle_Calc = Read_GyroYawAngleCalc(&rccu_struct);
    /************************ 运动控制 ************************************/
    /************************ 运动控制 ************************************/
    switch ( rccu_struct.mode_run )
    {
    case CHASSIS_RELAX:
    case CHASSIS_STOP:
    {
      //停止
      rccu_struct.chassis_struct.vx = 0;
      rccu_struct.chassis_struct.vy = 0;
      rccu_struct.chassis_struct.vw = 0;
    }
    break;
    case CHASSIS_NORMAL:
    {
      //常规模式
      rccu_chassisctrl_normalmode(&rccu_struct);
    }
    break;
    case CHASSIS_COORD:
    {
      //坐标模式
      rccu_chassisctrl_coordmode(&rccu_struct);
    }
    break;
    case CHASSIS_TRACKING:
    {
      //循迹模式
      rccu_chassisctrl_trackingmode(&rccu_struct);
    }
    break;
    default:
      break;
    }
    rccu_chassisctrl( &rccu_struct );
    /************************ 模式变更 ************************************/
    /************************ 模式变更 ************************************/
    if(rccu_struct.mode_run != rccu_struct.mode_order)
    {
      //变更
      rccu_struct.mode_run = rccu_struct.mode_order;
      rccu_struct.Gyro_YawAngle_Calc = Read_GyroYawAngleCalc(&rccu_struct);
      switch (rccu_struct.mode_run)
      {
      case CHASSIS_RELAX:
      case CHASSIS_STOP:
      case CHASSIS_NORMAL:  //遥控
        rccu_struct.Gyro_YawAngle_Chassis = rccu_struct.Gyro_YawAngle_Calc;
        rccu_struct.ChassisCoord_CtrlStruct.soft_yaw = rccu_struct.Gyro_YawAngle_Chassis;
        rccu_struct.ChassisCoord_CtrlStruct.goal_yaw = rccu_struct.ChassisCoord_CtrlStruct.soft_yaw;
        break;
      case CHASSIS_COORD:   //坐标模式
        rccu_struct.ChassisCoord_CtrlStruct.soft_yaw = rccu_struct.Gyro_YawAngle_Calc;
        rccu_struct.ChassisCoord_CtrlStruct.goal_yaw = rccu_struct.ChassisCoord_CtrlStruct.soft_yaw;
        Location_Tracker_NewTask( &rccu_struct.ChassisCoord_CtrlStruct.Yaw_Tracker_Struct, \
                                  rccu_struct.ChassisCoord_CtrlStruct.soft_yaw, \
                                  0 );
        rccu_struct.ChassisCoord_CtrlStruct.soft_x = rccu_struct.chassis_struct.position.position_x_mm;
        rccu_struct.ChassisCoord_CtrlStruct.goal_x = rccu_struct.ChassisCoord_CtrlStruct.soft_x;
        Location_Tracker_NewTask( &rccu_struct.ChassisCoord_CtrlStruct.X_Tracker_Struct, \
                                  rccu_struct.ChassisCoord_CtrlStruct.soft_x, \
                                  0 );
        rccu_struct.ChassisCoord_CtrlStruct.soft_y = rccu_struct.chassis_struct.position.position_y_mm;
        rccu_struct.ChassisCoord_CtrlStruct.goal_y = rccu_struct.ChassisCoord_CtrlStruct.soft_y;
        Location_Tracker_NewTask( &rccu_struct.ChassisCoord_CtrlStruct.Y_Tracker_Struct, \
                                  rccu_struct.ChassisCoord_CtrlStruct.soft_y, \
                                  0 );
        break;
      default:
        break;
      }
    }
    /************************ 软目标提取 **********************************/
    /************************ 软目标提取 **********************************/
    switch (rccu_struct.mode_run)
    {
    case CHASSIS_COORD://坐标模式
      Location_Tracker_Capture_Goal( &rccu_struct.ChassisCoord_CtrlStruct.Yaw_Tracker_Struct, \
                                     rccu_struct.ChassisCoord_CtrlStruct.goal_yaw );
      if(ABS(rccu_struct.ChassisCoord_CtrlStruct.soft_yaw - rccu_struct.ChassisCoord_CtrlStruct.goal_yaw) < 1.0f)
        rccu_struct.ChassisCoord_CtrlStruct.soft_yaw = rccu_struct.ChassisCoord_CtrlStruct.goal_yaw;
      else
        rccu_struct.ChassisCoord_CtrlStruct.soft_yaw = rccu_struct.ChassisCoord_CtrlStruct.Yaw_Tracker_Struct.go_location;
      Location_Tracker_Capture_Goal( &rccu_struct.ChassisCoord_CtrlStruct.X_Tracker_Struct, \
                                     rccu_struct.ChassisCoord_CtrlStruct.goal_x );
      rccu_struct.ChassisCoord_CtrlStruct.soft_x = rccu_struct.ChassisCoord_CtrlStruct.X_Tracker_Struct.go_location;

      Location_Tracker_Capture_Goal( &rccu_struct.ChassisCoord_CtrlStruct.Y_Tracker_Struct, \
                                     rccu_struct.ChassisCoord_CtrlStruct.goal_y );
      rccu_struct.ChassisCoord_CtrlStruct.soft_y = rccu_struct.ChassisCoord_CtrlStruct.Y_Tracker_Struct.go_location;
      break;
    case CHASSIS_TRACKING:
      LineTracker_Scan();
      rccu_struct.ChassisCoord_CtrlStruct.soft_yaw = LineTracker_Struct.yaw_PracticalOut;
      rccu_struct.ChassisCoord_CtrlStruct.soft_x = LineTracker_Struct.x_axis_PracticalOut;
      rccu_struct.ChassisCoord_CtrlStruct.soft_y = LineTracker_Struct.y_axis_PracticalOut;
    default:
      break;
    }
#ifdef __SLAVE_SteeringEngine_6CH_H__
    if( ( AppData_Struct.x != SCARA_S.cartesian[0] ) ||
        ( AppData_Struct.y != SCARA_S.cartesian[1] ) ||
        ( AppData_Struct.z != SCARA_S.cartesian[2] ) )
    {
      SCARA_CoordinateCtrl(AppData_Struct.x, AppData_Struct.y, AppData_Struct.z);
    }
#endif
    My_mDelay( 9 );
  }
}
int Task_RCCU_create(void)
{
  rccu_init( &rccu_struct );

  thread_rccu = rt_thread_create("rccu",            /* 线程名字 */
                                 rccu_task,         /* 线程入口函数 */
                                 RT_NULL,           /* 线程入口函数参数 */
                                 1024,              /* 线程栈大小 */
                                 2,                 /* 线程的优先级 */
                                 20);               /* 线程时间片 */
  if(thread_rccu != RT_NULL)
  {
    rt_thread_startup(thread_rccu);
    rt_kprintf("thread_rccu startup!\n");
  }
  return 0;
}
INIT_APP_EXPORT(Task_RCCU_create);
