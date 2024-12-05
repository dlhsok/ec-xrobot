/* Includes ------------------------------------------------------------------*/
#include "thread_arm.h"
#include "bsp.h"
#include "data.h"
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
/* Private types -------------------------------------------------------------*/
//typedef enum
//{
//  CHASSIS_RELAX = 0,          //安全模式
//  CHASSIS_STOP,               //底盘停止
//  CHASSIS_NORMAL,             //底盘正常模式（带陀螺仪）
//  CHASSIS_COORD,              //底盘坐标模式
//  CHASSIS_TRACKING,           //底盘循迹模式
//} ArmCtrlMode_TypeDef;
typedef struct
{
  //硬目标位置和速度
  float		goal_x;
  float		goal_y;
  //软目标位置和速度
  float		soft_x;
  float		soft_y;

  Location_Tracker_Typedef X_Tracker_Struct;
  Location_Tracker_Typedef Y_Tracker_Struct;
} ArmCoord_CtrlTypeDef;

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 定义线程控制块指针 */
rt_thread_t thread_arm = RT_NULL;
ArmCoord_CtrlTypeDef ArmCoord_CtrlStruct = {0};

/**
  * @brief  电机控制初始化
  * @param  NULL
  * @retval NULL
**/
static void arm_init()
{
  //X 跟随器
  Location_Tracker_Init( &ArmCoord_CtrlStruct.X_Tracker_Struct,
                         CONTROL_FREQ_HZ,
                         DEVICE_ROBOTARM_MAX_SPEED_X*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO,
                         DEVICE_ROBOTARM_MAX_SPEED_X*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO*5.0f,
                         DEVICE_ROBOTARM_MAX_SPEED_X*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO*5.0f,
                         DEVICE_ROBOTARM_MAX_SPEED_X*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO / 4);
  //Y 跟随器
  Location_Tracker_Init( &ArmCoord_CtrlStruct.Y_Tracker_Struct,
                         CONTROL_FREQ_HZ,
                         DEVICE_ROBOTARM_MAX_SPEED_Y*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO,
                         DEVICE_ROBOTARM_MAX_SPEED_Y*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO*5.0f,
                         DEVICE_ROBOTARM_MAX_SPEED_Y*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO*5.0f,
                         DEVICE_ROBOTARM_MAX_SPEED_Y*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO / 4 );
}

int arm_cnt = 0;
void arm_task(void *pvParameters)
{
  //延时等待车子稳定
  My_mDelay(500);
  //读取当前参数

  ArmCoord_CtrlStruct.goal_x = RobotArmData_Struct.SCARA_Cartesian[0]*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO;
  ArmCoord_CtrlStruct.goal_y = RobotArmData_Struct.SCARA_Cartesian[1]*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO;
  ArmCoord_CtrlStruct.soft_x = RobotArmData_Struct.SCARA_Cartesian[0]*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO;
  ArmCoord_CtrlStruct.soft_y = RobotArmData_Struct.SCARA_Cartesian[1]*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO;
  Location_Tracker_NewTask( &ArmCoord_CtrlStruct.X_Tracker_Struct, \
                            ArmCoord_CtrlStruct.soft_x, \
                            0 );
  Location_Tracker_NewTask( &ArmCoord_CtrlStruct.Y_Tracker_Struct, \
                            ArmCoord_CtrlStruct.soft_y, \
                            0 );
  while(1)
    {

      Location_Tracker_Capture_Goal( &ArmCoord_CtrlStruct.X_Tracker_Struct, \
                                     ArmCoord_CtrlStruct.goal_x );
      ArmCoord_CtrlStruct.soft_x = ArmCoord_CtrlStruct.X_Tracker_Struct.go_location;

      Location_Tracker_Capture_Goal( &ArmCoord_CtrlStruct.Y_Tracker_Struct, \
                                     ArmCoord_CtrlStruct.goal_y );
      ArmCoord_CtrlStruct.soft_y = ArmCoord_CtrlStruct.Y_Tracker_Struct.go_location;

      RobotArmData_Struct.SCARA_Cartesian[0] = 1.0f * ArmCoord_CtrlStruct.soft_x / DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO;
      RobotArmData_Struct.SCARA_Cartesian[1] = 1.0f * ArmCoord_CtrlStruct.soft_y / DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO;

      My_mDelay(10);
      arm_cnt++;
    }
}

void ArmCoord_SetAbsolute(float _x, float _y)
{
  ArmCoord_CtrlStruct.goal_x = _x*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO;
  ArmCoord_CtrlStruct.goal_y = _y*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO;
}

void RobotArm_WaitStop(void)
{
  //等待机械臂运动停止
  while( 
//		ABS((ArmCoord_CtrlStruct.soft_y - ArmCoord_CtrlStruct.goal_y) > 0.2f*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO ) || 
//		ABS((ArmCoord_CtrlStruct.soft_x - ArmCoord_CtrlStruct.goal_x) > 0.2f*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO ) || 
		( RobotArmData_Struct.UploadData_U.bit.Motor1_RunSta == 1 )
  || ( RobotArmData_Struct.UploadData_U.bit.Motor2_RunSta == 1 )
  || ( RobotArmData_Struct.UploadData_U.bit.Motor3_RunSta == 1 ) 
	)
    {
      My_mDelay(10);
    }
}

int Task_Arm_create(void)
{
  arm_init();

  thread_arm = rt_thread_create("arm",            /* 线程名字 */
                                arm_task,         /* 线程入口函数 */
                                RT_NULL,           /* 线程入口函数参数 */
                                1024,              /* 线程栈大小 */
                                3,                 /* 线程的优先级 */
                                20);               /* 线程时间片 */
  if(thread_arm != RT_NULL)
    {
      rt_thread_startup(thread_arm);
      rt_kprintf("thread_arm startup!\n");
    }
  return 0;
}


//INIT_APP_EXPORT(Task_Arm_create);
