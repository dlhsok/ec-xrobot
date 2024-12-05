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
//����Ƶ��_hz
#define CONTROL_FREQ_HZ			  (100)
/* Private types -------------------------------------------------------------*/
//typedef enum
//{
//  CHASSIS_RELAX = 0,          //��ȫģʽ
//  CHASSIS_STOP,               //����ֹͣ
//  CHASSIS_NORMAL,             //��������ģʽ���������ǣ�
//  CHASSIS_COORD,              //��������ģʽ
//  CHASSIS_TRACKING,           //����ѭ��ģʽ
//} ArmCtrlMode_TypeDef;
typedef struct
{
  //ӲĿ��λ�ú��ٶ�
  float		goal_x;
  float		goal_y;
  //��Ŀ��λ�ú��ٶ�
  float		soft_x;
  float		soft_y;

  Location_Tracker_Typedef X_Tracker_Struct;
  Location_Tracker_Typedef Y_Tracker_Struct;
} ArmCoord_CtrlTypeDef;

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* �����߳̿��ƿ�ָ�� */
rt_thread_t thread_arm = RT_NULL;
ArmCoord_CtrlTypeDef ArmCoord_CtrlStruct = {0};

/**
  * @brief  ������Ƴ�ʼ��
  * @param  NULL
  * @retval NULL
**/
static void arm_init()
{
  //X ������
  Location_Tracker_Init( &ArmCoord_CtrlStruct.X_Tracker_Struct,
                         CONTROL_FREQ_HZ,
                         DEVICE_ROBOTARM_MAX_SPEED_X*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO,
                         DEVICE_ROBOTARM_MAX_SPEED_X*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO*5.0f,
                         DEVICE_ROBOTARM_MAX_SPEED_X*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO*5.0f,
                         DEVICE_ROBOTARM_MAX_SPEED_X*DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO / 4);
  //Y ������
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
  //��ʱ�ȴ������ȶ�
  My_mDelay(500);
  //��ȡ��ǰ����

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
  //�ȴ���е���˶�ֹͣ
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

  thread_arm = rt_thread_create("arm",            /* �߳����� */
                                arm_task,         /* �߳���ں��� */
                                RT_NULL,           /* �߳���ں������� */
                                1024,              /* �߳�ջ��С */
                                3,                 /* �̵߳����ȼ� */
                                20);               /* �߳�ʱ��Ƭ */
  if(thread_arm != RT_NULL)
    {
      rt_thread_startup(thread_arm);
      rt_kprintf("thread_arm startup!\n");
    }
  return 0;
}


//INIT_APP_EXPORT(Task_Arm_create);
