/* Includes ------------------------------------------------------------------*/
#include "thread_user.h"
/* �����������ļ� */
#include "data.h"
/* ��������ͷ�ļ� */
#include "thread_rccu.h"
#include "thread_gui.h"
#include "chassis_LineTracker.h"
/* Private macros ------------------------------------------------------------*/
#define BRICK_NUM    4 //ץש����
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* �����߳̿��ƿ�ָ�� */
rt_thread_t thread_PathWrite = RT_NULL;
/*
*/
/***************************************************************/
//�¼������������

#define Obstacle_width  1000
#define Obstacle_length 1500  //���Ա�ԭ����󣬳����������һ�ξ���
#define Brick_Size 80
void CircleRecognitionModule_Start(RecognitionModule_s *hand);
void RobotArmGetblock(uint8_t color);
void RobotArmDown(uint8_t color);
uint8_t Find_Obstacle(DirectionDef_e Car_Direction);
void Avoid_Obstacle(DirectionDef_e Car_Direction, uint8_t Car_side);
uint8_t LineTrack_Scan(DirectionDef_e Car_Direction, uint8_t *Count);
void LineSingle_Tracker(uint8_t Line_TrackScan, DirectionDef_e Car_Direction);
uint16_t FindEnd_Head(void);
uint16_t FindEnd_Tail(void);
uint16_t FindHead_Obstacle(void);
uint16_t FindEnd_Obstacle(void);
uint8_t color_haveget[3] = {0, 0, 0}; //��Ӧcolor_e���������
uint8_t Car_position = 0, run_num = 0;
//RobotArmData_Struct.ServoPwmDuty[1]   1800  		1420 			1100
uint16_t ServoPwmDuty[3] = {1800, 1500, 1100};
int16_t position_Cartesian[3][3] = {{400, -140, -50}, {410, 20, -60}, {420, 280, -50}};
int16_t position_Cartesian1[3][3] = {{400, -140, -130}, {410, 20, -130}, {420, 280, -80}};
typedef enum
{
  color_green = 0,
  color_red,
  color_blue,
} color_e;
uint8_t color_set[] = {color_red, color_green, color_blue, color_green, color_blue, color_red};
/***************************************************************/


int32_t PositionXmm_Old;
int32_t PositionYmm_Old;
int32_t PositionXmm_Diff;
int32_t PositionYmm_Diff;
__IO uint16_t run_cnt = 0;
/* Private functions ---------------------------------------------------------*/
static void Debug_Await(void)
{
  while( (KEY_4() == 0) || (lcd_page != 1))
    My_mDelay(50);
  while( (KEY_4() == 1) || (lcd_page != 1))
    My_mDelay(50);
}
static void RobotArm_WaitStop(void)
{
  //�ȴ���е���˶�ֹͣ
  while( ( RobotArmData_Struct.UploadData_U.bit.Motor1_RunSta == 1 )
         || ( RobotArmData_Struct.UploadData_U.bit.Motor2_RunSta == 1 )
         || ( RobotArmData_Struct.UploadData_U.bit.Motor3_RunSta == 1 ) )
  {
    My_mDelay(10);
  }
}
static void RobotArm_Rst(void)
{
  while( ( RobotArmData_Struct.UploadData_U.bit.IN1 == 1 )
         || ( RobotArmData_Struct.UploadData_U.bit.IN2 == 1 )
         || ( RobotArmData_Struct.UploadData_U.bit.IN3 == 1 ) )
  {
    RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 0;
    RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 0;
    RobotArmData_Struct.SCARAflg_U.bit.SCARAReset_Enabled = 1;
    My_mDelay(500);
    RobotArmData_Struct.SCARAflg_U.bit.SCARAReset_Enabled = 0;
    My_mDelay(500);
    RobotArm_WaitStop();
  }
}
static void Recognition_Start(void)
{
  //��ʼ����ʶ��
  bool_recognitionflag = 0;
  RecognitionModule_Start(&RecognitionModule_t);
  while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
  {
    if(RecognitionModule_t.RecognitionModuleSte == RM_error)
    {
      //ʶ��ʧ��
      if(RecognitionModule_t.err == ERR_disconnect)
      {
        My_mDelay(50);
        RecognitionModule_Start(&RecognitionModule_t);
      }
    }
    else if(RecognitionModule_t.RecognitionModuleSte == RM_Identify)
    {
      break;
    }
    My_mDelay(10);
  }

}
static void Rotation_Claw(int16_t _angle_x10)
{
  //��еצ��Ӧש��Ƕ�
  int32_t set_angle;
  int16_t _angle_pwm;

  if(_angle_x10 < -900)
    _angle_x10 += 1800;
  else if(_angle_x10 > 900)
    _angle_x10 -= 1800;

  _angle_pwm = (_angle_x10 * 20 / 18);
  set_angle = Rotation_Claw_Init + _angle_pwm;
  if(set_angle < 500)
    set_angle = 2000 + set_angle;
  else if(set_angle > 2500)
    set_angle = set_angle - 2000;

  RobotArmData_Struct.ServoPwmDuty[1] = set_angle;
}
#define Brick_XY_Debug 0
#if Brick_XY_Debug == 1
uint8_t test = 0;
float set_loc_x = 30;
float set_loc_y = -30;
uint16_t IdentifyBrick_Get_cnt = 0;
#endif


//�Ӿ�����(-100,-10)-->(-25,-10) ��ǰ�н�   IdentifyBrick_Get(-10,-25);
//�Ӿ�����(-25,60)--> (-25,-10)  �����н�   IdentifyBrick_Get(-10,-25);
static void IdentifyBrick_Get(float SET_LOCATION_X, float SET_LOCATION_Y)
{
  //ʶ��ש������
  float y_err[2] = {0, 0};
  float x_err[2] = {0, 0};
  uint8_t timeout = 0;
#if 1
#define SPEED_MAX 35
  float yout = 0, xout = 0;

  while(1)
  {
#if Brick_XY_Debug == 1
    IdentifyBrick_Get_cnt++;
    SET_LOCATION_X = set_loc_x;
    SET_LOCATION_Y = set_loc_y;
#endif
    if(RecognitionModule_t.RecognitionModuleSte == RM_succeed)
    {
      x_err[1] = x_err[0];
      x_err[0] = SET_LOCATION_X - BrickData_Struct.y;
      xout = ( 0.8f * x_err[0] ) + ( 0.5f * ( x_err[0] - x_err[1] ) );
      if( xout > SPEED_MAX ) xout = SPEED_MAX;
      else if( xout < -SPEED_MAX )xout = -SPEED_MAX;

      y_err[1] = y_err[0];
      y_err[0] = SET_LOCATION_Y - BrickData_Struct.x;
      yout = ( 0.8f * y_err[0] ) + ( 0.5f * ( y_err[0] - y_err[1] ) );
      if( yout > SPEED_MAX ) yout = SPEED_MAX;
      else if( yout < -SPEED_MAX )yout = -SPEED_MAX;
#if Brick_XY_Debug == 1
      if(test == 1)
        ChassisSpeed_Set(0, 0);
      else
#endif
        if( ( ABS(y_err[0]) < 10 ) && ( ABS(x_err[0]) < 10 ) )
        {
          ChassisSpeed_Set(0, 0);
#if Brick_XY_Debug == 0
          break;
#endif
        }
        else
        {
          ChassisSpeed_Set(-xout, yout);
        }
      timeout = 0;
      RecognitionModule_t.RecognitionModuleSte = RM_Identify;
    }
    else
#if Brick_XY_Debug == 1
      if (test == 0)
#endif
      {
        timeout++;
        if(timeout > 200)
        {
          timeout = 0;
          ChassisCoord_Set(100, 0, 0);
          ChassisCoord_WaitStop();
          ChassisSpeed_Set(0, 0);
        }
      }
    My_mDelay(25);
  }
  My_mDelay(2000);
  RecognitionModule_t.RecognitionModuleSte = RM_Identify;
  while(1)
  {
    if(RecognitionModule_t.RecognitionModuleSte == RM_succeed)
    {
      x_err[0] = SET_LOCATION_X - BrickData_Struct.y;
      y_err[0] = SET_LOCATION_Y - BrickData_Struct.x ;
      // x_err[0] = SET_LOCATION_X - BrickData_Struct.x;
      // y_err[0] = BrickData_Struct.y - SET_LOCATION_Y;
      ChassisCoord_Set(-x_err[0], y_err[0], 0);
      ChassisCoord_WaitStop();
      ChassisSpeed_Set(0, 0);
      break;
    }
  }
#else
  while(1)
  {
    if(RecognitionModule_t.RecognitionModuleSte == RM_succeed)
    {
      x_err[0] = SET_LOCATION_X - BrickData_Struct.x;
      y_err[0] = BrickData_Struct.y - SET_LOCATION_Y;
#if 1
      if( ( ABS(y_err[0]) <= 10 ) && ( ABS(x_err[0]) <= 10 ) )
      {
        ChassisSpeed_Set(0, 0);
        break;
      }
      else
      {
        ChassisCoord_Set(y_err[0], x_err[0], 0);
        ChassisCoord_WaitStop();
        My_mDelay(2000);
      }
      RecognitionModule_t.RecognitionModuleSte = RM_Identify;
#else
      ChassisCoord_Set(y_err[0], x_err[0], 0);
      ChassisCoord_WaitStop();
      ChassisSpeed_Set(0, 0);
      break;
#endif
      timeout = 0;
    }
    else
    {
      timeout++;
      if(timeout >= 100)
      {
        ChassisCoord_Set(-100, 0, 0);
        ChassisCoord_WaitStop();
        My_mDelay(1000);
        timeout = 0;
      }
      My_mDelay(10);
    }
  }
#endif
}
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
uint8_t time_cnt = 0, Line_Count = 0;;
uint8_t Line_ScanNum = 0;
void PathWrite_task(void *pvParameters)
{
  //·���滮����
  RobotArm_Rst();
  RecognitionModule_Stop(&RecognitionModule_t);

  while( (KEY_4() == 1) || (lcd_page != 1))
    My_mDelay(10);
  while( (KEY_4() == 0) || (lcd_page != 1))
    My_mDelay(10);






#if 1 //
  while(1)
  {

//    RobotArmData_Struct.MotorAngle_Target[2] = -180;
//    RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 1;
//    My_mDelay(500);
//    RobotArm_WaitStop();
//    My_mDelay(1500);
//    RobotArmData_Struct.SCARA_Cartesian[0] = -316.7f;
//    RobotArmData_Struct.SCARA_Cartesian[1] = .0f;
//    RobotArmData_Struct.SCARA_Cartesian[2] = 220.f;
//    RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 1;
//    RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 1;
//    My_mDelay(500);
//    RobotArm_WaitStop();
//    My_mDelay(1500);
//    ChassisCoord_Set(0, 1000, 0);
//    ChassisCoord_WaitStop();
//    My_mDelay(2000);
//    ChassisCoord_Set(0, -1000, 0);
//    ChassisCoord_WaitStop();
//    My_mDelay(2000);
//    ChassisCoord_Set(0, 1000, 0);
//    ChassisCoord_WaitStop();
//    My_mDelay(2000);
//    ChassisCoord_Set(0, -1000, 0);
//    ChassisCoord_WaitStop();
//    My_mDelay(2000);
    Debug_Await();
  }
#endif







#if 0//
  while(1)
  {
    //���뵽ѭ��ģʽ
    rccu_setmode_to_tracking();
    //��ʱʹ�����л�����rccu������е���ģʽ�л�����
    My_mDelay(50);
    //��ͷΪ������250���ٶ�ѭ��2���ߣ������ڳ���ʱֹͣ����ʹ��ѭ������
    LineTracker_Execute_LineNum(CarDirection_Head, 250, 1, 1, 0);
    //��е�۵��̽Ƕ�����Ϊ-180
    RobotArmData_Struct.MotorAngle_Target[2] = -180;
    //�Ƕ�ʹ��
    RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 1;
    //��ʱ500ʹMODBUSͨѶ���԰����ݴ����ȥ
    My_mDelay(500);
    //��ͷΪ������250���ٶ�ѭ��2���ߣ������ڳ���ʱֹͣ��ʹ��ѭ������
    LineTracker_Execute_LineNum(CarDirection_Head, 250, 1, 1, 1);
    LineTracker_Execute_SituAdjust(CarDirection_Head, 1, 1000);
    //����Ϊ������150���ٶ�ѭ��1���ߣ������ڳ��ո�ѭ������ֹͣ��ʹ��ѭ������
    LineTracker_Execute_LineNum(CarDirection_Left, 150, 1, 0, 1);
    for(run_cnt = 0; run_cnt < BRICK_NUM; run_cnt++)
    {
      LineTracker_WaitCarToStop();
      My_mDelay(2000);
      RobotArm_WaitStop();
      My_mDelay(500);
      PositionXmm_Old = Read_Position_x_mm();
      PositionYmm_Old = Read_Position_y_mm();
      Recognition_Start();
      IdentifyBrick_Get(-20, -20); //ʶ��שλ�ý��бջ�����С������λ��
      My_mDelay(500);
      RecognitionModule_Stop(&RecognitionModule_t);
      Rotation_Claw(BrickData_Struct.yaw);
//		RobotArmData_Struct.ServoPwmDuty[0] = Claw_S_MAX;
      RobotArmData_Struct.SCARA_Cartesian[0] = -380;
      RobotArmData_Struct.SCARA_Cartesian[1] = 0;
      RobotArmData_Struct.SCARA_Cartesian[2] = -120;
      My_mDelay(100);
      RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 1;
      My_mDelay(1000);
      RobotArm_WaitStop();
      RobotArmData_Struct.SCARA_Cartesian[2] = -170;
      My_mDelay(500);
      RobotArm_WaitStop();
      RobotArmData_Struct.ServoPwmDuty[0] = Claw_J;
      My_mDelay(400);
      RobotArmData_Struct.SCARA_Cartesian[0] = -380;
      RobotArmData_Struct.SCARA_Cartesian[1] = 0;
      RobotArmData_Struct.SCARA_Cartesian[2] = -40;
      My_mDelay(500);
      RobotArm_WaitStop();
      PositionXmm_Diff = PositionXmm_Old - Read_Position_x_mm();
      PositionYmm_Diff = PositionYmm_Old - Read_Position_y_mm();
      ChassisCoord_Set( PositionXmm_Diff, PositionYmm_Diff, 0 );//ת�Ƶ��տ�ʼʶ���λ��
      ChassisCoord_WaitStop();
      My_mDelay(2000);



      rccu_setmode_to_tracking();
      My_mDelay(50);
      LineTracker_Execute_LineNum(CarDirection_Right, 250, 2, 1, 1);
      LineTracker_WaitCarToStop();
      My_mDelay(500);
      RobotArmData_Struct.SCARA_Cartesian[0] = 0;
      RobotArmData_Struct.SCARA_Cartesian[1] = -380;//-(run_cnt*25);
      RobotArmData_Struct.SCARA_Cartesian[2] = -100 + (run_cnt * 50);
      LineTracker_Execute_LineNum(CarDirection_Head, 250, 5 - run_cnt, 0, 1);
      Rotation_Claw(1000);
      LineTracker_WaitCarToStop();
      My_mDelay(500);
      RobotArm_WaitStop();
      RobotArmData_Struct.SCARA_Cartesian[2] = -170 + (run_cnt * 50); //����
      My_mDelay(500);
      RobotArm_WaitStop();
      RobotArmData_Struct.ServoPwmDuty[0] = Claw_S;
      My_mDelay(300);
      RobotArmData_Struct.SCARA_Cartesian[2] += 50;//��е��̧��
      My_mDelay(500);
      RobotArm_WaitStop();
      if(run_cnt < (BRICK_NUM - 1))
      {
        RobotArmData_Struct.ServoPwmDuty[1] = Rotation_Claw_Init;
        RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 0;
        RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 0;
        RobotArmData_Struct.SCARAflg_U.bit.SCARAReset_Enabled = 1;
        My_mDelay(500);
        RobotArmData_Struct.SCARAflg_U.bit.SCARAReset_Enabled = 0;
        My_mDelay(500);
        LineTracker_Execute_Encoder(CarDirection_Tail, 200, 0, 150, 1);
        LineTracker_WaitCarToStop();
        My_mDelay(300);
        LineTracker_Execute_LineNum(CarDirection_Tail, 250, 3 - run_cnt, 1, 1);
        LineTracker_Execute_LineNum(CarDirection_Left, 250, 2, 0, 1);
        RobotArm_Rst();
        RobotArmData_Struct.MotorAngle_Target[2] = -180;
        RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 1;
        My_mDelay(500);
      }
    }
    for(; run_cnt > 0; run_cnt--)
    {
      if(run_cnt != BRICK_NUM)
      {
        LineTracker_Execute_LineNum(CarDirection_Left, 250, 2, 1, 1);
        My_mDelay(2000);
      }
      RobotArmData_Struct.SCARA_Cartesian[0] = 0;
      RobotArmData_Struct.SCARA_Cartesian[1] = -380;// - ( ( run_cnt - 1 ) * 25 );
      RobotArmData_Struct.SCARA_Cartesian[2] = ( ( run_cnt - 1 ) * 50 ) - 100;
      My_mDelay(100);
      if(run_cnt != BRICK_NUM)
      {
        LineTracker_Execute_LineNum(CarDirection_Head, 250, 4, 0, 1);
      }
      RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 1;
      RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 1;
      My_mDelay(1000);
      RobotArm_WaitStop();
      LineTracker_WaitCarToStop();
      RobotArmData_Struct.SCARA_Cartesian[2] = ( ( run_cnt - 1 ) * 50 ) - 170;
      My_mDelay(500);
      RobotArm_WaitStop();
      RobotArmData_Struct.ServoPwmDuty[0] = Claw_J;
      My_mDelay(400);
      RobotArmData_Struct.SCARA_Cartesian[2] = ( ( run_cnt - 1 ) * 50 ) - 80;
      My_mDelay(900);
      RobotArm_WaitStop();
      RobotArmData_Struct.SCARA_Cartesian[0] = 380;// + ((BRICK_NUM-run_cnt)*25);
      RobotArmData_Struct.SCARA_Cartesian[1] = 0;
      LineTracker_Execute_Encoder(CarDirection_Tail, 200, 0, 150, 1);
      LineTracker_WaitCarToStop();
      My_mDelay(300);
      RobotArmData_Struct.SCARA_Cartesian[2] = -100 + ((BRICK_NUM - run_cnt) * 50); //����
      LineTracker_Execute_LineNum(CarDirection_Tail, 250, 3, 1, 1);
      LineTracker_Execute_LineNum(CarDirection_Right, 250, 2, 0, 1);
      LineTracker_WaitCarToStop();
      RobotArm_WaitStop();
      RobotArmData_Struct.SCARA_Cartesian[2] = -170 + ((BRICK_NUM - run_cnt) * 50); //��
      My_mDelay(500);
      RobotArm_WaitStop();
      RobotArmData_Struct.ServoPwmDuty[0] = Claw_S;
      My_mDelay(300);
      RobotArmData_Struct.SCARA_Cartesian[2] += 50;//��е��̧��
      My_mDelay(500);
      RobotArm_WaitStop();
    }
    LineTracker_Execute_LineNum(CarDirection_Left, 250, 3, 1, 1);
    RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 0;
    RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 0;
    RobotArmData_Struct.SCARAflg_U.bit.SCARAReset_Enabled = 1;
    My_mDelay(500);
    RobotArmData_Struct.SCARAflg_U.bit.SCARAReset_Enabled = 0;
    LineTracker_WaitCarToStop();
    My_mDelay(500);
    LineTracker_Execute_LineNum(CarDirection_Tail, 250, 4, 0, 1);
    LineTracker_Execute_Encoder(CarDirection_Tail, 200, 0, 420, 0);
    LineTracker_WaitCarToStop();
    Debug_Await();
  }
#endif
  //*/


  //Debug_Await();
}


int Task_User_create(void)
{

  thread_PathWrite = rt_thread_create( "PathWrite",             /* �߳����� */
                                       PathWrite_task,          /* �߳���ں��� */
                                       RT_NULL,                 /* �߳���ں������� */
                                       1024,          		  /* �߳�ջ��С */
                                       10,                      /* �̵߳����ȼ� */
                                       20);                     /* �߳�ʱ��Ƭ */
  if(thread_PathWrite != RT_NULL)
  {
    rt_thread_startup(thread_PathWrite);
    rt_kprintf("thread_PathWrite startup!\n");
  }

  return 0;
}
void RobotArmGetblock(uint8_t color)
{
  RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 0;
  RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 0;
  //��ʱ500ʹMODBUSͨѶ���԰����ݴ����ȥ
  My_mDelay(500);
  //��е�۵��̽Ƕ�����Ϊ��׼����ĽǶ�
  RobotArmData_Struct.MotorAngle_Target[0] = -30 ;
  RobotArmData_Struct.MotorAngle_Target[1] =  110;
  //	RobotArmData_Struct.MotorAngle_Target[2] = -170; //���ץ
  RobotArmData_Struct.MotorAngle_Target[2] = 0; //�Ҳ�ץ
  //�Ƕ�ʹ��
  RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 1;
  //��ʱ500ʹMODBUSͨѶ���԰����ݴ����ȥ
  My_mDelay(500);
  RobotArm_WaitStop();//�����Ӿ�ʶ��ĽǶ�
  RobotArmData_Struct.ServoPwmDuty[0] = 1600;//��ֵ���������Ӧ
  My_mDelay(500);

  RecognitionModule_Start(&RecognitionModule_t);//�Ӿ����Һ�ɫ
  while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
  {
    if(RecognitionModule_t.RecognitionModuleSte == RM_error)
    {
      //ʶ��ʧ��
      if(RecognitionModule_t.err == ERR_disconnect)
      {
        My_mDelay(50);
        RecognitionModule_Start(&RecognitionModule_t);
      }
    }
    else if(RecognitionModule_t.RecognitionModuleSte == RM_Identify)
    {
      break;
    }
    My_mDelay(10);
  }


//	switch(color) //��¼Ҫץȡ����ɫ�������Ӿ������ȴ��Ӿ�����
//	{
//		case color_red:
//		color_haveget[color_red]++;
//		RecognitionModule_Start(&RecognitionModule_t);//�Ӿ����Һ�ɫ
//		while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
//		{
//			if(RecognitionModule_t.RecognitionModuleSte == RM_error)
//			{//ʶ��ʧ��
//				if(RecognitionModule_t.err == ERR_disconnect)
//				{
//					My_mDelay(50);
//					RecognitionModule_Start(&RecognitionModule_t);
//				}
//			}
//			else if(RecognitionModule_t.RecognitionModuleSte == RM_Identify)
//			{
//				break;
//			}
//			My_mDelay(10);
//		}
//		break;
//		case color_green:
//		color_haveget[color_green]++;
//		ColorRecognitionModule_Start(&RecognitionModule_t);//�Ӿ�������ɫ
//		while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
//		{
//			if(RecognitionModule_t.RecognitionModuleSte == RM_error)
//			{//ʶ��ʧ��
//				if(RecognitionModule_t.err == ERR_disconnect)
//				{
//					My_mDelay(50);
//					ColorRecognitionModule_Start(&RecognitionModule_t);
//				}
//			}
//			else if(RecognitionModule_t.RecognitionModuleSte == RM_Identify)
//			{
//				break;
//			}
//			My_mDelay(10);
//		}
//		break;
//		case color_blue:
//		color_haveget[color_blue]++;
//		CircleRecognitionModule_Start(&RecognitionModule_t);//�Ӿ�������ɫ
//		while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
//		{
//			if(RecognitionModule_t.RecognitionModuleSte == RM_error)
//			{//ʶ��ʧ��
//				if(RecognitionModule_t.err == ERR_disconnect)
//				{
//					My_mDelay(50);
//					CircleRecognitionModule_Start(&RecognitionModule_t);
//				}
//			}
//			else if(RecognitionModule_t.RecognitionModuleSte == RM_Identify)
//			{
//				break;
//			}
//			My_mDelay(10);
//		}
//		break;
//		default:break;
//	}
  IdentifyBrick_Get(-20, 25); //ʶ��שλ�ý��бջ�����С������λ��
//����ֵ������Ҫ�Ӿ�ʶ����������Ӿ���Ļ��λ��(һ����Ӿ�����λ��)��������������̣������ƶ�
  My_mDelay(500);
  RecognitionModule_Stop(&RecognitionModule_t);
  Rotation_Claw(BrickData_Struct.yaw);
//��ʱ�����ڶ�Ӧλ�ã�����ץȡ
  //�Ҳ�ץ
  ChassisSpeed_Set(0, 0);
  switch(color)
  {
  case color_red:
    color_haveget[color_red]++;
    RobotArmData_Struct.ServoPwmDuty[1] = ServoPwmDuty[color_red];//ץȡ����
    if(color_haveget[color_red] == 1)
    {
      RobotArmData_Struct.SCARA_Cartesian[0] = position_Cartesian[color_red][0];
      RobotArmData_Struct.SCARA_Cartesian[1] = position_Cartesian[color_red][1];
      RobotArmData_Struct.SCARA_Cartesian[2] = position_Cartesian[color_red][2];
    }
    if(color_haveget[color_red] == 2)
    {
      RobotArmData_Struct.SCARA_Cartesian[0] = position_Cartesian1[color_red][0];
      RobotArmData_Struct.SCARA_Cartesian[1] = position_Cartesian1[color_red][1];
      RobotArmData_Struct.SCARA_Cartesian[2] = position_Cartesian1[color_red][2];
    }
    break;
  case color_green:
    RobotArmData_Struct.ServoPwmDuty[1] = ServoPwmDuty[color_green];//ץȡ����
    color_haveget[color_green]++;
    if(color_haveget[color_green] == 1)
    {
      RobotArmData_Struct.SCARA_Cartesian[0] = position_Cartesian[color_green][0];
      RobotArmData_Struct.SCARA_Cartesian[1] = position_Cartesian[color_green][1];
      RobotArmData_Struct.SCARA_Cartesian[2] = position_Cartesian[color_green][2];
    }
    if(color_haveget[color_green] == 2)
    {
      RobotArmData_Struct.SCARA_Cartesian[0] = position_Cartesian1[color_green][0];
      RobotArmData_Struct.SCARA_Cartesian[1] = position_Cartesian1[color_green][1];
      RobotArmData_Struct.SCARA_Cartesian[2] = position_Cartesian1[color_green][2];
    }
    break;
  case color_blue:
    color_haveget[color_blue]++;
    RobotArmData_Struct.ServoPwmDuty[1] = ServoPwmDuty[color_blue];//ץȡ����
    if(color_haveget[color_blue] == 1)
    {
      RobotArmData_Struct.SCARA_Cartesian[0] = position_Cartesian[color_blue][0];
      RobotArmData_Struct.SCARA_Cartesian[1] = position_Cartesian[color_blue][1];
      RobotArmData_Struct.SCARA_Cartesian[2] = position_Cartesian[color_blue][2];
    }
    if(color_haveget[color_blue] == 2)
    {
      RobotArmData_Struct.SCARA_Cartesian[0] = position_Cartesian1[color_blue][0];
      RobotArmData_Struct.SCARA_Cartesian[1] = position_Cartesian1[color_blue][1];
      RobotArmData_Struct.SCARA_Cartesian[2] = position_Cartesian1[color_blue][2];
    }
    break;
  default:
    break;
  }
//	RobotArmData_Struct.SCARA_Cartesian[0] = 390;
//	RobotArmData_Struct.SCARA_Cartesian[1] = 100;
//	RobotArmData_Struct.SCARA_Cartesian[2] = -150+(2-color_haveget[color])*Brick_Size;
//	//���ץ
//	RobotArmData_Struct.SCARA_Cartesian[0] = -430;
//	RobotArmData_Struct.SCARA_Cartesian[1] = -60;
//	RobotArmData_Struct.SCARA_Cartesian[2] = -150+(2-color_haveget[color])*Brick_Size;
  //��е������ϵ�������������
  RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 1;//����ϵʹ��
  My_mDelay(500);
  RobotArm_WaitStop();
  RobotArmData_Struct.ServoPwmDuty[0] = 1250;//ץȡ����
  My_mDelay(400);
  //�Ҳ�ץ--�������ƶ�����ͷ����

  RobotArmData_Struct.SCARA_Cartesian[2] += 100;
  My_mDelay(1000);
  RobotArm_WaitStop();
  RobotArmData_Struct.SCARA_Cartesian[0] = 50;
  RobotArmData_Struct.SCARA_Cartesian[1] = -300;
  RobotArmData_Struct.SCARA_Cartesian[2] = 150;




  //���ץ
//	RobotArmData_Struct.SCARA_Cartesian[0] = -300;
//	RobotArmData_Struct.SCARA_Cartesian[1] = -60;
//	RobotArmData_Struct.SCARA_Cartesian[2] +=150;
  //��ʱ500ʹMODBUSͨѶ���԰����ݴ����ȥ
//	RobotArmData_Struct.ServoPwmDuty[1] = 1450;//ץȡ����
  My_mDelay(500);
  RobotArm_WaitStop();
  ChassisSpeed_Set(0, 0);
}




uint8_t set_num = 0;
void RobotArmDown(uint8_t color)   // ������0����  1����  2����
{
  set_num = color;
  switch(set_num)
  {
  // 0 1 2 �����������ҵ�˳��
  case 0 :
    ChassisSpeed_Set(-100, 0); //��ǰ�ط�����Ҫ����ҪŲ��
    My_mDelay(1000);
    while((LineTrack_Scan(CarDirection_Tail, &Line_Count) & (3 << 3)) != (3 << 3) )
    {
      //����ls1�����  <=
      ChassisSpeed_Set(-100, 0);
    }
    ChassisSpeed_Set(0, 0);
    break;
  case 1 :
    ChassisSpeed_Set(0, 0);
    break;
  case 2 :
    ChassisSpeed_Set(100, 0); //��ǰ�ط�����Ҫ����ҪŲ��
    My_mDelay(1000);
    while((LineTrack_Scan(CarDirection_Tail, &Line_Count) & (3 << 3)) != (3 << 3) ) //00011000  ��ͷ�����Ҳ���ls1
    {
      //����ls1�����  <=
      ChassisSpeed_Set(100, 0);
    }
    ChassisSpeed_Set(0, 0);
    break;
  default:
    break;
  }
  if(color_haveget[color] > 1) //�����
  {
    switch(color) //��¼Ҫץȡ����ɫ�������Ӿ������ȴ��Ӿ�����
    {
    case color_red:
      RecognitionModule_Start(&RecognitionModule_t);//�Ӿ����Һ�ɫ
      while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
      {
        if(RecognitionModule_t.RecognitionModuleSte == RM_error)
        {
          //ʶ��ʧ��
          if(RecognitionModule_t.err == ERR_disconnect)
          {
            My_mDelay(50);
            RecognitionModule_Start(&RecognitionModule_t);
          }
        }
        else if(RecognitionModule_t.RecognitionModuleSte == RM_Identify)
        {
          break;
        }
        My_mDelay(10);
      }
      break;
    case color_green:
      ColorRecognitionModule_Start(&RecognitionModule_t);//�Ӿ�������ɫ
      while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
      {
        if(RecognitionModule_t.RecognitionModuleSte == RM_error)
        {
          //ʶ��ʧ��
          if(RecognitionModule_t.err == ERR_disconnect)
          {
            My_mDelay(50);
            ColorRecognitionModule_Start(&RecognitionModule_t);
          }
        }
        else if(RecognitionModule_t.RecognitionModuleSte == RM_Identify)
        {
          break;
        }
        My_mDelay(10);
      }
      break;
    case color_blue:
      CircleRecognitionModule_Start(&RecognitionModule_t);//�Ӿ�������ɫ
      while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
      {
        if(RecognitionModule_t.RecognitionModuleSte == RM_error)
        {
          //ʶ��ʧ��
          if(RecognitionModule_t.err == ERR_disconnect)
          {
            My_mDelay(50);
            CircleRecognitionModule_Start(&RecognitionModule_t);
          }
        }
        else if(RecognitionModule_t.RecognitionModuleSte == RM_Identify)
        {
          break;
        }
        My_mDelay(10);
      }
      break;
    default:
      break;
    }
    IdentifyBrick_Get(-10, -25); //ʶ��שλ�ý��бջ�����С������λ��
//����ֵ������Ҫ�Ӿ�ʶ����������Ӿ���Ļ��λ��(һ����Ӿ�����λ��)��������������̣������ƶ�
    My_mDelay(500);
    RecognitionModule_Stop(&RecognitionModule_t);
    Rotation_Claw(BrickData_Struct.yaw);
  }
//75 -450 -150
  RobotArmData_Struct.SCARA_Cartesian[0] = 75;
  RobotArmData_Struct.SCARA_Cartesian[1] = -450;
  RobotArmData_Struct.SCARA_Cartesian[2] = -150 + (color_haveget[color] - 1) * Brick_Size;
  //��е������ϵ�������������
  RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 1;//����ϵʹ��
  My_mDelay(500);
  RobotArm_WaitStop();
  RobotArmData_Struct.ServoPwmDuty[0] = Claw_S;//�ɿ�צ��
  My_mDelay(2000);
  RobotArmData_Struct.SCARA_Cartesian[0] = 50;
  RobotArmData_Struct.SCARA_Cartesian[1] = -300;
  RobotArmData_Struct.SCARA_Cartesian[2] = 150;
  My_mDelay(500);
  RobotArm_WaitStop();

  switch(set_num)
  {
  // 0 1 2 �����������ҵ�˳��
  case 0 :
    ChassisSpeed_Set(100, 0); //��ǰ�ط�����Ҫ����ҪŲ��
    My_mDelay(1000);
    while((LineTrack_Scan(CarDirection_Tail, &Line_Count) & (3 << 3)) != (3 << 3) )
    {
      //����ls1�����  <=
      ChassisSpeed_Set(150, 0);
    }
    ChassisSpeed_Set(0, 0);
    break;
  case 1 :
    ChassisSpeed_Set(0, 0);
    break;
  case 2 :
    ChassisSpeed_Set(-100, 0); //��ǰ�ط�����Ҫ����ҪŲ��
    My_mDelay(1000);
    while((LineTrack_Scan(CarDirection_Tail, &Line_Count) & (3 << 3)) != (3 << 3) ) //00011000  ��ͷ�����Ҳ���ls1
    {
      //����ls1�����  <=
      ChassisSpeed_Set(-150, 0);
    }
    ChassisSpeed_Set(0, 0);
    break;
  default:
    break;
  }
}


uint8_t LineTrack_Scan(DirectionDef_e Car_Direction, uint8_t *Count)
{
  *Count = 0;
  uint8_t Line_ScanNum = 1;
  switch (Car_Direction)
  {
  case CarDirection_Head:
    if(LineTracker_Struct.pSignal1->bit.ls1 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 0;
    if(LineTracker_Struct.pSignal1->bit.ls2 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 1;
    if(LineTracker_Struct.pSignal1->bit.ls3 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 2;
    if(LineTracker_Struct.pSignal1->bit.ls4 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 3;
    if(LineTracker_Struct.pSignal1->bit.ls5 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 4;
    if(LineTracker_Struct.pSignal1->bit.ls6 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 5;
    if(LineTracker_Struct.pSignal1->bit.ls7 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 6;
    if(LineTracker_Struct.pSignal1->bit.ls8 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 7;
    break;
  case CarDirection_Tail:
    if(LineTracker_Struct.pSignal3->bit.ls1 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 0;
    if(LineTracker_Struct.pSignal3->bit.ls2 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 1;
    if(LineTracker_Struct.pSignal3->bit.ls3 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 2;
    if(LineTracker_Struct.pSignal3->bit.ls4 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 3;
    if(LineTracker_Struct.pSignal3->bit.ls5 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 4;
    if(LineTracker_Struct.pSignal3->bit.ls6 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 5;
    if(LineTracker_Struct.pSignal3->bit.ls7 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 6;
    if(LineTracker_Struct.pSignal3->bit.ls8 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 7;
    break;
  case CarDirection_Left:
    if(LineTracker_Struct.pSignal2->bit.ls1 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 0;
    if(LineTracker_Struct.pSignal2->bit.ls2 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 1;
    if(LineTracker_Struct.pSignal2->bit.ls3 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 2;
    if(LineTracker_Struct.pSignal2->bit.ls4 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 3;
    if(LineTracker_Struct.pSignal2->bit.ls5 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 4;
    if(LineTracker_Struct.pSignal2->bit.ls6 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 5;
    if(LineTracker_Struct.pSignal2->bit.ls7 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 6;
    if(LineTracker_Struct.pSignal2->bit.ls8 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 7;
    break;
  case CarDirection_Right:
    if(LineTracker_Struct.pSignal4->bit.ls1 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 0;
    if(LineTracker_Struct.pSignal4->bit.ls2 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 1;
    if(LineTracker_Struct.pSignal4->bit.ls3 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 2;
    if(LineTracker_Struct.pSignal4->bit.ls4 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 3;
    if(LineTracker_Struct.pSignal4->bit.ls5 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 4;
    if(LineTracker_Struct.pSignal4->bit.ls6 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 5;
    if(LineTracker_Struct.pSignal4->bit.ls7 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 6;
    if(LineTracker_Struct.pSignal4->bit.ls8 == LineTracker_Struct.active_level) Line_ScanNum |= 1 << 7;
    break;
  default:
    break;
  }
  for(int i = 0; i < 8; i++)
  {
    if(Line_ScanNum & (1 << i))  (*Count)++;
  }
  if(Line_ScanNum == 1) return 0;
  return Line_ScanNum;
}

/****************************
Car_direction:�н�����
Car_side:   ������ƻ��Ǵ��ұ���  0������    1������
***********************/
void Avoid_Obstacle(DirectionDef_e Car_Direction, uint8_t Car_side)
{
  int16_t speed = 150;
  uint16_t time = 2500;
  ChassisSpeed_Set(0, 0);
  //�ٶȿ���
  switch (Car_Direction)
  {
  case CarDirection_Head:
    if(Car_side == 0)
    {
      ChassisSpeed_Set(-speed, 0);
      My_mDelay(time);
      ChassisSpeed_Set(0, speed);
      My_mDelay(time + 2000);
      ChassisSpeed_Set(speed, 0);
      My_mDelay(time);
      ChassisSpeed_Set(0, 0);
      My_mDelay(time);
    }
    if(Car_side == 1)
    {
      ChassisSpeed_Set(speed, 0);
      My_mDelay(time);
      ChassisSpeed_Set(0, speed);
      My_mDelay(time + 2000);
      ChassisSpeed_Set(-speed, 0);
      My_mDelay(time);
      ChassisSpeed_Set(0, 0);
      My_mDelay(time);
    }
    break;
  case CarDirection_Tail:
    if(Car_side == 0)
    {
      ChassisSpeed_Set(-speed, 0);
      My_mDelay(time);
      ChassisSpeed_Set(0, -speed);
      My_mDelay(time + 2000);
      ChassisSpeed_Set(speed, 0);
      My_mDelay(time);
      ChassisSpeed_Set(0, 0);
      My_mDelay(time);
    }
    if(Car_side == 1)
    {
      ChassisSpeed_Set(speed, 0);
      My_mDelay(time);
      ChassisSpeed_Set(0, -speed);
      My_mDelay(time + 2000);
      ChassisSpeed_Set(-speed, 0);
      My_mDelay(time);
      ChassisSpeed_Set(0, 0);
      My_mDelay(time);
    }
    break;
  default:
    break;
  }
  //λ�ÿ���
//	//x��ƽ��
//	PositionXmm_Diff =  dir*Obstacle_length*1.0/2.0+PositionXmm_Old - Read_Position_x_mm();
//	PositionYmm_Diff = PositionYmm_Old  - Read_Position_y_mm();
//	ChassisCoord_Set( PositionXmm_Diff, PositionYmm_Diff, 0 );
//	ChassisCoord_WaitStop();//�ȴ�
//	//y��ƽ��
//	PositionXmm_Diff = dir*Obstacle_length*1.0/2.0+PositionXmm_Old - Read_Position_x_mm();
//	PositionYmm_Diff = Obstacle_width*1.0/2.0 +PositionYmm_Old - Read_Position_y_mm();
//	ChassisCoord_Set( PositionXmm_Diff, PositionYmm_Diff, 0 );
//	ChassisCoord_WaitStop();
//	//x��ƽ��
//	PositionXmm_Diff = PositionXmm_Old - Read_Position_x_mm();
//	PositionYmm_Diff = Obstacle_width*1.0/2.0 +PositionYmm_Old - Read_Position_y_mm();
//	ChassisCoord_Set( PositionXmm_Diff, PositionYmm_Diff, 0 );
//	ChassisCoord_WaitStop();//�ȴ�
////	ChassisStop();
////	PositionXmm_Diff = Read_Position_x_mm();
////	PositionYmm_Diff = Read_Position_y_mm();
}

uint8_t Find_Obstacle(DirectionDef_e Car_Direction)//Ѳ�߷���ȴ��ϰ���CarDirection_Head
{
  static uint8_t time_cnt = 0;
  uint16_t Distance = 0;

  switch (Car_Direction)
  {
  case CarDirection_Head:

    if(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance2)//��ֹ�������������ʱ����������Ч
    {
//				LineSingle_Tracker(LineTrack_Scan(CarDirection_Head,&Line_Count),CarDirection_Head);
      time_cnt = 0;
      while(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance2 <= 200) //��ͷ����ĳ���������
      {
        My_mDelay(5);  //�˴�����̽�鵽�ϰ�����ͣ������
        time_cnt++;
        if(time_cnt >= 5) //�˴�����̽�鵽�ϰ�����ͣ������
        {
          LineTracker_Struct.run_mode = Mode_Await;
          time_cnt = 0;
//						ChassisSpeed_Set(0,0);
          return 1;//ǰ�����ϰ����ʱ����ѭ���������
        }
      }
    }
    break;
  case CarDirection_Tail:

    while(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance4)
    {
//				LineSingle_Tracker(LineTrack_Scan(CarDirection_Tail,&Line_Count),CarDirection_Tail);
      time_cnt = 0;
      while(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance4 <= 200) //��ͷ����ĳ���������
      {
        My_mDelay(5);  //�˴�����̽�鵽�ϰ�����ͣ������
        time_cnt++;
        if(time_cnt >= 5) //�˴�����̽�鵽�ϰ�����ͣ������
        {
          LineTracker_Struct.run_mode = Mode_Await;
          time_cnt = 0;
//					ChassisSpeed_Set(0,0);
          return 1;//ǰ�����ϰ����ʱ����ѭ���������
        }
      }
    }
    break;
  default:
    break;
  }
  return 0;
}
void LineSingle_Tracker(uint8_t Line_TrackScan, DirectionDef_e Car_Direction)
{
  uint8_t zuo_count = 0, you_count = 0; // ������������
  float output = 0, error, derivative;
  static float prev_error1, integral1;
  static float prev_error2, integral2;
  // ����ÿ�������״̬

  for (int i = 0; i < 8; i++)
  {
    if (Line_TrackScan & (1 << i))   // �������Ƿ񱻼���
    {
      if(i < 4) you_count += (4 - i);
      if(i >= 4) zuo_count += -(i - 4);
    }
  }
  // ����PID�������
  if(Car_Direction == CarDirection_Head)
    error = you_count + zuo_count; // ����ƫ��
  if(Car_Direction == CarDirection_Tail)
    error = zuo_count - you_count; // ����ƫ��

  switch(Car_Direction)
  {
  case CarDirection_Head:
    integral1 += error; // ���ָ���
    float derivative = error - prev_error1; // ΢��
//	output = 30.0f * error + 0 * integral + 0.2f * derivative;
    output = 1.5f * error + 0 * integral1 + 0 * derivative;
    prev_error1 = error; // ����ǰһ�����
//			ChassisSpeed_Set(output,150);//��ǰѭ��
    ChassisSpeed_Set1(0, 150, output);
    break;

  case CarDirection_Tail:
    integral2 += error; // ���ָ���
    derivative = error - prev_error2; // ΢��
//	output = 30.0f * error + 0 * integral + 0.2f * derivative;
    output = 4.0f * error + 0 * integral2 + 0.3f * derivative;
    prev_error2 = error; // ����ǰһ�����
//			ChassisSpeed_Set(output,-150);//����ѭ��
    ChassisSpeed_Set1(0, -150, output);
    break;
  default:
    break;
  }
  zuo_count = 0;
  you_count = 0;
}

uint16_t FindEnd_Head(void)
{
  LineTrack_Scan(CarDirection_Head, &Line_Count);
  while(Line_Count >= 5)
  {
    My_mDelay(5);
    LineTrack_Scan(CarDirection_Head, &Line_Count);
    if(Line_Count >= 5)
      return 1;
    else return 0;
  }
  return 0;
}
uint16_t FindEnd_Tail(void)
{
  LineTrack_Scan(CarDirection_Tail, &Line_Count);
  while(Line_Count >= 5)
  {
    My_mDelay(5);
    LineTrack_Scan(CarDirection_Tail, &Line_Count);
    if(Line_Count >= 5)
      return 1;
    else return 0;
  }
  return 0;
}
uint16_t FindHead_Obstacle(void)
{
  uint8_t time_cnt = 0;
  if(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance1)//��ֹ�������������ʱ����������Ч
  {
    time_cnt = 0;
    while(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance1 <= 200) //��ͷ����ĳ���������
    {
      My_mDelay(5);  //�˴�����̽�鵽�ϰ�����ͣ������
      time_cnt++;
      if(time_cnt >= 5) //�˴�����̽�鵽�ϰ�����ͣ������
      {
        LineTracker_Struct.run_mode = Mode_Await;
        time_cnt = 0;
        return 1;//ǰ�����ϰ����ʱ����ѭ���������
      }
    }
  }
  return 0;
}
uint16_t FindEnd_Obstacle(void)
{
  uint8_t time_cnt = 0;
  if(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance3)//��ֹ�������������ʱ����������Ч
  {
    while(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance3 <= 200) //��ͷ����ĳ���������
    {
      My_mDelay(5);  //�˴�����̽�鵽�ϰ�����ͣ������
      time_cnt++;
      if(time_cnt >= 5) //�˴�����̽�鵽�ϰ�����ͣ������
      {
        LineTracker_Struct.run_mode = Mode_Await;

        return 1;//ǰ�����ϰ����ʱ����ѭ���������
      }
    }
  }
  return 0;
}
void CircleRecognitionModule_Start(RecognitionModule_s *hand)
{
  if((hand->RecognitionModuleSte == RM_leisure)
      || (hand->RecognitionModuleSte == RM_error)
      || (hand->RecognitionModuleSte == RM_succeed))
  {
    hand->RecognitionModuleSte = RM_Circlestart;
  }
}