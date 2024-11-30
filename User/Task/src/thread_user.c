/* Includes ------------------------------------------------------------------*/
#include "thread_user.h"
/* 开发板数据文件 */
#include "data.h"
/* 其他任务头文件 */
#include "thread_rccu.h"
#include "thread_gui.h"
#include "chassis_LineTracker.h"
/* Private macros ------------------------------------------------------------*/
#define BRICK_NUM    4 //抓砖个数
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 定义线程控制块指针 */
rt_thread_t thread_PathWrite = RT_NULL;
/*
*/
/***************************************************************/
//新加入参数，函数

#define Obstacle_width  1000
#define Obstacle_length 1500  //可以比原物体大，超声波测距有一段距离
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
uint16_t FindEnd_Left(void);
uint16_t FindEnd_Right(void);
uint16_t FindHead_Obstacle(void);
uint16_t FindEnd_Obstacle(void);
uint8_t color_haveget[3] = {0, 0, 0}; //对应color_e里面的数量
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
color_e color_now = color_green; // 当前正在夹取的物块颜色
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
  //等待机械臂运动停止
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
  //开始开启识别
  bool_recognitionflag = 0;
  RecognitionModule_Start(&RecognitionModule_t);
  while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
  {
    if(RecognitionModule_t.RecognitionModuleSte == RM_error)
    {
      //识别失败
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
  //机械爪对应砖块角度
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


//视觉坐标(-100,-10)-->(-25,-10) 往前行进   IdentifyBrick_Get(-10,-25);
//视觉坐标(-25,60)--> (-25,-10)  往右行进   IdentifyBrick_Get(-10,-25);
static void IdentifyBrick_Get(float SET_LOCATION_X, float SET_LOCATION_Y)
{
  //识别砖的坐标
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


typedef enum
{
  CHASSIS_STATE_STOP = 0,  // 底盘机械臂停止
  CHASSIS_STATE_1,         // 底盘从启动区到资源岛
  CHASSIS_STATE_2,         // 机械臂从资源岛夹取物块
  CHASSIS_STATE_3,         // 底盘从资源岛到第一个丁字路口并在位修正
  CHASSIS_STATE_4,         // 底盘从第一个丁字路口往前走避障
  CHASSIS_STATE_5,         // 底盘绿色终点走向红色终点前丁字路口并在位修正
  CHASSIS_STATE_6,         // 底盘检测到障碍物之后走到绿色终点前丁字路口并在位修正
  CHASSIS_STATE_7,         // 底盘绿色终点走向蓝色终点前丁字路口并在位修正
  CHASSIS_STATE_8,         // 机械臂放置物块到终点函数
  CHASSIS_STATE_9,         // 底盘从红色终点前丁字路口出发返回第一个丁字路口
  CHASSIS_STATE_10,        // 底盘从绿色终点前丁字路口出发返回第一个丁字路口
  CHASSIS_STATE_11,        // 底盘从蓝色终点前丁字路口出发返回第一个丁字路口


} robot_chassis_state;

uint8_t time_cnt = 0, Line_Count = 0;;
uint8_t Line_ScanNum = 0;
int stack_free, stack_percentage;
int count_test = 10;
robot_chassis_state game_stat, last_game_stat; // game的状态和上次状态
float v1 = 100, v2 = 200, v3 = 300; // 三挡速度
uint8_t brick_count; // 已经夹取物块数量
float l1 = 900, // 启动区到资源岛
      l2 = 350, // 资源岛到第一个丁字路口
      l3 = 400, // 避障左右移动距离
      l4 = 200, // 车前进方向碰线后进行T字修正前移动量
      l5 = 1200, // 超车障碍块前后移动距离
      l6 = 1200, // 从红色终点T字往回走到
      l7 = 520; //


void buzz_note_state_delay_100ms_begin()
{
  buzzerSound(M1);
  My_mDelay(50);
  buzzerSound(M5);
  My_mDelay(50);
  buzzerSound(0);
}
void buzz_note_state_delay_100ms_end()
{
  buzzerSound(M5);
  My_mDelay(50);
  buzzerSound(M1);
  My_mDelay(50);
  buzzerSound(0);
}
void PathWrite_task(void *pvParameters)
{
  //路劲规划任务
  // 上电后复位机械臂
  RobotArm_Rst();
  // 停止视觉模块
  RecognitionModule_Stop(&RecognitionModule_t);

//  My_mDelay(500);
//  RobotArmData_Struct.SCARA_Cartesian[0] = -316.7f;
//  RobotArmData_Struct.SCARA_Cartesian[1] = .0f;
//  RobotArmData_Struct.SCARA_Cartesian[2] = 220.f;
//  RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 1;
//  RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 1;


  // 等待按键按下并抬起后开始
  while( (KEY_4() == 1) || (lcd_page != 1))
    My_mDelay(10);
  while( (KEY_4() == 0) || (lcd_page != 1))
    My_mDelay(10);

  game_stat = CHASSIS_STATE_1;

#if 1 //
  while(1)
  {

    /* ARM TEST */
//    RobotArmData_Struct.MotorAngle_Target[2] = -90;
//    for(int i = 0; i < 1000; i++)
//    {
//      RobotArmData_Struct.MotorAngle_Target[2] += 0.01;
//      My_mDelay(10);
//    }
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

    /* CHASSIS TEST */
//    ChassisCoord_Set(0, 10, 0);
//    ChassisCoord_WaitStop();
//    My_mDelay(200);
//    ChassisCoord_Set(0, -10, 0);
//    ChassisCoord_WaitStop();
//    My_mDelay(200);
//    ChassisCoord_Set(0, 10, 0);
//    ChassisCoord_WaitStop();
//    My_mDelay(200);
//    ChassisCoord_Set(0, -10, 0);
//    ChassisCoord_WaitStop();
//    My_mDelay(200);


    /* BRICK FOLLOWING TEST */
//    RecognitionModule_t.RecognitionModuleSte = 1; // red
//    RecognitionModule_Start(&RecognitionModule_t);
//  IdentifyBrick_Get(BrickData_Struct.x, BrickData_Struct.y);

    /* LINE TRACKING TEST*/
//    rccu_setmode_to_tracking();
////    My_mDelay(50);
//		LineTracker_Execute_Condition(CarDirection_Head, 100, FindHead_Obstacle, 1, 0);
//		LineTracker_WaitCarToStop();
//		My_mDelay(50);

//			rccu_setmode_to_coord();
//		while(count_test--){
//      buzz_note_state_delay_100ms_begin();
//			ChassisCoord_Set(300,0,90);
//			ChassisCoord_WaitStop();
//			My_mDelay(1000);
//			ChassisCoord_Set(0,300,-90);
//			ChassisCoord_WaitStop();
//			My_mDelay(1000);
//			ChassisCoord_Set(-300,0,90);
//			ChassisCoord_WaitStop();
//			My_mDelay(1000);
//			ChassisCoord_Set(0,-300,-90);
//			ChassisCoord_WaitStop();
//			My_mDelay(1000);
//		}


//    buzz_note_state_delay_100ms_begin();
//    rccu_setmode_to_coord();
//    My_mDelay(100);

//    ChassisCoord_Set(0, 50000, 0);
//    ChassisCoord_WaitStop();
//    buzz_note_state_delay_100ms_end();
//    buzz_note_state_delay_100ms_begin();
//    My_mDelay(250);
//    ChassisSpeed_Set( 0, 300);
//    My_mDelay(2500);
//        buzz_note_state_delay_100ms_end();
//		ChassisSpeed_Set( 0, 200);
//    My_mDelay(200);
//		ChassisSpeed_Set( 0, 300);
//    My_mDelay(1000);
//		ChassisSpeed_Set( 0, 200);
//    My_mDelay(200);
//		ChassisSpeed_Set( 0, 100);
//    My_mDelay(200);

//    buzz_note_state_delay_100ms_begin();
//    rccu_setmode_to_tracking();
//    My_mDelay(100);


//		LineTracker_Execute_Condition(CarDirection_Head, 100, FindEnd_Head, 1, 0);
//		LineTracker_WaitCarToStop();
//    LineTracker_Execute_Condition(CarDirection_Head, 100, FindEnd_Head, 1, 1);


//    LineTracker_Execute_Condition(CarDirection_Head, 100, FindEnd_Tail, 1, 1);
//    LineTracker_WaitCarToStop();
//    buzz_note_state_delay_100ms_end();

//		while(1){

//			LineTracker_Execute_SituAdjust_T_Line(CarDirection_Left, 0, 1000);
//			count_test++;
//			LineTracker_CorrectiveCtrl_T_Line(LineTracker_Struct.pSignal2, LineTracker_Struct.pSignal1, LineTracker_Struct.pSignal4);
//			LineTracker_ChassisPostureCalc_T_Line(LineTracker_Struct.pSignal2, LineTracker_Struct.pSignal1, LineTracker_Struct.pSignal4);
//      My_mDelay(50);
//		}
//              rccu_setmode_to_tracking();
//              My_mDelay(100);
//              LineTracker_Execute_SituAdjust_T_Line(CarDirection_Head, 0, 5000);
//              while(1)
//              {
//                My_mDelay(100);
//              }
    while (1)
    {
      last_game_stat = game_stat;
      buzz_note_state_delay_100ms_begin();
      switch(game_stat)
      {
      case CHASSIS_STATE_STOP:  // 底盘机械臂停止
        break;
      case CHASSIS_STATE_1:  // 底盘从启动区到资源岛
        ChassisSpeed_Set(v2, 0);
        My_mDelay(1000 * l1 / v2);
        rccu_setmode_to_tracking();
        My_mDelay(100);
        LineTracker_Execute_SituAdjust(CarDirection_Right, 1, 1000);
        My_mDelay(1000);
        game_stat = CHASSIS_STATE_2;
        break;
      case CHASSIS_STATE_2:  // 机械臂从资源岛夹取物块
        // 先夹取上面的，中间的，下面的
//						利用brick_count;
        if(brick_count == 0)
          color_now = color_red; // 我夹到了红色
        else if(brick_count == 1)
          color_now = color_green; // 我夹到了红色
        else if(brick_count == 2)
          color_now = color_blue; // 我夹到了红色
        else if(brick_count == 2)
          color_now = color_red; // 我夹到了红色
        if(color_now == color_red)
        {
          game_stat = CHASSIS_STATE_3;
        }
        else if(color_now == color_green)
        {
          game_stat = CHASSIS_STATE_4;
        }
        else if(color_now == color_blue)
        {
          game_stat = CHASSIS_STATE_5;
        }
        break;
      case CHASSIS_STATE_3:  // 底盘从资源岛到第一个丁字路口并在位修正
        ChassisSpeed_Set(-v2, 0);
        My_mDelay(1000 * l2 / v2);
        rccu_setmode_to_tracking();
        My_mDelay(100);
        LineTracker_Execute_SituAdjust_T_Line(CarDirection_Head, 0, 1000);
        My_mDelay(1000);
        game_stat = CHASSIS_STATE_4;
        break;
      case CHASSIS_STATE_4:  // 底盘从第一个丁字路口往前走避障
        rccu_setmode_to_tracking();
        My_mDelay(100);
        LineTracker_Execute_Condition(CarDirection_Head, v1, FindHead_Obstacle, 1, 1);
        LineTracker_WaitCarToStop();
        game_stat = CHASSIS_STATE_6;
        break;
      case CHASSIS_STATE_5:  // 底盘绿色终点走向红色终点前丁字路口并在位修正
        ChassisSpeed_Set(-v2, 0);
        My_mDelay(1000 * l7 / v2);
        rccu_setmode_to_tracking();
        My_mDelay(100);
        LineTracker_Execute_SituAdjust_T_Line(CarDirection_Tail, 0, 1000);
        My_mDelay(1000);
        game_stat = CHASSIS_STATE_8;
        break;
      case CHASSIS_STATE_6:  // 底盘检测到障碍物之后走到绿色终点前丁字路口并在位修正
        ChassisSpeed_Set(-v2, 0);
        My_mDelay(1000 * l3 / v2);
        ChassisSpeed_Set(0, v2);
        My_mDelay(1000 * l5 / v2);
        ChassisSpeed_Set(v2, 0);
        My_mDelay(1000 * l3 / v2);
        rccu_setmode_to_tracking();
        My_mDelay(100);
        LineTracker_Execute_Condition(CarDirection_Head, v1, FindEnd_Head, 1, 0);
        LineTracker_WaitCarToStop();
        ChassisSpeed_Set(0, v2);
        My_mDelay(1000 * l4 / v2);
        rccu_setmode_to_tracking();
        My_mDelay(100);
        LineTracker_Execute_SituAdjust_T_Line(CarDirection_Tail, 0, 1000);
        My_mDelay(1000);
        if(color_now == color_red)
        {
          game_stat = CHASSIS_STATE_5;
        }
        else if(color_now == color_green)
        {
          game_stat = CHASSIS_STATE_8;
        }
        else if(color_now == color_blue)
        {
          game_stat = CHASSIS_STATE_7;
        }
        break;
      case CHASSIS_STATE_7:  // 底盘绿色终点走向蓝色终点前丁字路口并在位修正
        ChassisSpeed_Set(v2, 0);
        My_mDelay(1000 * l7 / v2);
        rccu_setmode_to_tracking();
        My_mDelay(100);
        LineTracker_Execute_SituAdjust_T_Line(CarDirection_Tail, 0, 1000);
        My_mDelay(1000);
        game_stat = CHASSIS_STATE_8;
        break;
      case CHASSIS_STATE_8:
        // 机械臂放置物块函数
        if(color_now == color_red)
        {
          game_stat = CHASSIS_STATE_9;
        }
        else if(color_now == color_green)
        {
          game_stat = CHASSIS_STATE_10;
        }
        else if(color_now == color_blue)
        {
          game_stat = CHASSIS_STATE_11;
        }
        break;
      case CHASSIS_STATE_9:
        // 底盘从红色终点前丁字路口出发返回绿色终点
        ChassisSpeed_Set(v2, 0);
        My_mDelay(1000 * l7 / v2);
        game_stat = CHASSIS_STATE_10;
        break;
      case CHASSIS_STATE_10:
        // 底盘从绿色终点前丁字路口出发返回资源岛
        rccu_setmode_to_tracking();
        My_mDelay(100);
        LineTracker_Execute_Condition(CarDirection_Tail, v1, FindEnd_Obstacle, 1, 1);
        LineTracker_WaitCarToStop();
        ChassisSpeed_Set(-v2, 0);
        My_mDelay(1000 * l3 / v2);
        ChassisSpeed_Set(0, -v2);
        My_mDelay(1000 * l5 / v2);
        ChassisSpeed_Set(v2, 0);
        My_mDelay(1000 * l3 / v2);
        rccu_setmode_to_tracking();
        My_mDelay(100);
        LineTracker_Execute_Condition(CarDirection_Tail, v1, FindEnd_Tail, 1, 0);
        LineTracker_WaitCarToStop();
        ChassisSpeed_Set(0, -v2);
        My_mDelay(1000 * l4 / v2);
        LineTracker_Execute_SituAdjust_T_Line(CarDirection_Head, 0, 1000);
        My_mDelay(1000);
        ChassisSpeed_Set(v2, 0);
        My_mDelay(1000 * l2 / v2);
        brick_count++;
        ChassisSpeed_Set(0, 0);
        game_stat = CHASSIS_STATE_2;
        break;
      case CHASSIS_STATE_11:
        // 底盘从蓝色终点前丁字路口出发返回绿色终点
        ChassisSpeed_Set(-v2, 0);
        My_mDelay(1000 * l7 / v2);
        game_stat = CHASSIS_STATE_10;
        break;
      default:
        break;
      }
      ChassisSpeed_Set(0, 0);
      buzz_note_state_delay_100ms_end();
      My_mDelay(1000);
    }
    while( (KEY_4() == 0) || (lcd_page != 1))
    {
      My_mDelay(50);
      rt_thread_t self = rt_thread_self();
      rt_uint8_t *ptr = (rt_uint8_t *)self->stack_addr;
      while (*ptr == '#') ptr++;
      stack_free = (rt_ubase_t)ptr - (rt_ubase_t)self->stack_addr;
      stack_percentage = 100 * ((rt_ubase_t)ptr - (rt_ubase_t)self->stack_addr) / self->stack_size;
    }
    while( (KEY_4() == 1) || (lcd_page != 1))
    {
      My_mDelay(50);
      rt_thread_t self = rt_thread_self();
      rt_uint8_t *ptr = (rt_uint8_t *)self->stack_addr;
      while (*ptr == '#') ptr++;
      stack_free = (rt_ubase_t)ptr - (rt_ubase_t)self->stack_addr;
      stack_percentage = 100 * ((rt_ubase_t)ptr - (rt_ubase_t)self->stack_addr) / self->stack_size;
    }

    Debug_Await();
  }
#endif







#if 0//
  while(1)
  {
    //进入到循迹模式
    rccu_setmode_to_tracking();
    //延时使任务切换，让rccu任务进行底盘模式切换操作
    My_mDelay(50);
    //以头为方向用250的速度循迹2条线，横线在车底时停止，不使用循迹纠正
    LineTracker_Execute_LineNum(CarDirection_Head, 250, 1, 1, 0);
    //机械臂底盘角度设置为-180
    RobotArmData_Struct.MotorAngle_Target[2] = -180;
    //角度使能
    RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 1;
    //延时500使MODBUS通讯可以把数据传输过去
    My_mDelay(500);
    //以头为方向用250的速度循迹2条线，横线在车底时停止，使用循迹纠正
    LineTracker_Execute_LineNum(CarDirection_Head, 250, 1, 1, 1);
    LineTracker_Execute_SituAdjust(CarDirection_Head, 1, 1000);
    //以左为方向用150的速度循迹1条线，横线在车刚刚循迹到就停止，使用循迹纠正
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
      IdentifyBrick_Get(-20, -20); //识别砖位置进行闭环控制小车底盘位置
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
      ChassisCoord_Set( PositionXmm_Diff, PositionYmm_Diff, 0 );//转移到刚开始识别的位置
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
      RobotArmData_Struct.SCARA_Cartesian[2] = -170 + (run_cnt * 50); //放下
      My_mDelay(500);
      RobotArm_WaitStop();
      RobotArmData_Struct.ServoPwmDuty[0] = Claw_S;
      My_mDelay(300);
      RobotArmData_Struct.SCARA_Cartesian[2] += 50;//机械臂抬起
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
      RobotArmData_Struct.SCARA_Cartesian[2] = -100 + ((BRICK_NUM - run_cnt) * 50); //放下
      LineTracker_Execute_LineNum(CarDirection_Tail, 250, 3, 1, 1);
      LineTracker_Execute_LineNum(CarDirection_Right, 250, 2, 0, 1);
      LineTracker_WaitCarToStop();
      RobotArm_WaitStop();
      RobotArmData_Struct.SCARA_Cartesian[2] = -170 + ((BRICK_NUM - run_cnt) * 50); //放
      My_mDelay(500);
      RobotArm_WaitStop();
      RobotArmData_Struct.ServoPwmDuty[0] = Claw_S;
      My_mDelay(300);
      RobotArmData_Struct.SCARA_Cartesian[2] += 50;//机械臂抬起
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

  thread_PathWrite = rt_thread_create( "PathWrite",             /* 线程名字 */
                                       PathWrite_task,          /* 线程入口函数 */
                                       RT_NULL,                 /* 线程入口函数参数 */
                                       4096,          		  /* 线程栈大小 */
                                       10,                      /* 线程的优先级 */
                                       20);                     /* 线程时间片 */
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
  //延时500使MODBUS通讯可以把数据传输过去
  My_mDelay(500);
  //机械臂底盘角度设置为对准物体的角度
  RobotArmData_Struct.MotorAngle_Target[0] = -30 ;
  RobotArmData_Struct.MotorAngle_Target[1] =  110;
  //	RobotArmData_Struct.MotorAngle_Target[2] = -170; //左侧抓
  RobotArmData_Struct.MotorAngle_Target[2] = 0; //右侧抓
  //角度使能
  RobotArmData_Struct.SCARAflg_U.bit.AngleCtrl_Enabled = 1;
  //延时500使MODBUS通讯可以把数据传输过去
  My_mDelay(500);
  RobotArm_WaitStop();//到达视觉识别的角度
  RobotArmData_Struct.ServoPwmDuty[0] = 1600;//数值与物体间距对应
  My_mDelay(500);

  RecognitionModule_Start(&RecognitionModule_t);//视觉查找红色
  while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
  {
    if(RecognitionModule_t.RecognitionModuleSte == RM_error)
    {
      //识别失败
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


//	switch(color) //记录要抓取的颜色反馈给视觉，并等待视觉回馈
//	{
//		case color_red:
//		color_haveget[color_red]++;
//		RecognitionModule_Start(&RecognitionModule_t);//视觉查找红色
//		while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
//		{
//			if(RecognitionModule_t.RecognitionModuleSte == RM_error)
//			{//识别失败
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
//		ColorRecognitionModule_Start(&RecognitionModule_t);//视觉查找绿色
//		while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
//		{
//			if(RecognitionModule_t.RecognitionModuleSte == RM_error)
//			{//识别失败
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
//		CircleRecognitionModule_Start(&RecognitionModule_t);//视觉查找蓝色
//		while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
//		{
//			if(RecognitionModule_t.RecognitionModuleSte == RM_error)
//			{//识别失败
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
  IdentifyBrick_Get(-20, 25); //识别砖位置进行闭环控制小车底盘位置
//两个值代表想要视觉识别的物体在视觉屏幕的位置(一般给视觉中心位置)，计算后反馈给底盘，底盘移动
  My_mDelay(500);
  RecognitionModule_Stop(&RecognitionModule_t);
  Rotation_Claw(BrickData_Struct.yaw);
//此时物体在对应位置，可以抓取
  //右侧抓
  ChassisSpeed_Set(0, 0);
  switch(color)
  {
  case color_red:
    color_haveget[color_red]++;
    RobotArmData_Struct.ServoPwmDuty[1] = ServoPwmDuty[color_red];//抓取物体
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
    RobotArmData_Struct.ServoPwmDuty[1] = ServoPwmDuty[color_green];//抓取物体
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
    RobotArmData_Struct.ServoPwmDuty[1] = ServoPwmDuty[color_blue];//抓取物体
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
//	//左侧抓
//	RobotArmData_Struct.SCARA_Cartesian[0] = -430;
//	RobotArmData_Struct.SCARA_Cartesian[1] = -60;
//	RobotArmData_Struct.SCARA_Cartesian[2] = -150+(2-color_haveget[color])*Brick_Size;
  //机械臂坐标系，到物体的坐标
  RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 1;//坐标系使能
  My_mDelay(500);
  RobotArm_WaitStop();
  RobotArmData_Struct.ServoPwmDuty[0] = 1250;//抓取物体
  My_mDelay(400);
  //右侧抓--将物体移动到车头方向

  RobotArmData_Struct.SCARA_Cartesian[2] += 100;
  My_mDelay(1000);
  RobotArm_WaitStop();
  RobotArmData_Struct.SCARA_Cartesian[0] = 50;
  RobotArmData_Struct.SCARA_Cartesian[1] = -300;
  RobotArmData_Struct.SCARA_Cartesian[2] = 150;




  //左侧抓
//	RobotArmData_Struct.SCARA_Cartesian[0] = -300;
//	RobotArmData_Struct.SCARA_Cartesian[1] = -60;
//	RobotArmData_Struct.SCARA_Cartesian[2] +=150;
  //延时500使MODBUS通讯可以把数据传输过去
//	RobotArmData_Struct.ServoPwmDuty[1] = 1450;//抓取物体
  My_mDelay(500);
  RobotArm_WaitStop();
  ChassisSpeed_Set(0, 0);
}




uint8_t set_num = 0;
void RobotArmDown(uint8_t color)   // 堆料区0：左  1：中  2：右
{
  set_num = color;
  switch(set_num)
  {
  // 0 1 2 代表从左往右的顺序
  case 0 :
    ChassisSpeed_Set(-100, 0); //当前地方满足要求，需要挪动
    My_mDelay(1000);
    while((LineTrack_Scan(CarDirection_Tail, &Line_Count) & (3 << 3)) != (3 << 3) )
    {
      //假设ls1在左边  <=
      ChassisSpeed_Set(-100, 0);
    }
    ChassisSpeed_Set(0, 0);
    break;
  case 1 :
    ChassisSpeed_Set(0, 0);
    break;
  case 2 :
    ChassisSpeed_Set(100, 0); //当前地方满足要求，需要挪动
    My_mDelay(1000);
    while((LineTrack_Scan(CarDirection_Tail, &Line_Count) & (3 << 3)) != (3 << 3) ) //00011000  车头方向右侧是ls1
    {
      //假设ls1在左边  <=
      ChassisSpeed_Set(100, 0);
    }
    ChassisSpeed_Set(0, 0);
    break;
  default:
    break;
  }
  if(color_haveget[color] > 1) //叠物块
  {
    switch(color) //记录要抓取的颜色反馈给视觉，并等待视觉回馈
    {
    case color_red:
      RecognitionModule_Start(&RecognitionModule_t);//视觉查找红色
      while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
      {
        if(RecognitionModule_t.RecognitionModuleSte == RM_error)
        {
          //识别失败
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
      ColorRecognitionModule_Start(&RecognitionModule_t);//视觉查找绿色
      while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
      {
        if(RecognitionModule_t.RecognitionModuleSte == RM_error)
        {
          //识别失败
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
      CircleRecognitionModule_Start(&RecognitionModule_t);//视觉查找蓝色
      while( RecognitionModule_t.RecognitionModuleSte != RM_succeed )
      {
        if(RecognitionModule_t.RecognitionModuleSte == RM_error)
        {
          //识别失败
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
    IdentifyBrick_Get(-10, -25); //识别砖位置进行闭环控制小车底盘位置
//两个值代表想要视觉识别的物体在视觉屏幕的位置(一般给视觉中心位置)，计算后反馈给底盘，底盘移动
    My_mDelay(500);
    RecognitionModule_Stop(&RecognitionModule_t);
    Rotation_Claw(BrickData_Struct.yaw);
  }
//75 -450 -150
  RobotArmData_Struct.SCARA_Cartesian[0] = 75;
  RobotArmData_Struct.SCARA_Cartesian[1] = -450;
  RobotArmData_Struct.SCARA_Cartesian[2] = -150 + (color_haveget[color] - 1) * Brick_Size;
  //机械臂坐标系，到物体的坐标
  RobotArmData_Struct.SCARAflg_U.bit.XYZ_Enabled = 1;//坐标系使能
  My_mDelay(500);
  RobotArm_WaitStop();
  RobotArmData_Struct.ServoPwmDuty[0] = Claw_S;//松开爪子
  My_mDelay(2000);
  RobotArmData_Struct.SCARA_Cartesian[0] = 50;
  RobotArmData_Struct.SCARA_Cartesian[1] = -300;
  RobotArmData_Struct.SCARA_Cartesian[2] = 150;
  My_mDelay(500);
  RobotArm_WaitStop();

  switch(set_num)
  {
  // 0 1 2 代表从左往右的顺序
  case 0 :
    ChassisSpeed_Set(100, 0); //当前地方满足要求，需要挪动
    My_mDelay(1000);
    while((LineTrack_Scan(CarDirection_Tail, &Line_Count) & (3 << 3)) != (3 << 3) )
    {
      //假设ls1在左边  <=
      ChassisSpeed_Set(150, 0);
    }
    ChassisSpeed_Set(0, 0);
    break;
  case 1 :
    ChassisSpeed_Set(0, 0);
    break;
  case 2 :
    ChassisSpeed_Set(-100, 0); //当前地方满足要求，需要挪动
    My_mDelay(1000);
    while((LineTrack_Scan(CarDirection_Tail, &Line_Count) & (3 << 3)) != (3 << 3) ) //00011000  车头方向右侧是ls1
    {
      //假设ls1在左边  <=
      ChassisSpeed_Set(-150, 0);
    }
    ChassisSpeed_Set(0, 0);
    break;
  default:
    break;
  }
}

// BITWISE 扫描方式，可能会误判
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

//uint8_t LineTrack_Scan(DirectionDef_e Car_Direction, uint8_t *Count)
//{
//  *Count = 0;
//  uint8_t Line_ScanNum = 1;
//  switch (Car_Direction)
//  {
//  case CarDirection_Head:
//    if(LineTracker_Struct.pSignal1->adc_byte[0] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 0;
//    if(LineTracker_Struct.pSignal1->adc_byte[1] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 1;
//    if(LineTracker_Struct.pSignal1->adc_byte[2] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 2;
//    if(LineTracker_Struct.pSignal1->adc_byte[3] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 3;
//    if(LineTracker_Struct.pSignal1->adc_byte[4] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 4;
//    if(LineTracker_Struct.pSignal1->adc_byte[5] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 5;
//    if(LineTracker_Struct.pSignal1->adc_byte[6] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 6;
//    if(LineTracker_Struct.pSignal1->adc_byte[7] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 7;
//    break;
//  case CarDirection_Tail:
//    if(LineTracker_Struct.pSignal3->adc_byte[0] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 0;
//    if(LineTracker_Struct.pSignal3->adc_byte[1] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 1;
//    if(LineTracker_Struct.pSignal3->adc_byte[2] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 2;
//    if(LineTracker_Struct.pSignal3->adc_byte[3] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 3;
//    if(LineTracker_Struct.pSignal3->adc_byte[4] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 4;
//    if(LineTracker_Struct.pSignal3->adc_byte[5] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 5;
//    if(LineTracker_Struct.pSignal3->adc_byte[6] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 6;
//    if(LineTracker_Struct.pSignal3->adc_byte[7] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 7;
//    break;
//  case CarDirection_Left:
//    if(LineTracker_Struct.pSignal2->adc_byte[0] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 0;
//    if(LineTracker_Struct.pSignal2->adc_byte[1] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 1;
//    if(LineTracker_Struct.pSignal2->adc_byte[2] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 2;
//    if(LineTracker_Struct.pSignal2->adc_byte[3] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 3;
//    if(LineTracker_Struct.pSignal2->adc_byte[4] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 4;
//    if(LineTracker_Struct.pSignal2->adc_byte[5] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 5;
//    if(LineTracker_Struct.pSignal2->adc_byte[6] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 6;
//    if(LineTracker_Struct.pSignal2->adc_byte[7] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 7;
//    break;
//  case CarDirection_Right:
//    if(LineTracker_Struct.pSignal4->adc_byte[0] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 0;
//    if(LineTracker_Struct.pSignal4->adc_byte[1] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 1;
//    if(LineTracker_Struct.pSignal4->adc_byte[2] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 2;
//    if(LineTracker_Struct.pSignal4->adc_byte[3] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 3;
//    if(LineTracker_Struct.pSignal4->adc_byte[4] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 4;
//    if(LineTracker_Struct.pSignal4->adc_byte[5] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 5;
//    if(LineTracker_Struct.pSignal4->adc_byte[6] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 6;
//    if(LineTracker_Struct.pSignal4->adc_byte[7] >= LINETRACKER_ADC_THRESHOLD) Line_ScanNum |= 1 << 7;
//    break;
//  default:
//    break;
//  }
//  for(int i = 0; i < 8; i++)
//  {
//    if(Line_ScanNum & (1 << i))  (*Count)++;
//  }
//  if(Line_ScanNum == 1) return 0;
//  return Line_ScanNum;
//}

/****************************
Car_direction:行进方向
Car_side:   从左边绕还是从右边绕  0：左绕    1：右绕
***********************/
void Avoid_Obstacle(DirectionDef_e Car_Direction, uint8_t Car_side)
{
  int16_t speed = 150;
  uint16_t time = 2500;
  ChassisSpeed_Set(0, 0);
  //速度控制
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
  //位置控制
//	//x轴平移
//	PositionXmm_Diff =  dir*Obstacle_length*1.0/2.0+PositionXmm_Old - Read_Position_x_mm();
//	PositionYmm_Diff = PositionYmm_Old  - Read_Position_y_mm();
//	ChassisCoord_Set( PositionXmm_Diff, PositionYmm_Diff, 0 );
//	ChassisCoord_WaitStop();//等待
//	//y轴平移
//	PositionXmm_Diff = dir*Obstacle_length*1.0/2.0+PositionXmm_Old - Read_Position_x_mm();
//	PositionYmm_Diff = Obstacle_width*1.0/2.0 +PositionYmm_Old - Read_Position_y_mm();
//	ChassisCoord_Set( PositionXmm_Diff, PositionYmm_Diff, 0 );
//	ChassisCoord_WaitStop();
//	//x轴平移
//	PositionXmm_Diff = PositionXmm_Old - Read_Position_x_mm();
//	PositionYmm_Diff = Obstacle_width*1.0/2.0 +PositionYmm_Old - Read_Position_y_mm();
//	ChassisCoord_Set( PositionXmm_Diff, PositionYmm_Diff, 0 );
//	ChassisCoord_WaitStop();//等待
////	ChassisStop();
////	PositionXmm_Diff = Read_Position_x_mm();
////	PositionYmm_Diff = Read_Position_y_mm();
}

uint8_t Find_Obstacle(DirectionDef_e Car_Direction)//巡线方向等待障碍物CarDirection_Head
{
  static uint8_t time_cnt = 0;
  uint16_t Distance = 0;

  switch (Car_Direction)
  {
  case CarDirection_Head:

    if(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance2)//防止超声波距离过长时，检测输出无效
    {
//				LineSingle_Tracker(LineTrack_Scan(CarDirection_Head,&Line_Count),CarDirection_Head);
      time_cnt = 0;
      while(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance2 <= 200) //车头方向的超声波距离
      {
        My_mDelay(5);  //此处更改探查到障碍物后的停车距离
        time_cnt++;
        if(time_cnt >= 5) //此处更改探查到障碍物后的停车距离
        {
          LineTracker_Struct.run_mode = Mode_Await;
          time_cnt = 0;
//						ChassisSpeed_Set(0,0);
          return 1;//前方有障碍物，此时条件循迹条件达成
        }
      }
    }
    break;
  case CarDirection_Tail:

    while(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance4)
    {
//				LineSingle_Tracker(LineTrack_Scan(CarDirection_Tail,&Line_Count),CarDirection_Tail);
      time_cnt = 0;
      while(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance4 <= 200) //车头方向的超声波距离
      {
        My_mDelay(5);  //此处更改探查到障碍物后的停车距离
        time_cnt++;
        if(time_cnt >= 5) //此处更改探查到障碍物后的停车距离
        {
          LineTracker_Struct.run_mode = Mode_Await;
          time_cnt = 0;
//					ChassisSpeed_Set(0,0);
          return 1;//前方有障碍物，此时条件循迹条件达成
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
  uint8_t zuo_count = 0, you_count = 0; // 激活灯珠的数量
  float output = 0, error, derivative;
  static float prev_error1, integral1;
  static float prev_error2, integral2;
  // 遍历每个灯珠的状态

  for (int i = 0; i < 8; i++)
  {
    if (Line_TrackScan & (1 << i))   // 检查灯珠是否被激活
    {
      if(i < 4) you_count += (4 - i);
      if(i >= 4) zuo_count += -(i - 4);
    }
  }
  // 计算PID控制输出
  if(Car_Direction == CarDirection_Head)
    error = you_count + zuo_count; // 计算偏差
  if(Car_Direction == CarDirection_Tail)
    error = zuo_count - you_count; // 计算偏差

  switch(Car_Direction)
  {
  case CarDirection_Head:
    integral1 += error; // 积分更新
    float derivative = error - prev_error1; // 微分
//	output = 30.0f * error + 0 * integral + 0.2f * derivative;
    output = 1.5f * error + 0 * integral1 + 0 * derivative;
    prev_error1 = error; // 更新前一个误差
//			ChassisSpeed_Set(output,150);//往前循迹
    ChassisSpeed_Set1(0, 150, output);
    break;

  case CarDirection_Tail:
    integral2 += error; // 积分更新
    derivative = error - prev_error2; // 微分
//	output = 30.0f * error + 0 * integral + 0.2f * derivative;
    output = 4.0f * error + 0 * integral2 + 0.3f * derivative;
    prev_error2 = error; // 更新前一个误差
//			ChassisSpeed_Set(output,-150);//往后循迹
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
uint16_t FindEnd_Left(void)
{
  LineTrack_Scan(CarDirection_Left, &Line_Count);
  while(Line_Count >= 5)
  {
    My_mDelay(5);
    LineTrack_Scan(CarDirection_Left, &Line_Count);
    if(Line_Count >= 5)
      return 1;
    else return 0;
  }
  return 0;
}
uint16_t FindEnd_Right(void)
{
  LineTrack_Scan(CarDirection_Right, &Line_Count);
  while(Line_Count >= 5)
  {
    My_mDelay(5);
    LineTrack_Scan(CarDirection_Right, &Line_Count);
    if(Line_Count >= 5)
      return 1;
    else return 0;
  }
  return 0;
}


uint16_t FindHead_Obstacle(void)
{
  uint8_t time_cnt = 0;
  if(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance1)//防止超声波距离过长时，检测输出无效
  {
    time_cnt = 0;
    while(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance1 <= 200) //车头方向的超声波距离
    {
      My_mDelay(5);  //此处更改探查到障碍物后的停车距离
      time_cnt++;
      if(time_cnt >= 5) //此处更改探查到障碍物后的停车距离
      {
        LineTracker_Struct.run_mode = Mode_Await;
        time_cnt = 0;
        return 1;//前方有障碍物，此时条件循迹条件达成
      }
    }
  }
  return 0;
}
uint16_t FindEnd_Obstacle(void)
{
  uint8_t time_cnt = 0;
  if(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance3)//防止超声波距离过长时，检测输出无效
  {
    while(UltrasonicRanging_S.UltrasonicRanging_UploadData1.DATE.Distance3 <= 200) //车头方向的超声波距离
    {
      My_mDelay(5);  //此处更改探查到障碍物后的停车距离
      time_cnt++;
      if(time_cnt >= 5) //此处更改探查到障碍物后的停车距离
      {
        LineTracker_Struct.run_mode = Mode_Await;

        return 1;//前方有障碍物，此时条件循迹条件达成
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
