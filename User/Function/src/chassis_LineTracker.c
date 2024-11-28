/* 包含头文件 ----------------------------------------------------------------*/
#include "chassis_LineTracker.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有类型定义 --------------------------------------------------------------*/
//typedef enum
//{
//	CorrectiveMode_Slash_A = 0,  //单斜线修正
//	CorrectiveMode_Slash_Double, //双斜线修正（这里的双线指的是十字交叉线）
//}CorrectiveMode_e;
/* 私有变量 ------------------------------------------------------------------*/
pid_t x_pid;
pid_t yaw_pid;
/* 扩展变量 ------------------------------------------------------------------*/
LineTracker_TypeDef LineTracker_Struct;
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
void LineTracker_CorrectiveCtrl(CorrectiveMode_e mode);
static float LineTracker_DistanceCalc(SignalDef_u *pSignal)
{
  float Distance;
  uint8_t num;

  num = 0;
  if(pSignal->bit.ls1 == LineTracker_Struct.active_level) num++;
  if(pSignal->bit.ls2 == LineTracker_Struct.active_level) num++;
  if(pSignal->bit.ls3 == LineTracker_Struct.active_level) num++;
  if(pSignal->bit.ls4 == LineTracker_Struct.active_level) num++;
  if(pSignal->bit.ls5 == LineTracker_Struct.active_level) num++;
  if(pSignal->bit.ls6 == LineTracker_Struct.active_level) num++;
  if(pSignal->bit.ls7 == LineTracker_Struct.active_level) num++;
  if(pSignal->bit.ls8 == LineTracker_Struct.active_level) num++;

  if( ( num >= LINETRACKER_LINE_REFERENCE ) || \
      ( ( pSignal->bit.ls4 == LineTracker_Struct.active_level ) && \
        ( pSignal->bit.ls5 == LineTracker_Struct.active_level ) && \
        ( pSignal->bit.ls3 == pSignal->bit.ls6 ) ) )
  {
    Distance = 7;
  }
  //==========================双线=================================
  else if( ( pSignal->bit.ls3 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls4 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls5 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls6 == LineTracker_Struct.active_level ) ) Distance = 7;
  else if( ( pSignal->bit.ls4 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls5 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls6 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls7 == LineTracker_Struct.active_level ) ) Distance = 9;
  else if( ( pSignal->bit.ls2 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls3 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls4 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls5 == LineTracker_Struct.active_level ) ) Distance = 5;
  else if( ( pSignal->bit.ls5 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls6 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls7 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls8 == LineTracker_Struct.active_level ) ) Distance = 11;
  else if( ( pSignal->bit.ls1 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls2 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls3 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls4 == LineTracker_Struct.active_level ) ) Distance = 3;


  else if( ( pSignal->bit.ls3 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls4 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls5 == LineTracker_Struct.active_level ) ) Distance = 6;
  else if( ( pSignal->bit.ls4 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls5 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls6 == LineTracker_Struct.active_level ) ) Distance = 8;
  else if( ( pSignal->bit.ls2 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls3 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls4 == LineTracker_Struct.active_level ) ) Distance = 4;
  else if( ( pSignal->bit.ls5 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls6 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls7 == LineTracker_Struct.active_level ) ) Distance = 10;
  else if( ( pSignal->bit.ls1 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls2 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls3 == LineTracker_Struct.active_level ) ) Distance = 2;
  else if( ( pSignal->bit.ls6 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls7 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls8 == LineTracker_Struct.active_level ) ) Distance = 12;
  //==========================单线=================================
  else if( ( pSignal->bit.ls4 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls5 == LineTracker_Struct.active_level ) ) Distance = 7;
  else if( ( pSignal->bit.ls3 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls4 == LineTracker_Struct.active_level ) ) Distance = 5;
  else if( ( pSignal->bit.ls5 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls6 == LineTracker_Struct.active_level ) ) Distance = 9;
  else if( ( pSignal->bit.ls2 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls3 == LineTracker_Struct.active_level ) ) Distance = 3;
  else if( ( pSignal->bit.ls6 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls7 == LineTracker_Struct.active_level ) ) Distance = 11;
  else if( ( pSignal->bit.ls1 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls2 == LineTracker_Struct.active_level ) ) Distance = 1;
  else if( ( pSignal->bit.ls7 == LineTracker_Struct.active_level ) && \
           ( pSignal->bit.ls8 == LineTracker_Struct.active_level ) ) Distance = 13;

  else if( pSignal->bit.ls4 == LineTracker_Struct.active_level )     Distance = 6;
  else if( pSignal->bit.ls5 == LineTracker_Struct.active_level )     Distance = 8;
  else if( pSignal->bit.ls3 == LineTracker_Struct.active_level )     Distance = 4;
  else if( pSignal->bit.ls6 == LineTracker_Struct.active_level )     Distance = 10;
  else if( pSignal->bit.ls2 == LineTracker_Struct.active_level )     Distance = 2;
  else if( pSignal->bit.ls7 == LineTracker_Struct.active_level )     Distance = 12;
  else if( pSignal->bit.ls1 == LineTracker_Struct.active_level )     Distance = 0;
  else if( pSignal->bit.ls8 == LineTracker_Struct.active_level )     Distance = 14;

  return (Distance * RL_LENGTH_FAC / 14.0f);
}

//static void LineTracker_ChassisPostureCalc(SignalDef_u *pHeadSignal, SignalDef_u *pTailSignal)
void LineTracker_ChassisPostureCalc(SignalDef_u *pHeadSignal, SignalDef_u *pTailSignal)
{
  float  C;

  if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction < CarDirection_Left) // 前后
  {
    C = (LINETRACKER_WHEELTRACK - RL_LENGTH_FAC) / 2.0f; // （轮距-循迹条长度）/ 2
  }
  else // 左右
  {
    C = (LINETRACKER_WHEELBASE - RL_LENGTH_FAC) / 2.0f; // （轴距-循迹条长度）/ 2
  }
  if((pHeadSignal->byte != 0x00) && (pHeadSignal->byte != 0xff))
  {
    //计算黑线与小车头部最左侧边沿的距离
    LineTracker_Struct.Line1 = RL_LENGTH_FAC - LineTracker_DistanceCalc(pHeadSignal) + C;
  }
  if((pTailSignal->byte != 0x00) && (pTailSignal->byte != 0xff))
  {
    //计算黑线与小车尾部左侧边沿的距离
    LineTracker_Struct.Line2 = LineTracker_DistanceCalc(pTailSignal) + C;
  }
  LineTracker_Struct.D_Offs = ( RL_LENGTH_FAC + (2.0f * C) - ( LineTracker_Struct.Line1 + LineTracker_Struct.Line2 ) ) / 2.0f;
  if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction < CarDirection_Left)
  {
    LineTracker_Struct.Alpha = atan( ( LineTracker_Struct.Line1 - LineTracker_Struct.Line2 ) / B_LENGTH ) * RAD_TO_ANGLE;
  }
  else
  {
    LineTracker_Struct.Alpha = atan( ( LineTracker_Struct.Line1 - LineTracker_Struct.Line2 ) / A_LENGTH ) * RAD_TO_ANGLE;
  }
}
//static void LineTracker_CorrectiveCtrl(CorrectiveMode_e mode)
void LineTracker_CorrectiveCtrl(CorrectiveMode_e mode)
{
  switch(mode)
  {
  case CorrectiveMode_Slash_A://单斜线修正
    if( ( LineTracker_Struct.pSignal1 != 0 ) && \
        ( LineTracker_Struct.pSignal3 != 0 ) && \
        ( LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction < CarDirection_Left ) ) // 前后
    {
      LineTracker_ChassisPostureCalc(LineTracker_Struct.pSignal1, LineTracker_Struct.pSignal3);
    }
    else if( ( LineTracker_Struct.pSignal2 != 0 ) && \
             ( LineTracker_Struct.pSignal4 != 0 ) && \
             ( LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction > CarDirection_Tail ) ) // 左右
    {
      LineTracker_ChassisPostureCalc(LineTracker_Struct.pSignal2, LineTracker_Struct.pSignal4);
    }
    else
    {
      LineTracker_Struct.D_Offs = 0;
      LineTracker_Struct.Alpha = 0;
    }
    LineTracker_Struct.x_axis_out = x_pid.f_pid_calc( &x_pid, LineTracker_Struct.D_Offs, 0.0f );
    LineTracker_Struct.yaw_out = yaw_pid.f_pid_calc( &yaw_pid, LineTracker_Struct.Alpha, 0.0f );
    break;
  case CorrectiveMode_Slash_Double: //双斜线修正（这里的双线指的是十字交叉线）
    if( ( LineTracker_Struct.pSignal1 != 0 ) && \
        ( LineTracker_Struct.pSignal3 != 0 ) && \
        ( LineTracker_Struct.pSignal2 != 0 ) && \
        ( LineTracker_Struct.pSignal4 != 0 ) )
    {
      LineTracker_ChassisPostureCalc(LineTracker_Struct.pSignal1, LineTracker_Struct.pSignal3);
      LineTracker_Struct.x_axis_out = x_pid.f_pid_calc( &x_pid, LineTracker_Struct.D_Offs, 0.0f );
      LineTracker_Struct.yaw_out = yaw_pid.f_pid_calc( &yaw_pid, LineTracker_Struct.Alpha, 0.0f );
      LineTracker_ChassisPostureCalc(LineTracker_Struct.pSignal2, LineTracker_Struct.pSignal4);
      LineTracker_Struct.y_axis_out = x_pid.f_pid_calc( &x_pid, LineTracker_Struct.D_Offs, 0.0f );
      LineTracker_Struct.yaw_out += yaw_pid.f_pid_calc( &yaw_pid, LineTracker_Struct.Alpha, 0.0f );
      LineTracker_Struct.yaw_out = LineTracker_Struct.yaw_out / 2;
    }
    break;
  }
}
static void LineTracker_OpenLoopWalkDelay(void)
{
  //延时开环控制
  LineTracker_Struct.pLineTracker_ParaStruct->TimeMem += (1000 / LineTracker_Struct.CtrlFreqHZ);
  if(LineTracker_Struct.pLineTracker_ParaStruct->TimeMem >= LineTracker_Struct.pLineTracker_ParaStruct->SetVal)
  {
    LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
    LineTracker_Struct.run_mode = Mode_Await;
  }
  if(LineTracker_Struct.run_mode == Mode_Await)
  {
    LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
    Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, 0);
    LineTracker_Struct.y_axis_out = 0;
    LineTracker_Struct.x_axis_out = 0;
    LineTracker_Struct.yaw_out = 0;
  }
  else
  {
    LineTracker_Struct.x_axis_out = LineTracker_Struct.pLineTracker_ParaStruct->x_axis_set;
    LineTracker_Struct.y_axis_out = LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set;
    LineTracker_Struct.yaw_out = LineTracker_Struct.pLineTracker_ParaStruct->yaw_set;
  }
}

void LineTracker_Single(void)
{
  uint8_t num;



}
static void LineTracker_LineNum(void)
{
  //线条巡线
  uint8_t num;

  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    //================循迹纠正=====================
    if( LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag )
    {
      LineTracker_CorrectiveCtrl(CorrectiveMode_Slash_A);
    }
    //================循迹读横线=======================
    num = 0;
    if(LineTracker_Struct.pHeadSignal->bit.ls1 == LineTracker_Struct.active_level) num++;
    if(LineTracker_Struct.pHeadSignal->bit.ls2 == LineTracker_Struct.active_level) num++;
    if(LineTracker_Struct.pHeadSignal->bit.ls3 == LineTracker_Struct.active_level) num++;
    if(LineTracker_Struct.pHeadSignal->bit.ls4 == LineTracker_Struct.active_level) num++;
    if(LineTracker_Struct.pHeadSignal->bit.ls5 == LineTracker_Struct.active_level) num++;
    if(LineTracker_Struct.pHeadSignal->bit.ls6 == LineTracker_Struct.active_level) num++;
    if(LineTracker_Struct.pHeadSignal->bit.ls7 == LineTracker_Struct.active_level) num++;
    if(LineTracker_Struct.pHeadSignal->bit.ls8 == LineTracker_Struct.active_level) num++;

    switch(LineTracker_Struct.pLineTracker_ParaStruct->run_state)
    {
    case 0: //寻找一条横线
    {
#if 0
      if( num >= LINETRACKER_LINE_REFERENCE )
#else
      if( ( num >= LINETRACKER_LINE_REFERENCE ) \
          && ( ( LineTracker_Struct.pHeadSignal->bit.ls8 == LineTracker_Struct.active_level ) \
               || ( LineTracker_Struct.pHeadSignal->bit.ls1 == LineTracker_Struct.active_level ) ) )
#endif
      {
        LineTracker_Struct.pLineTracker_ParaStruct->TimeMem++;
        if( LineTracker_Struct.pLineTracker_ParaStruct->TimeMem >= 5 )
        {
          LineTracker_Struct.line_cnt++;
          LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
          LineTracker_Struct.pLineTracker_ParaStruct->run_state = 1; //在横线上面
          if(LineTracker_Struct.pLineTracker_ParaStruct->SetVal == 0)
          {
            //设置为刚循迹到白线就停止
            if(LineTracker_Struct.line_cnt == LineTracker_Struct.pLineTracker_ParaStruct->line_num)
            {
              //到达设置的白线数
              LineTracker_Struct.run_mode = Mode_Await;
            }
            else if(LineTracker_Struct.line_cnt == (LineTracker_Struct.pLineTracker_ParaStruct->line_num - 1))
            {
              //根据减速度进行减速
              if(LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set > LINETRACKER_SPEED_STOP)
                LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = LINETRACKER_SPEED_STOP;
            }
          }
          else
          {
            if(LineTracker_Struct.pLineTracker_ParaStruct->line_num == 1)
            {
              //根据减速度进行减速
              if(LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set > LINETRACKER_SPEED_STOP)
              {
                LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = LINETRACKER_SPEED_STOP;
                LineTracker_Struct.Speed_TrackerStruct.down_acc = LINETRACKER_SPEED_MAX * 4;
              }
            }
            else if(LineTracker_Struct.line_cnt == (LineTracker_Struct.pLineTracker_ParaStruct->line_num - 1))
            {
              //根据减速度进行减速
              if(LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set > LINETRACKER_SPEED_STOP)
                LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = LINETRACKER_SPEED_STOP;
            }
          }
        }
      }
      else
      {
        LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
      }
    }
    break;
    case 1: //屏蔽当前这个白线
    {
      LineTracker_Struct.x_axis_out = 0;
      LineTracker_Struct.yaw_out = 0;
      if( num < LINETRACKER_LINE_REFERENCE )
      {
        LineTracker_Struct.pLineTracker_ParaStruct->TimeMem += (1000 / LineTracker_Struct.CtrlFreqHZ);
        if(LineTracker_Struct.pLineTracker_ParaStruct->TimeMem >= 50)
        {
          LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
          if(LineTracker_Struct.line_cnt == LineTracker_Struct.pLineTracker_ParaStruct->line_num)
          {
            LineTracker_Struct.pLineTracker_ParaStruct->run_state = 3;//准备停车
          }
          else
          {
            LineTracker_Struct.pLineTracker_ParaStruct->run_state = 2;//添加屏蔽白线的条件
          }
        }
      }
      else
      {
        LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
      }
    }
    break;
    case 2: //冗余的屏蔽白线程序
    {
#if 0
      SignalDef_u *pSignal;

      if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction == CarDirection_Head)
        pSignal = LineTracker_Struct.pSignal3;
      else if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction == CarDirection_Tail)
        pSignal = LineTracker_Struct.pSignal1;
      else if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction == CarDirection_Left)
        pSignal = LineTracker_Struct.pSignal4;
      else if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction == CarDirection_Right)
        pSignal = LineTracker_Struct.pSignal2;
      num = 0;
      if(pSignal->bit.ls1 == LineTracker_Struct.active_level) num++;
      if(pSignal->bit.ls2 == LineTracker_Struct.active_level) num++;
      if(pSignal->bit.ls3 == LineTracker_Struct.active_level) num++;
      if(pSignal->bit.ls4 == LineTracker_Struct.active_level) num++;
      if(pSignal->bit.ls5 == LineTracker_Struct.active_level) num++;
      if(pSignal->bit.ls6 == LineTracker_Struct.active_level) num++;
      if(pSignal->bit.ls7 == LineTracker_Struct.active_level) num++;
      if(pSignal->bit.ls8 == LineTracker_Struct.active_level) num++;
      if(num >= LINETRACKER_LINE_REFERENCE)
      {
        LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
      }
#elif 0
      if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction < CarDirection_Left)
      {
        if( ( LineTracker_Struct.pSignal2->byte != 0 ) || \
            ( LineTracker_Struct.pSignal4->byte != 0 ) )
        {
          LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
        }
      }
      else
      {
        if( ( LineTracker_Struct.pSignal1->byte != 0 ) || \
            ( LineTracker_Struct.pSignal3->byte != 0 ) )
        {
          LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
        }
      }
#else
      LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
#endif
    }
    break;
    case 3: //进行小车停止（小车停在白线上）
    {
      if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction < CarDirection_Left)
      {
        if( ( LineTracker_Struct.pSignal2->bit.ls4 == LineTracker_Struct.active_level ) || \
            ( LineTracker_Struct.pSignal4->bit.ls4 == LineTracker_Struct.active_level ) || \
            ( LineTracker_Struct.pSignal2->bit.ls5 == LineTracker_Struct.active_level ) || \
            ( LineTracker_Struct.pSignal4->bit.ls5 == LineTracker_Struct.active_level ) )
        {
          LineTracker_Struct.run_mode = Mode_Await;
        }
      }
      else if( ( LineTracker_Struct.pSignal1->bit.ls4 == LineTracker_Struct.active_level ) || \
               ( LineTracker_Struct.pSignal3->bit.ls4 == LineTracker_Struct.active_level ) || \
               ( LineTracker_Struct.pSignal1->bit.ls5 == LineTracker_Struct.active_level ) || \
               ( LineTracker_Struct.pSignal3->bit.ls5 == LineTracker_Struct.active_level ) )
      {
        LineTracker_Struct.run_mode = Mode_Await;
      }
    }
    break;
    }
    if(LineTracker_Struct.run_mode == Mode_Await)
    {
      LineTracker_Struct.line_cnt = 0;
      LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, 0);
      LineTracker_Struct.Speed_TrackerStruct.down_acc = LINETRACKER_SPEED_DOWN_ACC;
      LineTracker_Struct.y_axis_out = 0;
      LineTracker_Struct.x_axis_out = 0;
      LineTracker_Struct.yaw_out = 0;
    }
    else
    {
      //================加减速=======================
      LineTracker_Struct.y_axis_out = Speed_Tracker_Capture_Goal(&LineTracker_Struct.Speed_TrackerStruct, LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set);
    }
  }
}
static void LineTracker_Delay(void)
{
  //延时巡线
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    //================循迹纠正=====================
    if( LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag )
    {
      LineTracker_CorrectiveCtrl(CorrectiveMode_Slash_A);
    }
    LineTracker_Struct.pLineTracker_ParaStruct->TimeMem += (1000 / LineTracker_Struct.CtrlFreqHZ);
    if(LineTracker_Struct.pLineTracker_ParaStruct->TimeMem >= LineTracker_Struct.pLineTracker_ParaStruct->SetVal)
    {
      LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
      LineTracker_Struct.run_mode = Mode_Await;
    }
    else if(LineTracker_Struct.y_axis_out > LINETRACKER_SPEED_STOP)
    {
      if( ( ( LineTracker_Struct.y_axis_out - LINETRACKER_SPEED_STOP ) * 1000 / LINETRACKER_SPEED_DOWN_ACC ) >=
          ( LineTracker_Struct.pLineTracker_ParaStruct->SetVal - LineTracker_Struct.pLineTracker_ParaStruct->TimeMem ) )
      {
        LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = LINETRACKER_SPEED_STOP;
      }
    }
    if(LineTracker_Struct.run_mode == Mode_Await)
    {
      LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, 0);
      LineTracker_Struct.y_axis_out = 0;
      LineTracker_Struct.x_axis_out = 0;
      LineTracker_Struct.yaw_out = 0;
    }
    else
    {
      //================加减速=======================
      LineTracker_Struct.y_axis_out = Speed_Tracker_Capture_Goal( &LineTracker_Struct.Speed_TrackerStruct, \
                                      LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set );
    }
  }
}
static void LineTracker_Condition(void)
{
  //条件巡线
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    //================循迹纠正=====================
    if( LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag )
    {
      LineTracker_CorrectiveCtrl(CorrectiveMode_Slash_A); // 默认单斜线纠正
    }
    if(LineTracker_Struct.pLineTracker_ParaStruct->ReadData_func == 0)
    {
      //并没有条件函数立即停止
      LineTracker_Struct.run_mode = Mode_Await;
    }
    else if(LineTracker_Struct.pLineTracker_ParaStruct->ReadData_func() == LineTracker_Struct.pLineTracker_ParaStruct->SetVal)
    {
      //达到条件立即停止
      LineTracker_Struct.run_mode = Mode_Await;
    }
    if(LineTracker_Struct.run_mode == Mode_Await)
    {
      LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, 0);
      LineTracker_Struct.y_axis_out = 0;
      LineTracker_Struct.x_axis_out = 0;
      LineTracker_Struct.yaw_out = 0;
    }
    else
    {
      //================加减速=======================
      LineTracker_Struct.y_axis_out = Speed_Tracker_Capture_Goal( &LineTracker_Struct.Speed_TrackerStruct, \
                                      LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set );
    }
  }
}
static void LineTracker_SituAdjust(void)
{
  //原地矫正
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    if(LineTracker_Struct.pLineTracker_ParaStruct->TimeMem > (1000 / LineTracker_Struct.CtrlFreqHZ))
    {
      LineTracker_Struct.pLineTracker_ParaStruct->TimeMem -= (1000 / LineTracker_Struct.CtrlFreqHZ);
    }
    else
    {
      LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
    }
    if(LineTracker_Struct.pLineTracker_ParaStruct->TimeMem == 0)
    {
      LineTracker_Struct.run_mode = Mode_Await;
    }
    else
    {
      if(LineTracker_Struct.pLineTracker_ParaStruct->run_state == 0)
      {
        LineTracker_CorrectiveCtrl(CorrectiveMode_Slash_Double);
      }
      else
      {
        LineTracker_CorrectiveCtrl(CorrectiveMode_Slash_A);
      }
    }
    if(LineTracker_Struct.run_mode == Mode_Await)
    {
      LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, 0);
      LineTracker_Struct.y_axis_out = 0;
      LineTracker_Struct.x_axis_out = 0;
      LineTracker_Struct.yaw_out = 0;
    }
  }
}
static void LineTracker_Wheel_90(void)
{
  //90度转弯
  int32_t location_sub;
  int32_t need_down_location;
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    switch(LineTracker_Struct.pLineTracker_ParaStruct->run_state)
    {
    case 0:  //开始
    {
      if(LineTracker_Struct.pLineTracker_ParaStruct->SetVal != 0)
      {
        //前进一段路
        LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = LINETRACKER_SPEED_BEFORE_TURNING;
        if(LINETRACKER_SPEED_BEFORE_TURNING < LINETRACKER_SPEED_START)
          Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LINETRACKER_SPEED_BEFORE_TURNING);
        else
          Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LINETRACKER_SPEED_START);
        LineTracker_Struct.pLineTracker_ParaStruct->run_state = 1;
      }
      else
      {
        //直接进行转弯
        LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = 0;
        LineTracker_Struct.x_axis_out = 0;
        LineTracker_Struct.y_axis_out = 0;
        LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 100;
        LineTracker_Struct.pLineTracker_ParaStruct->run_state = 2;
      }
    }
    break;
    case 1:  //前进动作
    {
      LineTracker_Struct.pLineTracker_ParaStruct->TimeMem += LineTracker_Struct.y_axis_out;
      location_sub = ( LineTracker_Struct.pLineTracker_ParaStruct->SetVal * LineTracker_Struct.CtrlFreqHZ - \
                       LineTracker_Struct.pLineTracker_ParaStruct->TimeMem ) / LineTracker_Struct.CtrlFreqHZ;
      if(location_sub <= 0)
      {
        LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
        LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = 0;
        LineTracker_Struct.x_axis_out = 0;
        LineTracker_Struct.y_axis_out = 0;
        LineTracker_Struct.pLineTracker_ParaStruct->run_state = 2;
      }
      else
      {
        if(LineTracker_Struct.y_axis_out > LINETRACKER_SPEED_STOP)
        {
          //计算需要的减速位移(浮点运算)
          //位移等于二分之一的加速度乘以时间的平方
          need_down_location = (int32_t)( (float)LineTracker_Struct.y_axis_out * \
                                          (float)LineTracker_Struct.y_axis_out * \
                                          (float)(0.5f / LINETRACKER_SPEED_DOWN_ACC));
          if(location_sub < need_down_location)
          {
            LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = LINETRACKER_SPEED_STOP;
          }
        }
        //================循迹纠正=====================
        if( LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag )
        {
          LineTracker_CorrectiveCtrl(CorrectiveMode_Slash_A);
        }
        //================加减速=======================
        LineTracker_Struct.y_axis_out = Speed_Tracker_Capture_Goal( &LineTracker_Struct.Speed_TrackerStruct, \
                                        LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set );
      }
    }
    break;
    case 2://进行旋转
      LineTracker_Struct.pLineTracker_ParaStruct->TimeMem += (1000 / LineTracker_Struct.CtrlFreqHZ);
      if(LineTracker_Struct.pLineTracker_ParaStruct->TimeMem >= 100)
      {
        LineTracker_Struct.Speed_TrackerStruct.up_acc = LINETRACKER_ANGULAR_SPEED_UP_ACC;
        LineTracker_Struct.Speed_TrackerStruct.down_acc = LINETRACKER_ANGULAR_SPEED_DOWN_ACC;
        if(abs(LineTracker_Struct.pLineTracker_ParaStruct->yaw_set) < LINETRACKER_ANGULAR_SPEED_START)
        {
          Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LineTracker_Struct.pLineTracker_ParaStruct->yaw_set);
        }
        else
        {
          if(LineTracker_Struct.pLineTracker_ParaStruct->yaw_set < 0)
            Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, -LINETRACKER_ANGULAR_SPEED_START);
          else
            Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LINETRACKER_ANGULAR_SPEED_START);
        }
        LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
        LineTracker_Struct.yaw_out = 0;
        LineTracker_Struct.pLineTracker_ParaStruct->SetVal = 0;//这里这个参数用来当作角度计算
        LineTracker_Struct.pLineTracker_ParaStruct->run_state = 3;
      }
      break;
    case 3: //转弯动作，屏蔽当前的线
      if( ( LineTracker_Struct.pHeadSignal->bit.ls3 != LineTracker_Struct.active_level ) &&
          ( LineTracker_Struct.pHeadSignal->bit.ls4 != LineTracker_Struct.active_level ) &&
          ( LineTracker_Struct.pHeadSignal->bit.ls5 != LineTracker_Struct.active_level ) &&
          ( LineTracker_Struct.pHeadSignal->bit.ls6 != LineTracker_Struct.active_level ) )
      {
        LineTracker_Struct.pLineTracker_ParaStruct->TimeMem += (1000 / LineTracker_Struct.CtrlFreqHZ);
        if(LineTracker_Struct.pLineTracker_ParaStruct->TimeMem >= 100)
        {
          LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
          LineTracker_Struct.pLineTracker_ParaStruct->run_state = 4;
        }
      }
      else
      {
        LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
      }
      break;
    case 4://转弯动作，寻找下一条线
      if( ( LineTracker_Struct.pHeadSignal->bit.ls4 == LineTracker_Struct.active_level ) && \
          ( LineTracker_Struct.pHeadSignal->bit.ls5 == LineTracker_Struct.active_level ) )
      {
        LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
        LineTracker_Struct.run_mode = Mode_Await;
      }
      break;
    }
    if(LineTracker_Struct.pLineTracker_ParaStruct->run_state > 2)
    {
      //进行旋转
      LineTracker_Struct.pLineTracker_ParaStruct->SetVal += abs(LineTracker_Struct.yaw_out);
      location_sub = ( ( 90 * LineTracker_Struct.CtrlFreqHZ ) - \
                       LineTracker_Struct.pLineTracker_ParaStruct->SetVal ) / LineTracker_Struct.CtrlFreqHZ;
      if(location_sub <= (-LINETRACKER_ANGLE_OFFSET_STOP))
      {
        LineTracker_Struct.run_mode = Mode_Await;
      }
      else
      {
        if(abs(LineTracker_Struct.yaw_out) > LINETRACKER_ANGULAR_SPEED_STOP)
        {
          //计算需要的减速位移(浮点运算)
          //位移等于二分之一的加速度乘以时间的平方
          need_down_location = (int32_t)( (float)abs(LineTracker_Struct.yaw_out) * \
                                          (float)abs(LineTracker_Struct.yaw_out) * \
                                          (float)(0.5f / LINETRACKER_ANGULAR_SPEED_DOWN_ACC));
          if(location_sub < need_down_location)
          {
            if(LineTracker_Struct.pLineTracker_ParaStruct->yaw_set < 0)
              LineTracker_Struct.pLineTracker_ParaStruct->yaw_set = -LINETRACKER_ANGULAR_SPEED_STOP;
            else
              LineTracker_Struct.pLineTracker_ParaStruct->yaw_set = LINETRACKER_ANGULAR_SPEED_STOP;
          }
        }
        LineTracker_Struct.yaw_out = Speed_Tracker_Capture_Goal( &LineTracker_Struct.Speed_TrackerStruct, \
                                     LineTracker_Struct.pLineTracker_ParaStruct->yaw_set );
      }
    }
    if(LineTracker_Struct.run_mode == Mode_Await)
    {
      LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
      LineTracker_Struct.Speed_TrackerStruct.up_acc = LINETRACKER_SPEED_UP_ACC;
      LineTracker_Struct.Speed_TrackerStruct.down_acc = LINETRACKER_SPEED_DOWN_ACC;
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, 0);
      LineTracker_Struct.y_axis_out = 0;
      LineTracker_Struct.x_axis_out = 0;
      LineTracker_Struct.yaw_out = 0;
    }
  }
}
static void LineTracker_RotateAngle(void)
{
  //旋转角度
  int32_t location_sub;
  int32_t need_down_location;

  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    switch(LineTracker_Struct.pLineTracker_ParaStruct->run_state)
    {
    case 0:
    {
      LineTracker_Struct.Speed_TrackerStruct.up_acc = LINETRACKER_ANGULAR_SPEED_UP_ACC;
      LineTracker_Struct.Speed_TrackerStruct.down_acc = LINETRACKER_ANGULAR_SPEED_DOWN_ACC;
      if(abs(LineTracker_Struct.pLineTracker_ParaStruct->yaw_set) < LINETRACKER_ANGULAR_SPEED_START)
      {
        Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LineTracker_Struct.pLineTracker_ParaStruct->yaw_set);
      }
      else
      {
        if(LineTracker_Struct.pLineTracker_ParaStruct->yaw_set < 0)
          Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, -LINETRACKER_ANGULAR_SPEED_START);
        else
          Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LINETRACKER_ANGULAR_SPEED_START);
      }
      LineTracker_Struct.y_axis_out = 0;
      LineTracker_Struct.x_axis_out = 0;
      LineTracker_Struct.yaw_out = 0;
      LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;//这里这个参数用来当作角度计算
      LineTracker_Struct.pLineTracker_ParaStruct->run_state = 1;
    }
    break;
    case 1:
    {
      LineTracker_Struct.pLineTracker_ParaStruct->TimeMem += abs(LineTracker_Struct.yaw_out);
      location_sub = ( LineTracker_Struct.pLineTracker_ParaStruct->SetVal * LineTracker_Struct.CtrlFreqHZ - \
                       LineTracker_Struct.pLineTracker_ParaStruct->TimeMem ) / LineTracker_Struct.CtrlFreqHZ;
      if(location_sub <= 0)
      {
        LineTracker_Struct.run_mode = Mode_Await;
      }
      else
      {
        if(abs(LineTracker_Struct.yaw_out) > LINETRACKER_ANGULAR_SPEED_STOP)
        {
          //计算需要的减速位移(浮点运算)
          //位移等于二分之一的加速度乘以时间的平方
          need_down_location = (int32_t)( (float)abs(LineTracker_Struct.yaw_out) * \
                                          (float)abs(LineTracker_Struct.yaw_out) * \
                                          (float)(0.5f / LINETRACKER_ANGULAR_SPEED_DOWN_ACC));
          if(location_sub < need_down_location)
          {
            if(LineTracker_Struct.pLineTracker_ParaStruct->yaw_set < 0)
              LineTracker_Struct.pLineTracker_ParaStruct->yaw_set = -LINETRACKER_ANGULAR_SPEED_STOP;
            else
              LineTracker_Struct.pLineTracker_ParaStruct->yaw_set = LINETRACKER_ANGULAR_SPEED_STOP;
          }
        }
        LineTracker_Struct.yaw_out = Speed_Tracker_Capture_Goal( &LineTracker_Struct.Speed_TrackerStruct, \
                                     LineTracker_Struct.pLineTracker_ParaStruct->yaw_set );
      }
    }
    break;
    }
    if(LineTracker_Struct.run_mode == Mode_Await)
    {
      LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
      LineTracker_Struct.Speed_TrackerStruct.up_acc = LINETRACKER_SPEED_UP_ACC;
      LineTracker_Struct.Speed_TrackerStruct.down_acc = LINETRACKER_SPEED_DOWN_ACC;
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, 0);
      LineTracker_Struct.y_axis_out = 0;
      LineTracker_Struct.x_axis_out = 0;
      LineTracker_Struct.yaw_out = 0;
    }
  }
}
static void LineTracker_Encoder(void)
{
  //编码器巡线
  int32_t location_sub;
  int32_t need_down_location;

  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    switch(LineTracker_Struct.pLineTracker_ParaStruct->run_state)
    {
    case 0:
      if(LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set < LINETRACKER_SPEED_START)
        Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set);
      else
        Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LINETRACKER_SPEED_START);
      LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = 0;
      if(LineTracker_Struct.pLineTracker_ParaStruct->pTotal_ecd != 0)
      {
        if( ( LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction == CarDirection_Head ) || \
            ( LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction == CarDirection_Right ) )
        {
          LineTracker_Struct.pLineTracker_ParaStruct->SetVal = *LineTracker_Struct.pLineTracker_ParaStruct->pTotal_ecd + LineTracker_Struct.pLineTracker_ParaStruct->SetVal;
        }
        else
        {
          LineTracker_Struct.pLineTracker_ParaStruct->SetVal = *LineTracker_Struct.pLineTracker_ParaStruct->pTotal_ecd - LineTracker_Struct.pLineTracker_ParaStruct->SetVal;
        }
      }
      LineTracker_Struct.pLineTracker_ParaStruct->run_state = 1;
      break;
    case 1:
      if(LineTracker_Struct.pLineTracker_ParaStruct->pTotal_ecd != 0)
      {
        if( ( LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction == CarDirection_Head ) || \
            ( LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction == CarDirection_Right ) )
        {
          location_sub = LineTracker_Struct.pLineTracker_ParaStruct->SetVal - *LineTracker_Struct.pLineTracker_ParaStruct->pTotal_ecd;
        }
        else
        {
          location_sub = *LineTracker_Struct.pLineTracker_ParaStruct->pTotal_ecd - LineTracker_Struct.pLineTracker_ParaStruct->SetVal;
        }
      }
      else
      {
        LineTracker_Struct.pLineTracker_ParaStruct->TimeMem += LineTracker_Struct.y_axis_out;
        location_sub = ( ( LineTracker_Struct.pLineTracker_ParaStruct->SetVal * LineTracker_Struct.CtrlFreqHZ ) - \
                         LineTracker_Struct.pLineTracker_ParaStruct->TimeMem ) / LineTracker_Struct.CtrlFreqHZ;
      }
      if(location_sub <= 0)
      {
        LineTracker_Struct.run_mode = Mode_Await;
      }
      else
      {
        if(LineTracker_Struct.y_axis_out > LINETRACKER_SPEED_STOP)
        {
          //计算需要的减速位移(浮点运算)
          //位移等于二分之一的加速度乘以时间的平方
          need_down_location = (int32_t)( (float)(LineTracker_Struct.y_axis_out) * \
                                          (float)(LineTracker_Struct.y_axis_out) * \
                                          (float)(0.5f / LINETRACKER_SPEED_DOWN_ACC));
          if(location_sub < need_down_location)
          {
            LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = 0;
          }
        }
        //================循迹纠正=====================
        if( LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag )
        {
          LineTracker_CorrectiveCtrl(CorrectiveMode_Slash_A);
        }
        //================加减速=======================
        LineTracker_Struct.y_axis_out = Speed_Tracker_Capture_Goal( &LineTracker_Struct.Speed_TrackerStruct, \
                                        LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set );
      }
      break;
    }
    if(LineTracker_Struct.run_mode == Mode_Await)
    {
      LineTracker_Struct.pLineTracker_ParaStruct->run_state = 0;
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, 0);
      LineTracker_Struct.y_axis_out = 0;
      LineTracker_Struct.x_axis_out = 0;
      LineTracker_Struct.yaw_out = 0;
    }
  }
}



void LineTracker_Scan(void)
{
  switch(LineTracker_Struct.run_mode)
  {
  case Mode_Stop:                   //停止
    break;
  case Mode_Await:                  //等待
    free(LineTracker_Struct.pLineTracker_ParaStruct);
    LineTracker_Struct.pLineTracker_ParaStruct = 0;
    LineTracker_Struct.run_mode = Mode_Stop;
    break;
  case Mode_OpenLoopWalk_Delay:     //延时开环控制
    LineTracker_OpenLoopWalkDelay();
    break;
  case Mode_LineTracker_LineNum:    //线条巡线
    LineTracker_LineNum();
    break;
  case Mode_LineTracker_Delay:      //延时巡线
    LineTracker_Delay();
    break;
  case Mode_LineTracker_Condition:  //条件巡线
    LineTracker_Condition();
    break;
  case Mode_LineTracker_SituAdjust: //原地矫正
    LineTracker_SituAdjust();
    break;
  case Mode_LineTracker_Wheel_90:   //90度转弯
    LineTracker_Wheel_90();
    break;
  case Mode_Rotate_Angle:           //旋转角度
    LineTracker_RotateAngle();
    break;
  case Mode_LineTracker_Encoder:    //编码器巡线
    LineTracker_Encoder();
    break;
  case Mode_LineSingleTracker:    //编码器巡线
    LineTracker_Single();
    break;
  }
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    LineTracker_Struct.yaw_PracticalOut = LineTracker_Struct.yaw_out;
    if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction < CarDirection_Left)
    {
      LineTracker_Struct.x_axis_PracticalOut = LineTracker_Struct.x_axis_out;
      LineTracker_Struct.y_axis_PracticalOut = LineTracker_Struct.y_axis_out;
      if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction == CarDirection_Tail)
      {
        LineTracker_Struct.y_axis_PracticalOut = -LineTracker_Struct.y_axis_PracticalOut;
      }
    }
    else
    {
      LineTracker_Struct.y_axis_PracticalOut = LineTracker_Struct.x_axis_out;
      LineTracker_Struct.x_axis_PracticalOut = LineTracker_Struct.y_axis_out;
      if(LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction == CarDirection_Left)
      {
        LineTracker_Struct.x_axis_PracticalOut = -LineTracker_Struct.x_axis_PracticalOut;
      }
    }
  }
  else
  {
    if(LineTracker_Struct.run_mode > Mode_Await)
    {
      LineTracker_Struct.run_mode = Mode_Await;
      LineTracker_Struct.line_cnt = 0;
    }
    LineTracker_Struct.y_axis_out = 0;
    LineTracker_Struct.x_axis_out = 0;
    LineTracker_Struct.yaw_out = 0;
    LineTracker_Struct.yaw_PracticalOut = 0;
    LineTracker_Struct.y_axis_PracticalOut = 0;
    LineTracker_Struct.x_axis_PracticalOut = 0;
  }
}
void LineTracker_Init( uint8_t active_level,
                       uint16_t CtrlFreqHZ,
                       uint8_t *pSignal1,
                       uint8_t *pSignal2,
                       uint8_t *pSignal3,
                       uint8_t *pSignal4 )
{
  memset(&LineTracker_Struct, 0, sizeof(LineTracker_TypeDef));

  LineTracker_Struct.active_level = active_level;
  LineTracker_Struct.CtrlFreqHZ = CtrlFreqHZ;
  LineTracker_Struct.pSignal1 = (SignalDef_u*)pSignal1;
  LineTracker_Struct.pSignal2 = (SignalDef_u*)pSignal2;
  LineTracker_Struct.pSignal3 = (SignalDef_u*)pSignal3;
  LineTracker_Struct.pSignal4 = (SignalDef_u*)pSignal4;
  LineTracker_Struct.run_mode = Mode_Stop;

  Speed_Tracker_Init( &LineTracker_Struct.Speed_TrackerStruct,
                      CtrlFreqHZ,
                      LINETRACKER_SPEED_UP_ACC,
                      LINETRACKER_SPEED_DOWN_ACC );

  PID_struct_init(&x_pid, POSITION_PID, LINETRACKER_SPEED_MAX, 200, X_FDBCK_P, X_FDBCK_I, X_FDBCK_D);
  PID_struct_init(&yaw_pid, POSITION_PID, LINETRACKER_SPEED_MAX, 200, YAW_FDBCK_P, YAW_FDBCK_I, YAW_FDBCK_D);
}
uint16_t ReadSignal1_TransverseLine_func(void)
{
  uint8_t num;

  num = 0;

  if(LineTracker_Struct.pSignal1->bit.ls1 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal1->bit.ls2 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal1->bit.ls3 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal1->bit.ls4 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal1->bit.ls5 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal1->bit.ls6 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal1->bit.ls7 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal1->bit.ls8 == LineTracker_Struct.active_level) num++;

  return (num >= LINETRACKER_LINE_REFERENCE);
}
uint16_t ReadSignal2_TransverseLine_func(void)
{
  uint8_t num;

  num = 0;

  if(LineTracker_Struct.pSignal2->bit.ls1 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal2->bit.ls2 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal2->bit.ls3 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal2->bit.ls4 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal2->bit.ls5 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal2->bit.ls6 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal2->bit.ls7 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal2->bit.ls8 == LineTracker_Struct.active_level) num++;

  return (num >= LINETRACKER_LINE_REFERENCE);
}
uint16_t ReadSignal3_TransverseLine_func(void)
{
  uint8_t num;

  num = 0;

  if(LineTracker_Struct.pSignal3->bit.ls1 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal3->bit.ls2 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal3->bit.ls3 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal3->bit.ls4 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal3->bit.ls5 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal3->bit.ls6 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal3->bit.ls7 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal3->bit.ls8 == LineTracker_Struct.active_level) num++;

  return (num >= LINETRACKER_LINE_REFERENCE);
}
uint16_t ReadSignal4_TransverseLine_func(void)
{
  uint8_t num;

  num = 0;

  if(LineTracker_Struct.pSignal4->bit.ls1 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal4->bit.ls2 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal4->bit.ls3 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal4->bit.ls4 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal4->bit.ls5 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal4->bit.ls6 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal4->bit.ls7 == LineTracker_Struct.active_level) num++;
  if(LineTracker_Struct.pSignal4->bit.ls8 == LineTracker_Struct.active_level) num++;

  return (num >= LINETRACKER_LINE_REFERENCE);
}
void LineTracker_WaitCarToStop(void)
{
  while( ( LineTracker_Struct.pLineTracker_ParaStruct != 0 ) || ( LineTracker_Struct.run_mode != Mode_Stop ) )
    LINE_PATROL_DELAY(2);
}

void LineTracker_Execute_OpenLoopWalk_Delay(DirectionDef_e Car_Direction, int16_t y_val, int16_t x_val, int16_t yaw_val, uint16_t time)
{
  LineTracker_WaitCarToStop();
  LineTracker_Struct.pLineTracker_ParaStruct = malloc(sizeof(LineTracker_ParaTypeDef));
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    memset(LineTracker_Struct.pLineTracker_ParaStruct, 0, sizeof(LineTracker_ParaTypeDef));

    if(Car_Direction == CarDirection_Head)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal1;
    else if(Car_Direction == CarDirection_Tail)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal3;
    else if(Car_Direction == CarDirection_Left)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal2;
    else if(Car_Direction == CarDirection_Right)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal4;
    Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LINETRACKER_SPEED_START);
    LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction = Car_Direction;
    LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = y_val;
    LineTracker_Struct.pLineTracker_ParaStruct->x_axis_set = x_val;
    LineTracker_Struct.pLineTracker_ParaStruct->yaw_set    = yaw_val;
    LineTracker_Struct.pLineTracker_ParaStruct->SetVal = time;   //运行时间
    LineTracker_Struct.run_mode = Mode_OpenLoopWalk_Delay;
  }
}
void LineTracker_Execute_LineNum(DirectionDef_e Car_Direction, uint16_t spd, uint8_t line_num, uint8_t mode, uint8_t CorrectiveCtrl_Flag)
{
  LineTracker_WaitCarToStop();
  LineTracker_Struct.pLineTracker_ParaStruct = malloc(sizeof(LineTracker_ParaTypeDef));
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    memset(LineTracker_Struct.pLineTracker_ParaStruct, 0, sizeof(LineTracker_ParaTypeDef));

    if(Car_Direction == CarDirection_Head)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal1;
    else if(Car_Direction == CarDirection_Tail)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal3;
    else if(Car_Direction == CarDirection_Left)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal2;
    else if(Car_Direction == CarDirection_Right)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal4;
    if(spd < LINETRACKER_SPEED_START)
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, spd);
    else
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LINETRACKER_SPEED_START);
    LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction = Car_Direction;
    LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = spd;
    LineTracker_Struct.pLineTracker_ParaStruct->line_num = line_num;
    LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag = CorrectiveCtrl_Flag; //是否循迹纠正
    LineTracker_Struct.pLineTracker_ParaStruct->SetVal = mode;   //停止的状态
    LineTracker_Struct.run_mode = Mode_LineTracker_LineNum;
    //	LineTracker_Execute_LineNum(CarDirection_Head, 250, 2, 1, 0);
  }
}
void LineTracker_Execute_Delay(DirectionDef_e Car_Direction, uint16_t spd, uint16_t time, uint8_t CorrectiveCtrl_Flag)
{
  LineTracker_WaitCarToStop();
  LineTracker_Struct.pLineTracker_ParaStruct = malloc(sizeof(LineTracker_ParaTypeDef));
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    memset(LineTracker_Struct.pLineTracker_ParaStruct, 0, sizeof(LineTracker_ParaTypeDef));

    if(Car_Direction == CarDirection_Head)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal1;
    else if(Car_Direction == CarDirection_Tail)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal3;
    else if(Car_Direction == CarDirection_Left)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal2;
    else if(Car_Direction == CarDirection_Right)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal4;
    if(spd < LINETRACKER_SPEED_START)
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, spd);
    else
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LINETRACKER_SPEED_START);
    LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction = Car_Direction;
    LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = spd;
    LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag = CorrectiveCtrl_Flag; //是否循迹纠正
    LineTracker_Struct.pLineTracker_ParaStruct->SetVal = time;   //运行时间
    LineTracker_Struct.run_mode = Mode_LineTracker_Delay;
  }
}
void LineTracker_Execute_Condition(DirectionDef_e Car_Direction, uint16_t spd, uint16_t (*ReadData_func)(void), uint16_t TargetState, uint8_t CorrectiveCtrl_Flag)
{
  LineTracker_WaitCarToStop();
  LineTracker_Struct.pLineTracker_ParaStruct = malloc(sizeof(LineTracker_ParaTypeDef));
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    memset(LineTracker_Struct.pLineTracker_ParaStruct, 0, sizeof(LineTracker_ParaTypeDef));

    if(Car_Direction == CarDirection_Head)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal1;
    else if(Car_Direction == CarDirection_Tail)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal3;
    else if(Car_Direction == CarDirection_Left)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal2;
    else if(Car_Direction == CarDirection_Right)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal4;
    if(spd < LINETRACKER_SPEED_START)
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, spd);
    else
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LINETRACKER_SPEED_START);
    LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction = Car_Direction;
    LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = spd;
    LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag = CorrectiveCtrl_Flag; //是否循迹纠正
    LineTracker_Struct.pLineTracker_ParaStruct->ReadData_func = ReadData_func;
    LineTracker_Struct.pLineTracker_ParaStruct->SetVal = TargetState;
    LineTracker_Struct.run_mode = Mode_LineTracker_Condition;
  }
}
void LineTracker_Execute_Condition1(DirectionDef_e Car_Direction, uint16_t spd, uint16_t (*ReadData_func)(void), uint16_t TargetState, uint8_t CorrectiveCtrl_Flag)
{
  LineTracker_WaitCarToStop();
  LineTracker_Struct.pLineTracker_ParaStruct = malloc(sizeof(LineTracker_ParaTypeDef));
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    memset(LineTracker_Struct.pLineTracker_ParaStruct, 0, sizeof(LineTracker_ParaTypeDef));

    if(Car_Direction == CarDirection_Head)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal1;
    else if(Car_Direction == CarDirection_Tail)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal3;
    else if(Car_Direction == CarDirection_Left)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal2;
    else if(Car_Direction == CarDirection_Right)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal4;
    if(spd < LINETRACKER_SPEED_START)
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, spd);
    else
      Speed_Tracker_NewTask(&LineTracker_Struct.Speed_TrackerStruct, LINETRACKER_SPEED_START);
    LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction = Car_Direction;
    LineTracker_Struct.pLineTracker_ParaStruct->x_axis_set = spd;
    LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag = CorrectiveCtrl_Flag; //是否循迹纠正
    LineTracker_Struct.pLineTracker_ParaStruct->ReadData_func = ReadData_func;
    LineTracker_Struct.pLineTracker_ParaStruct->SetVal = TargetState;
    LineTracker_Struct.run_mode = Mode_LineTracker_Condition;
  }
}
void LineTracker_Execute_Wheel_90(DirectionDef_e Car_Direction, int16_t spd, uint16_t distance, uint8_t CorrectiveCtrl_Flag)
{
  LineTracker_WaitCarToStop();
  LineTracker_Struct.pLineTracker_ParaStruct = malloc(sizeof(LineTracker_ParaTypeDef));
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    memset(LineTracker_Struct.pLineTracker_ParaStruct, 0, sizeof(LineTracker_ParaTypeDef));

    if(Car_Direction == CarDirection_Head)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal1;
    else if(Car_Direction == CarDirection_Tail)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal3;
    else if(Car_Direction == CarDirection_Left)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal2;
    else if(Car_Direction == CarDirection_Right)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal4;

    LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction = Car_Direction;
    LineTracker_Struct.pLineTracker_ParaStruct->yaw_set = spd;
    LineTracker_Struct.pLineTracker_ParaStruct->SetVal = distance;
    LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag = CorrectiveCtrl_Flag; //是否循迹纠正
    LineTracker_Struct.run_mode = Mode_LineTracker_Wheel_90;
  }
}
void LineTracker_Execute_RotateAngle(DirectionDef_e Car_Direction, int16_t spd, int16_t angle)
{
  LineTracker_WaitCarToStop();
  LineTracker_Struct.pLineTracker_ParaStruct = malloc(sizeof(LineTracker_ParaTypeDef));
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    memset(LineTracker_Struct.pLineTracker_ParaStruct, 0, sizeof(LineTracker_ParaTypeDef));
    if(Car_Direction == CarDirection_Head)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal1;
    else if(Car_Direction == CarDirection_Tail)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal3;
    else if(Car_Direction == CarDirection_Left)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal2;
    else if(Car_Direction == CarDirection_Right)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal4;
    LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction = Car_Direction;
    if(angle < 0)
      LineTracker_Struct.pLineTracker_ParaStruct->yaw_set = -spd;
    else
      LineTracker_Struct.pLineTracker_ParaStruct->yaw_set = spd;
    //		LineTracker_Struct.pLineTracker_ParaStruct->SetVal = abs(angle*345/360);//360°要加补偿,这个补偿什么引起的还不知道
    LineTracker_Struct.pLineTracker_ParaStruct->SetVal = abs(angle) % 360;
    if(LineTracker_Struct.pLineTracker_ParaStruct->SetVal > 180)
    {
      LineTracker_Struct.pLineTracker_ParaStruct->SetVal = 360 - LineTracker_Struct.pLineTracker_ParaStruct->SetVal;
      LineTracker_Struct.pLineTracker_ParaStruct->yaw_set = -LineTracker_Struct.pLineTracker_ParaStruct->yaw_set;
    }
    LineTracker_Struct.run_mode = Mode_Rotate_Angle;
  }
}
void LineTracker_Execute_Encoder(DirectionDef_e Car_Direction, uint16_t spd, int32_t *pTotal_ecd, uint16_t distance, uint8_t CorrectiveCtrl_Flag)
{
  LineTracker_WaitCarToStop();
  LineTracker_Struct.pLineTracker_ParaStruct = malloc(sizeof(LineTracker_ParaTypeDef));
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    memset(LineTracker_Struct.pLineTracker_ParaStruct, 0, sizeof(LineTracker_ParaTypeDef));
    if(Car_Direction == CarDirection_Head)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal1;
    else if(Car_Direction == CarDirection_Tail)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal3;
    else if(Car_Direction == CarDirection_Left)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal2;
    else if(Car_Direction == CarDirection_Right)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal4;
    LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction = Car_Direction;
    LineTracker_Struct.pLineTracker_ParaStruct->y_axis_set = spd;
    LineTracker_Struct.pLineTracker_ParaStruct->pTotal_ecd = pTotal_ecd;
    LineTracker_Struct.pLineTracker_ParaStruct->SetVal = distance;
    LineTracker_Struct.pLineTracker_ParaStruct->CorrectiveCtrl_Flag = CorrectiveCtrl_Flag;
    LineTracker_Struct.run_mode = Mode_LineTracker_Encoder;
  }
}
void LineTracker_Execute_SituAdjust(DirectionDef_e Car_Direction, uint8_t mode, uint16_t OutTime)
{
  LineTracker_WaitCarToStop();
  LineTracker_Struct.pLineTracker_ParaStruct = malloc(sizeof(LineTracker_ParaTypeDef));
  if(LineTracker_Struct.pLineTracker_ParaStruct != 0)
  {
    memset(LineTracker_Struct.pLineTracker_ParaStruct, 0, sizeof(LineTracker_ParaTypeDef));
    if(Car_Direction == CarDirection_Head)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal1;
    else if(Car_Direction == CarDirection_Tail)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal3;
    else if(Car_Direction == CarDirection_Left)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal2;
    else if(Car_Direction == CarDirection_Right)
      LineTracker_Struct.pHeadSignal = LineTracker_Struct.pSignal4;
    if(mode == 0)
    {
      LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction = CarDirection_Head;
    }
    else
    {
      LineTracker_Struct.pLineTracker_ParaStruct->Car_Direction = Car_Direction;
    }
    LineTracker_Struct.pLineTracker_ParaStruct->TimeMem = OutTime;
    LineTracker_Struct.pLineTracker_ParaStruct->run_state = mode;
    LineTracker_Struct.run_mode = Mode_LineTracker_SituAdjust;
  }
}
