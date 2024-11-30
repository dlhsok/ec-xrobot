/* 包含头文件 ----------------------------------------------------------------*/
#include "SLAVE_DCMotorMiniwatt/SLAVE_DCMotorMiniwatt.h"
#include "string.h"
#include "CAN_Communication.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static DCMotorMiniwattDef_t* head_handle = NULL;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/

static uint16_t SLAVE_DCMotorMiniwatt_ProtocolDefinitions(DCMotorMiniwattDef_t* handle,
    uint16_t cannum,
    PgnCanTxDataType *head)
{
  uint16_t cannum_buff;
  cannum_buff = cannum;

  handle->CAN_Number = cannum_buff; //第一个帧的CAN编号，后面以次累加计算
#ifndef DCMOTOR_NEW
  head[cannum_buff++] = xCreate(  DEVICE_ID, DEVICE_LIST,
                                  MotorPlate_Addr,
                                  handle->Motor_SerialNumber.DATE.Serial_Number,
                                  2, 2,
                                  FALSE, PG_TX_FREE, MODE_REQ, TT_ASYNC, 50, 0,
                                  handle->Motor_SerialNumber.Datum, PG_RXMOTOR_SERIAL_NUMBER_LEN,
                                  handle->Motor_SerialNumber.Datum, PG_RXMOTOR_SERIAL_NUMBER_LEN); //序号修改
  head[cannum_buff - 1].ch = 1;
#else
  head[cannum_buff++] = xCreate(  DEVICE_ID, DEVICE_LIST,
                                  MotorPlate_Addr,
                                  handle->Motor_ControlData1.Data_id.id,
                                  1, 1,
                                  TRUE, PG_TX_FREE, MODE_BRO, TT_ASYNC, 10, 0,
                                  handle->Motor_ControlData1.Datum, PG_RXMOTORDATA1_LEN,
                                  handle->Motor_ControlData1_Ack.Datum, PG_DCMOTORCTRL_DATA1_STE_ACK_LEN); //控制数据1帧初始化
  head[cannum_buff - 1].ch = 1;
#endif
  head[cannum_buff++] = xCreate(  ALL_Addr, 0,
                                  MotorPlate_Addr,
#ifndef DCMOTOR_NEW
                                  handle->Motor_SerialNumber.DATE.Serial_Number,
#else
                                  handle->Motor_ControlData1.Data_id.id,
#endif
                                  0, 0,
                                  FALSE, PG_TX_FREE, MODE_BRO, TT_CYCLIC, 10, 0,
                                  0, 0,
                                  handle->Motor_UploadData.Datum, PG_TXMOTORDATA_LEN); //反馈数据帧初始化
  head[cannum_buff - 1].ch = 1;

  head[cannum_buff++] = xCreate(  DEVICE_ID, DEVICE_LIST,
                                  MotorPlate_Addr,
#ifndef DCMOTOR_NEW
                                  handle->Motor_SerialNumber.DATE.Serial_Number,
#else
                                  handle->Motor_ControlData1.Data_id.id,
#endif
                                  0, 0,
                                  TRUE, PG_TX_REQ, MODE_BRO, TT_CYCLIC, 10, 0,
                                  handle->Motor_ControlData.Datum, PG_RXMOTORDATA_LEN,
                                  0, 0); //控制数据帧初始化
  head[cannum_buff - 1].ch = 1;

  return cannum_buff;
}

uint16_t SLAVE_DCMotorMiniwatt_CANinit(uint16_t cannum, void *head)
{
  DCMotorMiniwattDef_t* target;
  uint16_t cannum_buff;
  cannum_buff = cannum;

  for(target = head_handle; target; target = target->next)
  {
    cannum_buff = SLAVE_DCMotorMiniwatt_ProtocolDefinitions(target, cannum_buff, ((PgnCanTxDataType*)head));
  }

  return cannum_buff;
}

uint8_t SLAVE_DCMotorMiniwatt_Init(DCMotorMiniwattDef_t* handle, uint8_t list)
{
  DCMotorMiniwattDef_t* target = head_handle;

  memset(handle, 0, sizeof(DCMotorMiniwattDef_t));

#ifndef DCMOTOR_NEW
  handle->Motor_SerialNumber.DATE.Serial_Number = list;
#else
  handle->Motor_ControlData1.Data_id.id = list;
#endif
  //电机速度： 0.5 rpm/位递增，从-16000 rpm开始计算，范围-16000 rpm到16000 rpm),0rpm计算得出32000;
  handle->Motor_ControlData.DATE.SetSpeed = 32000;
  handle->Motor_ControlData.DATE.SetPwm = 0;
  handle->Motor_UploadData.DATE.Speed = 32000;

  while(target)
  {
    if(target == handle)
      return 0;	//already exist.
    target = target->next;
  }
  handle->next = head_handle;
  head_handle = handle;
  return 1;
}

void SLAVE_DCMotorMiniwatt_NumberSet(DCMotorMiniwattDef_t* handle, uint8_t number)
{
  if(number > 15)
  {
    number = 15;
  }

#ifndef DCMOTOR_NEW
  handle->Motor_SerialNumber.DATE.Serial_Number = number;
#else
  handle->Motor_ControlData1.Data_id.id = number;
  handle->Motor_ControlData1.Data_id.order = 0x01;
#endif

  CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
#ifndef DCMOTOR_NEW
uint16_t SLAVE_DCMotorMiniwatt_PlaceRead(DCMotorMiniwattDef_t* handle)
{
  return handle->Motor_UploadData.DATE.Coded_Value;
}
#else
int32_t SLAVE_DCMotorMiniwatt_PlaceRead(DCMotorMiniwattDef_t* handle)
{
  return ((handle->Motor_UploadData.DATE.Coded_Value_H << 16) | handle->Motor_UploadData.DATE.Coded_Value_L);
}
#endif


int16_t SLAVE_DCMotorMiniwatt_SpeedRead(DCMotorMiniwattDef_t* handle)
{
  int16_t val;
  val = (handle->Motor_UploadData.DATE.Speed / 2) - 16000;

  return val;
}
#include "data.h"
void SLAVE_DCMotorMiniwatt_SpeedSet(DCMotorMiniwattDef_t* handle, int16_t val)
{

  handle->Motor_ControlData.DATE.SetPwm = 0;
  handle->Motor_ControlData.DATE.SetSpeed = (val + 16000) * 2;
#ifdef DCMOTOR_NEW
  handle->Motor_ControlData.DATE.SetCurr = 0;
  if(val != 0)
    handle->Motor_ControlData.DATE.SetLocation = 0;
#endif
}
void SLAVE_DCMotorMiniwatt_PWMSet(DCMotorMiniwattDef_t* handle, int16_t val)
{
  handle->Motor_ControlData.DATE.SetPwm = val;
  handle->Motor_ControlData.DATE.SetSpeed = 32000;
#ifdef DCMOTOR_NEW
  handle->Motor_ControlData.DATE.SetCurr = 0;
  handle->Motor_ControlData.DATE.SetLocation = 0;
#endif
}
#ifdef DCMOTOR_NEW
void SLAVE_DCMotorMiniwatt_CurrSet(DCMotorMiniwattDef_t* handle, int16_t val)
{
  handle->Motor_ControlData.DATE.SetCurr = val;
  handle->Motor_ControlData.DATE.SetSpeed = 32000;
  handle->Motor_ControlData.DATE.SetPwm = 0;
  handle->Motor_ControlData.DATE.SetLocation = 0;
}
void SLAVE_DCMotorMiniwatt_LocationSet(DCMotorMiniwattDef_t* handle, int16_t val)
{
  handle->Motor_ControlData.DATE.SetCurr = 0;
  handle->Motor_ControlData.DATE.SetSpeed = 32000;
  handle->Motor_ControlData.DATE.SetPwm = 0;
  handle->Motor_ControlData.DATE.SetLocation = val;
}
void SLAVE_DCMotorMiniwatt_Digital_PWMSet(DCMotorMiniwattDef_t* handle, int16_t val)
{
  handle->Motor_ControlData1.Data_pwm.pwm = val;
  handle->Motor_ControlData1.Data_pwm.order = 0x02;
  CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_DCMotorMiniwatt_Digital_SpeedSet(DCMotorMiniwattDef_t* handle, int16_t val)
{
  handle->Motor_ControlData1.Data_speed.speed = val;
  handle->Motor_ControlData1.Data_speed.order = 0x03;
  CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_DCMotorMiniwatt_Digital_LocationSet(DCMotorMiniwattDef_t* handle, int32_t val)
{
  handle->Motor_ControlData1.Data_location.location = val;
  handle->Motor_ControlData1.Data_location.order = 0x04;
  CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_DCMotorMiniwatt_Digital_OFF(DCMotorMiniwattDef_t* handle)
{
  handle->Motor_ControlData1.Data_id.order = 0x00;
  CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_DCMotorMiniwatt_Digital_Disable(DCMotorMiniwattDef_t* handle)
{
  handle->Motor_ControlData1.Data_Flag.order = 0x05;
  handle->Motor_ControlData1.Data_Flag.flag.bit.disable = 1;
  CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_DCMotorMiniwatt_Digital_Brake(DCMotorMiniwattDef_t* handle)
{
  handle->Motor_ControlData1.Data_Flag.order = 0x05;
  handle->Motor_ControlData1.Data_Flag.flag.bit.brake = 1;
  CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_DCMotorMiniwatt_Digital_ClrStall(DCMotorMiniwattDef_t* handle)
{
  handle->Motor_ControlData1.Data_Flag.order = 0x05;
  handle->Motor_ControlData1.Data_Flag.flag.bit.clear_stall = 0;
  CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_DCMotorMiniwatt_Digital_Clr(DCMotorMiniwattDef_t* handle)
{
  handle->Motor_ControlData1.Data_Flag.order = 0x05;
  handle->Motor_ControlData1.Data_Flag.flag.bit.disable = 0;
  handle->Motor_ControlData1.Data_Flag.flag.bit.brake = 0;
  handle->Motor_ControlData1.Data_Flag.flag.bit.clear_stall = 0;
  CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_DCMotorMiniwatt_Digital_SpeedPWMSet(DCMotorMiniwattDef_t* handle, int16_t val)
{
  handle->Motor_ControlData1.Data_speed.speed = val;
  handle->Motor_ControlData1.Data_speed.order = 0x07;
  CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_DCMotorMiniwatt_CANRXDispose(void *head, uint32_t Id)
{
  DCMotorMiniwattDef_t* target;
  static int16_t speed_old = 0;
  for(target = head_handle; target; target = target->next)
  {
    if(Id == ((PgnCanTxDataType*)head)[target->CAN_Number + 1].Target)
    {
      target->Coded_ValueAccu += (ABS(SLAVE_DCMotorMiniwatt_PlaceRead(target) - target->Coded_ValueOld));
      target->Coded_ValueOld = SLAVE_DCMotorMiniwatt_PlaceRead(target);

      if( ABS(target->Motor_ControlData1.Data_speed.speed) - ABS(speed_old) > 50 )
        target->Motor_ControlData1.Data_speed.speed = speed_old;
      speed_old = target->Motor_ControlData1.Data_speed.speed;
    }
  }
}
#else
void SLAVE_DCMotorMiniwatt_CANRXDispose(void *head, uint32_t Id)
{
  DCMotorMiniwattDef_t* target;

  for(target = head_handle; target; target = target->next)
  {
    if(Id == ((PgnCanTxDataType*)head)[target->CAN_Number + 1].Target)
    {
      if(target->Motor_UploadData.DATE.Coded_Value_H >= target->Coded_ValueOld)
      {
        target->Coded_ValueAccu += (target->Motor_UploadData.DATE.Coded_Value_H - target->Coded_ValueOld);
      }
      else
      {
        target->Coded_ValueAccu += (65536 - target->Coded_ValueOld + target->Motor_UploadData.DATE.Coded_Value_H);
      }
      target->Coded_ValueOld = target->Motor_UploadData.DATE.Coded_Value_H;
    }
  }
}
#endif






