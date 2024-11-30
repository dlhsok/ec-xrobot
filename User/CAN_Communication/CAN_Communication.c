/* 包含头文件 ----------------------------------------------------------------*/
#include "CAN_Communication.h"
/* 私有类型定义 --------------------------------------------------------------*/
typedef union
{
  uint32_t WORD;
  struct
  {
    uint32_t SourceList     : 4;
    uint32_t SourceAddr     : 8;
    uint32_t ComanndNumber  : 4;
    uint32_t TargetList     : 4;
    uint32_t TargetAddr     : 8;
    uint32_t Page           : 1;
    uint32_t retain         : 3;
  } bits;
} CanIDType;
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
static u16 Tim_GetCurrentTime = 0;
/* 扩展变量 ------------------------------------------------------------------*/
uint8_t canrxtxerr = 0;
/* 私有函数原形 --------------------------------------------------------------*/
/*****************************************************************************************
*  <summary>
*  get current time;
*  <returns> current time tick
******************************************************************************************/
static u8 Tim_CheckTimeOut(u16 TimeStart, u16 TimeNow, u16 TimeOut)
{
#define ActivateCounter ((u16)(TimeOut+TimeStart))
  if (ActivateCounter >= TimeStart)
  {
    if ((TimeNow >= ActivateCounter) ||
        (TimeNow < TimeStart))
    {
      return TRUE;
    }
  }
  else if ((TimeNow >= ActivateCounter) &&
           (TimeNow < TimeStart))
  {
    return TRUE;
  }
  return FALSE;
}
/** @brief  发送一帧数据
  * @param
  * @retval
  */
static void SendUsedPG(PgnCanTxDataType *pQ)
{
  u8 err;
  uint32_t RTR;
  uint32_t IDE;

  if(pQ->Datum == 0)
  {
    RTR = CAN_RTR_REMOTE;
  }
  else
  {
    RTR = CAN_RTR_DATA;
  }
  if(pQ->IDE == 1)
  {
    IDE = CAN_ID_STD;
  }
  else
  {
    IDE = CAN_ID_EXT;
  }
  err = CANSend(pQ->Source, IDE, RTR, (u8*)pQ->Datum, pQ->TxDataLength, pQ->ch);
  if(0 == err)  //发送成功
  {
    if(MODE_REQ == pQ->mode)
    {
      pQ->PGState = PG_TX_TxING;
    }
    else if((MODE_BRO == pQ->mode) || (MODE_ACK == pQ->mode))
    {
      pQ->PGState = PG_TX_SUC;
    }
  }
}
static void CANCommunication_InformationParsing(CanRxMemberType *pQ)
{
  u8 counts;
  u8 i;
  u8 dlcbuff;

  for (counts = TX_PGN_USER; counts < NR_OF_TX_PG; counts++)
  {
    if (TxPGArray[counts].Target == pQ->Id)
    {
      dlcbuff = 0;
      if(TxPGArray[counts].RxDatum != 0)
      {
        dlcbuff = (TxPGArray[counts].RxDataLength == pQ->DLC);
      }
      else if(TxPGArray[counts].RxDataLength != 0)
      {
        if(CAN_RTR_REMOTE == pQ->RTR)//为遥控帧
        {
          dlcbuff = 1;
        }
      }
      if(1 == dlcbuff)
      {
        if(TxPGArray[counts].RxDatum != 0)
        {
          for(i = 0; i < pQ->DLC; i++)
          {
            ((u8*)TxPGArray[counts].RxDatum)[i] = pQ->Data[i];
          }
        }
        GetReferenceToDevice(pQ->Id);
        if((TxPGArray[counts].Datum != 0) || (TxPGArray[counts].TxDataLength != 0))
        {
          switch(TxPGArray[counts].mode)
          {
          case MODE_BRO: //广播模式
            TxPGArray[counts].Timer = Tim_GetCurrentTime - TxPGArray[counts].TransRate;
            break;
          case MODE_REQ: //请求模式
            if(PG_TX_TxING == TxPGArray[counts].PGState)
            {
              TxPGArray[counts].PGState = PG_TX_SUC;
            }
            break;
          case MODE_ACK: //答应模式
            TxPGArray[counts].Enable = TRUE;
            TxPGArray[counts].PGState = PG_TX_REQ;
            TxPGArray[counts].Timer = Tim_GetCurrentTime - TxPGArray[counts].TransRate;
            break;
          }
        }
      }
    }
  }
}
/* 函数体 --------------------------------------------------------------------*/
void CAN_TT_ASYNC_Enable(u8 list) //异步模式使能
{
  if(TxPGArray[list].TransType == TT_ASYNC)
  {
    TxPGArray[list].PGState = PG_TX_REQ;
  }
}
void CAN_TT_EnableSet(u8 list, u8 enable) //使能失能
{
  if(enable == FALSE)
  {
    TxPGArray[list].Enable = FALSE;
  }
  else
  {
    TxPGArray[list].Enable = TRUE;
  }
}
uint8_t CAN_TT_PGStateRead(u8 list) //读取CAN总线中列表序号对应帧的通讯状态
{
  return TxPGArray[list].PGState;
}
uint8_t CAN_TT_ErrRead(u8 list) //读取CAN总线中列表序号对应帧的错误信息
{
  if(TxPGArray[list].LogBootFail_cnt >= 10)
  {
    return 0x81;
  }
  else if(TxPGArray[list].can_err >= 5)
  {
    return 0x82;
  }
  if(TxPGArray[list].LogBootFail_cnt > 0)
  {
    return 0x01;
  }
  else if(TxPGArray[list].can_err > 0)
  {
    return 0x02;
  }
  else
  {
    return 0;
  }
}

void CANCommunication_Scan(void)
{
  u8 counts;
  CanRxMemberType val_buf;
  /**************************************************************************************
  ***************************************************************************************
  **************************************接收部分*****************************************
  ***************************************************************************************
  **************************************************************************************/
  if(CAN_OutQueue(&val_buf))
  {
    CANCommunication_InformationParsing(&val_buf);
  }
  /**************************************************************************************
  ***************************************************************************************
  **************************************发送部分*****************************************
  ***************************************************************************************
  **************************************************************************************/
  GetReferenceFromDevice();
  for (counts = TX_PGN_USER; counts < NR_OF_TX_PG; counts++)
  {
    if (TxPGArray[counts].Enable)
    {
      if (TxPGArray[counts].TransType  == TT_ASYNC) //异步模式
      {
        if (TxPGArray[counts].PGState == PG_TX_REQ)
        {
          (void)SendUsedPG(&TxPGArray[counts]);
          if((TxPGArray[counts].can_err < 5) && (TxPGArray[counts].PGState == PG_TX_REQ))
          {
            TxPGArray[counts].can_err++;
          }
          TxPGArray[counts].Timer = Tim_GetCurrentTime;
          break;
        }
        else if(TxPGArray[counts].PGState == PG_TX_TxING)
        {
          TxPGArray[counts].can_err = 0;
          if(Tim_CheckTimeOut(TxPGArray[counts].Timer, Tim_GetCurrentTime, TxPGArray[counts].TransRate))
          {
            TxPGArray[counts].PGState = PG_TX_REQ;
            if(TxPGArray[counts].LogBootFail_cnt < 10)
            {
              TxPGArray[counts].LogBootFail_cnt++;
            }
          }
        }
        else if(TxPGArray[counts].PGState == PG_TX_SUC)
        {
          TxPGArray[counts].can_err = 0;
          TxPGArray[counts].LogBootFail_cnt = 0;
          TxPGArray[counts].PGState = PG_TX_FREE;
        }
      }
      else if (TxPGArray[counts].TransType == TT_CYCLIC) //循环模式
      {
        if ((Tim_CheckTimeOut(TxPGArray[counts].Timer, Tim_GetCurrentTime, TxPGArray[counts].TransRate))
            && (TxPGArray[counts].PGState == PG_TX_REQ))
        {
          (void)SendUsedPG(&TxPGArray[counts]);
          if((TxPGArray[counts].can_err < 5) && (TxPGArray[counts].PGState == PG_TX_REQ))
          {
            TxPGArray[counts].can_err++;
          }
          TxPGArray[counts].Timer = Tim_GetCurrentTime;
          break;
        }
        else if(TxPGArray[counts].PGState == PG_TX_TxING)
        {
          TxPGArray[counts].can_err = 0;
          if(Tim_CheckTimeOut(TxPGArray[counts].Timer, Tim_GetCurrentTime, TxPGArray[counts].TransRate))
          {
            TxPGArray[counts].PGState = PG_TX_REQ;
            if(TxPGArray[counts].LogBootFail_cnt < 10)
            {
              TxPGArray[counts].LogBootFail_cnt++;
            }
          }
        }
        else if (TxPGArray[counts].PGState == PG_TX_SUC)
        {
          TxPGArray[counts].can_err = 0;
          TxPGArray[counts].LogBootFail_cnt = 0;
          TxPGArray[counts].Timer = Tim_GetCurrentTime;
          TxPGArray[counts].PGState = PG_TX_REQ;
        }
      }
    }
  }
  /**************************************************************************************
  ***************************************************************************************
  **************************************检错部分*****************************************
  ***************************************************************************************
  **************************************************************************************/
  //...
  u8 err = 0;
  for (counts = TX_PGN_USER; counts < NR_OF_TX_PG; counts++)
  {
    if (TxPGArray[counts].can_err >= 5)
    {
      err |= 0x02;
      TxPGArray[counts].can_err = 0;
    }
    if(Bsp_Cancom_ReadERR() != 0)
    {
      err |= 0x04;
    }
  }
  if(err)
  {
    Bsp_Cancom_ConfigurationInit();
    Bsp_Cancom_FilterInit();
  }
  canrxtxerr |= err;
}
/**
  * 函 数 名: xCreate
  * 函数功能: CAN通讯数据结构创建原函数.
  * 输入参数: SourceAddr    数据来源地址
  *           SourceID      数据来源ID号
  *           TargetAddr    数据目标地址
  *           TargetID      数据目标ID号
  *           Enable        发送使能(TRUE/FALSE)
  *           PGState       发送使能(PG_TX_FREE/被动使能发送
  *                                  PG_TX_REQ/开机使能发送
  *                                  PG_TX_TxING
  *                                  PG_TX_SUC)
  *           mode          协议模式(MODE_BRO/广播模式
  *                                  MODE_REQ/请求模式
  *                                  MODE_ACK/答应模式)
  *           TransType     发送模式(TT_ASYNC/异步模式
  *                                  TT_CYCLIC/循环模式)
  *           TransRate     发送速率
  *           TxDatum       发送数据
  *           RxDatum       接收数据
  */
PgnCanTxDataType xCreate(   uint8_t SourceAddr,
                            uint8_t SourceID,
                            uint8_t TargetAddr,
                            uint8_t TargetID,
                            uint8_t TXComanndNumber,
                            uint8_t RXComanndNumber,
                            uint8_t Enable,
                            uint8_t PGState,
                            uint8_t mode,
                            uint8_t TransType,
                            uint16_t TransRate,
                            uint16_t Timer,
                            void *TxDatum,
                            uint8_t TxDataLength,
                            void *RxDatum,
                            uint8_t RxDataLength
                        )
{
  PgnCanTxDataType CanDataType;
  CanIDType PG_TX_ID;
  CanIDType PG_RX_ID;

  PG_TX_ID.WORD = 0;
  PG_RX_ID.WORD = 0;

  if((TxDatum != 0) || (TxDataLength != 0))
  {
    PG_TX_ID.bits.TargetList = TargetID;
    PG_TX_ID.bits.TargetAddr = TargetAddr;
    PG_TX_ID.bits.ComanndNumber = TXComanndNumber;
    PG_TX_ID.bits.SourceList = SourceID;
    PG_TX_ID.bits.SourceAddr = SourceAddr;
    PG_TX_ID.bits.Page = 0;
  }
  if((RxDatum != 0) || (RxDataLength != 0))
  {
    PG_RX_ID.bits.TargetList = SourceID;
    PG_RX_ID.bits.TargetAddr = SourceAddr;
    PG_RX_ID.bits.ComanndNumber = RXComanndNumber;
    PG_RX_ID.bits.SourceList = TargetID;
    PG_RX_ID.bits.SourceAddr = TargetAddr;
    PG_RX_ID.bits.Page = 0;
  }
  if(TxDataLength > 8)
  {
    TxDataLength = 8;
  }
  if(RxDataLength > 8)
  {
    RxDataLength = 8;
  }
  CanDataType.Enable = Enable;
  CanDataType.PGState = PGState;
  CanDataType.mode = mode;
  CanDataType.Timer = Timer;
  CanDataType.Source = PG_TX_ID.WORD;
  CanDataType.Target = PG_RX_ID.WORD;
  CanDataType.TransType = TransType;
  CanDataType.TransRate = TransRate;
  CanDataType.Datum = (u8*)TxDatum;
  CanDataType.TxDataLength = TxDataLength;
  CanDataType.RxDatum = (u8*)RxDatum;
  CanDataType.RxDataLength = RxDataLength;
  CanDataType.BagState = BAG_NOT;
  CanDataType.LogBootFail_cnt = 0;
  CanDataType.can_err = 0;

  return CanDataType;
}
void Tim_GetCurrentTimeAdd_Scan1MS(void)
{
  Tim_GetCurrentTime++;
}
void CANCommunication_Init(void)
{
  Tim_GetCurrentTime = 2000;
  InitialUsedPGs();
  Bsp_Cancom_FilterInit();
}

void QueueOverflow_Dispose(CanRxMemberType *pQ)
{
  CANCommunication_InformationParsing(pQ);
}


