/* 包含头文件 ----------------------------------------------------------------*/
#include "MyModbus.h"

/* 私有宏定义 ----------------------------------------------------------------*/
#ifndef  FALSE
#define  FALSE    0
#endif
#ifndef  TRUE
#define  TRUE     1
#endif
/* 私有类型定义 --------------------------------------------------------------*/
#define DATABUF_READY   0x8000
/* 私有函数原形 --------------------------------------------------------------*/
static void modbus_init(void);
static void modbus_Timer(void);
static void modbus_scan(void);
static uint8_t modbus_receive(uint8_t *str,uint16_t len);
static uint8_t modbus_transfer(uint8_t *str,uint16_t *len);
#if (MODBUS_MODE == MODBUS_MODE_HOST)
static uint16_t modbus_CreateSendCommand(uint8_t address,//地址域
    uint8_t function,//功能码 范围1-255（128-255为异常响应保留）
    _e_PGState   PGState_E, //PDU运行状态
    _e_TransType TransType_E, //PDU传输类型
    uint16_t TransRate,//循环速率，单位ms
    uint16_t Timer, //计时器
    uint16_t Reg_StartAddrNumber,   //数据起始地址
    uint16_t Reg_EndAddrNumber,     //数据终点地址
    void *DataSourceAddr,     //来源数据地址
    void (*ComSucceedCallBack)(struct _s_ModbusPDU *pQ),  //发送成功回调函数
    void (*OvertimeCallBack)(struct _s_ModbusPDU *pQ)); //发送超时回调函数
#else
static uint16_t modbus_CreateCommand(uint8_t function,//功能码 范围1-255（128-255为异常响应保留）
                                     uint16_t Reg_StartAddrNumber,   //数据起始地址
                                     uint16_t Reg_EndAddrNumber,     //数据终点地址
                                     void *DataSourceAddr,     //来源数据地址
                                     uint8_t (*ComSucceedCallBack)(struct _s_ModbusPDU *pQ)); //发送成功回调函数
#endif
/* 私有变量 ------------------------------------------------------------------*/
static const uint8_t auchCRCHi[] =
{
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
static const uint8_t auchCRCLo[] =
{
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
  0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
  0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
  0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
  0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
  0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
  0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
  0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
  0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;
/* 扩展变量 ------------------------------------------------------------------*/
#if (MODBUS_MODE == MODBUS_MODE_HOST)
_s_ModbusPDU modbus_nametab_tx[MODBUS_NR_OF_HOST_NAMETABE];
#else
_s_ModbusPDU modbus_nametab_rx[MODBUS_NR_OF_HOST_NAMETABE];
#endif
_s_modbus_dev modbus_dev_S=
{
  MODBUS_HOST, //默认为主模式
  MODBUS_SEND_FREE,
  0,      //运行时间
#if (MODBUS_MODE == MODBUS_MODE_HOST)
  0,    //命令发送个数
  modbus_nametab_tx, //指向发送命令的指针
#else
  0,    //命令接收个数
  modbus_nametab_rx, //指向接收命令的指针
#endif
  /*************指向运行的函数****************/
  modbus_init,//数据初始化函数
  modbus_Timer,//计时器函数
  modbus_scan,//任务处理函数
  modbus_receive,//数据接收函数
  modbus_transfer,//数据发送函数
#if (MODBUS_MODE == MODBUS_MODE_HOST)
  modbus_CreateSendCommand,//声明一帧发送命令
#else
  modbus_CreateCommand,
#endif
};
uint8_t MODBUS_RX_BUF[MODBUS_RX_LEN];
uint16_t MODBUS_RX_STA = 0;
uint8_t  MODBUS_TX_BUF[MODBUS_TX_LEN];
uint16_t MODBUS_TX_STA = 0;
/* 函数体 --------------------------------------------------------------------*/
uint16_t CRC_Compute(uint8_t *puchMsg, uint16_t usDataLen)
{
  uint8_t uchCRCHi = 0xFF ;
  uint8_t uchCRCLo = 0xFF ;
  uint32_t uIndex ;
  while (usDataLen--)
    {
      uIndex = uchCRCHi ^ *puchMsg++ ;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
      uchCRCLo = auchCRCLo[uIndex] ;
    }
  return ((uchCRCHi<< 8)  | (uchCRCLo)) ;
}
void modbus_PGEnable( uint8_t  address,
                      uint8_t  function,
                      uint16_t AddrNumber)
{
  uint16_t i;

  for(i=0; i<modbus_dev_S.pnum_tx; i++)
    {
      if( (modbus_nametab_tx[i].address == address) && \
      (modbus_nametab_tx[i].function == function) && \
      (modbus_nametab_tx[i].TransType_E == MODBUS_ASYNC) )
        {
          if( ( AddrNumber >= modbus_nametab_tx[i].Reg_StartAddrNumber ) && \
              ( AddrNumber <= modbus_nametab_tx[i].Reg_EndAddrNumber ) )
            {
              modbus_nametab_tx[i].PGState_E = MODBUS_PG_TX_REQ;
            }
        }
    }
}
void modbus_PGDisable( uint8_t  address,
                       uint8_t  function,
                       uint16_t AddrNumber)
{
  uint16_t i;

  for(i=0; i<modbus_dev_S.pnum_tx; i++)
    {
      if( (modbus_nametab_tx[i].address == address) && \
      (modbus_nametab_tx[i].function == function) && \
      (modbus_nametab_tx[i].TransType_E == MODBUS_ASYNC) )
        {
          if( ( AddrNumber >= modbus_nametab_tx[i].Reg_StartAddrNumber ) && \
              ( AddrNumber <= modbus_nametab_tx[i].Reg_EndAddrNumber ) )
            {
              modbus_nametab_tx[i].PGState_E = MODBUS_PG_TX_FREE;
            }
        }
    }
}
#if (MODBUS_MODE == MODBUS_MODE_HOST)
static uint8_t Tim_CheckTimeOut(uint16_t TimeStart, uint16_t TimeNow, uint16_t TimeOut)
{
#define ActivateCounter ((uint16_t)(TimeOut+TimeStart))

  if (ActivateCounter >= TimeStart)
    {
      if ((TimeNow >= ActivateCounter) || (TimeNow < TimeStart))
        {
          return TRUE;
        }
    }
  else if ((TimeNow >= ActivateCounter) && (TimeNow < TimeStart))
    {
      return TRUE;
    }

  return FALSE;
}
void modbus_TT_ASYNC_Enable( uint8_t  address,
                             uint8_t  function,
                             uint16_t Send_StartAddrNumber,
                             uint16_t Send_EndAddrNumber )
{
  uint16_t i;
  uint16_t BigAddr;
  uint16_t SmallAddr;

  for(i=0; i<modbus_dev_S.pnum_tx; i++)
    {
      if( (modbus_nametab_tx[i].address == address) && \
      (modbus_nametab_tx[i].function == function) && \
      (modbus_nametab_tx[i].TransType_E == MODBUS_ASYNC) )
        {
          if(Send_StartAddrNumber <= Send_EndAddrNumber)
            {
              BigAddr = Send_EndAddrNumber;
              SmallAddr = Send_StartAddrNumber;
            }
          else
            {
              BigAddr = Send_StartAddrNumber;
              SmallAddr = Send_EndAddrNumber;
            }
          if( ( SmallAddr >= modbus_nametab_tx[i].Reg_StartAddrNumber ) && \
              ( BigAddr <= modbus_nametab_tx[i].Reg_EndAddrNumber ) )
            {
              modbus_nametab_tx[i].Send_StartAddrNumber = SmallAddr;   //数据起始地址
              modbus_nametab_tx[i].Send_DataLen = BigAddr-SmallAddr+1; //数据域数据长度
              modbus_nametab_tx[i].PGState_E = MODBUS_PG_TX_REQ;
            }
        }
    }
}
static uint16_t modbus_CreateSendCommand(uint8_t address,//地址域
    uint8_t function,//功能码 范围1-255（128-255为异常响应保留）
    _e_PGState   PGState_E, //PDU运行状态
    _e_TransType TransType_E, //PDU传输类型
    uint16_t TransRate,//循环速率，单位ms
    uint16_t Timer, //计时器
    uint16_t Reg_StartAddrNumber,   //数据起始地址
    uint16_t Reg_EndAddrNumber,     //数据终点地址
    void *DataSourceAddr,     //来源数据地址
    void (*ComSucceedCallBack)(struct _s_ModbusPDU *pQ),  //发送成功回调函数
    void (*OvertimeCallBack)(struct _s_ModbusPDU *pQ)) //发送超时回调函数
{
  uint16_t BigAddr;
  uint16_t SmallAddr;
  uint16_t FrameNumber;

  if(modbus_dev_S.pnum_tx < MODBUS_NR_OF_HOST_NAMETABE)
    {
      /*************Modbus 信息*******************/
      modbus_nametab_tx[modbus_dev_S.pnum_tx].address = address;//地址域
      modbus_nametab_tx[modbus_dev_S.pnum_tx].function = function;
      /*************帧 信息*******************/
      modbus_nametab_tx[modbus_dev_S.pnum_tx].PGState_E = PGState_E;           //PDU运行状态
      modbus_nametab_tx[modbus_dev_S.pnum_tx].TransType_E = TransType_E;         //PDU传输类型
      modbus_nametab_tx[modbus_dev_S.pnum_tx].txerrnum = 0;            //发送失败次数
      modbus_nametab_tx[modbus_dev_S.pnum_tx].TransRate = TransRate;	          //循环速率，单位ms
      modbus_nametab_tx[modbus_dev_S.pnum_tx].Timer = Timer;               //计时器
      /*************错误对应码*******************/
      modbus_nametab_tx[modbus_dev_S.pnum_tx].Error_U.byte = 0;
      /*************Modbus中的数据地址信息*******************/
      if(Reg_StartAddrNumber <= Reg_EndAddrNumber)
        {
          BigAddr = Reg_EndAddrNumber;
          SmallAddr = Reg_StartAddrNumber;
        }
      else
        {
          BigAddr = Reg_StartAddrNumber;
          SmallAddr = Reg_EndAddrNumber;
        }
      modbus_nametab_tx[modbus_dev_S.pnum_tx].Reg_StartAddrNumber = SmallAddr;   //数据起始地址
      modbus_nametab_tx[modbus_dev_S.pnum_tx].Reg_EndAddrNumber = BigAddr;     //数据终点地址
      modbus_nametab_tx[modbus_dev_S.pnum_tx].Send_StartAddrNumber = SmallAddr;  //数据起始地址
      modbus_nametab_tx[modbus_dev_S.pnum_tx].Send_DataLen = BigAddr-SmallAddr+1;          //数据域数据长度
      /*************对应的代码中的数据信息*******************/
      modbus_nametab_tx[modbus_dev_S.pnum_tx].SourceDataLenMax = modbus_nametab_tx[modbus_dev_S.pnum_tx].Send_DataLen;    //来源数据域数据长度最大值
      modbus_nametab_tx[modbus_dev_S.pnum_tx].DataSourceAddr = DataSourceAddr;     //来源数据地址
      /*************事件响应的回调函数*******************/
      modbus_nametab_tx[modbus_dev_S.pnum_tx].ComSucceedCallBack = ComSucceedCallBack;  //发送成功回调函数
      modbus_nametab_tx[modbus_dev_S.pnum_tx].OvertimeCallBack = OvertimeCallBack;//发送超时回调函数
      FrameNumber = modbus_dev_S.pnum_tx;
      modbus_dev_S.pnum_tx++;
    }
  else
    {
      FrameNumber = MODBUS_NR_OF_HOST_NAMETABE;
    }

  return FrameNumber;
}
static uint8_t modbus_HOSTAnalysisUsedPG(_s_modbus_dev *handle,uint16_t *num)
{
  //主模式下，解析一帧数据
  uint8_t functioncode;
  uint8_t j;
  uint16_t i;
  uint16_t AddrNum;
  uint16_t DataNum;

  if((MODBUS_RX_STA&DATABUF_READY) &&
  (0 == MODBUS_TX_STA))
    {
      //是否有一帧数据
      MODBUS_RX_STA = 0;

      for(i=0; i<handle->pnum_tx; i++)
        {
          if(MODBUS_PG_TX_TxING == handle->para_tx[i].PGState_E)
            {
              //帧命令是在等待响应的
              functioncode = MODBUS_RX_BUF[1]&0x7f;
              if((handle->para_tx[i].address == MODBUS_RX_BUF[0]) &&
              (handle->para_tx[i].function == functioncode))
                {
                  //地址与功能码是对应的对应
                  if((MODBUS_RX_BUF[1]&0x80) == 0x80)
                    {
                      switch(MODBUS_RX_BUF[2])
                        {
                        case 0x01:
                          handle->para_tx[i].Error_U.bit.Err_RX_FunctionCode = 1;
                          break;
                        case 0x02:
                          handle->para_tx[i].Error_U.bit.Err_RX_DataAddr = 1;
                          break;
                        case 0x03:
                          handle->para_tx[i].Error_U.bit.Err_RX_OutNum = 1;
                          break;
                        case 0x04:
                          handle->para_tx[i].Error_U.bit.Err_RX_Data = 1;
                          break;
                        default:
                          handle->para_tx[i].Error_U.bit.Err_RX_Else = 1;
                          break;
                        }
                      *num = i;
                      return TRUE;
                    }
                  else
                    {
                      switch(handle->para_tx[i].function)
                        {
#ifdef FUNCTION_CODE_READ_BITIO
                        case FUNCTION_CODE_READ_BITIO: //读线圈
                          break;
#endif
#ifdef FUNCTION_CODE_READ_BITREG
                        case FUNCTION_CODE_READ_BITREG://读离散输入寄存器
                          break;
#endif
#ifdef FUNCTION_CODE_READ_HOLDREG
                        case FUNCTION_CODE_READ_HOLDREG: //读保持寄存器
                        {
                          AddrNum = MODBUS_RX_BUF[2]/2;
                          if(AddrNum == handle->para_tx[i].Send_DataLen)
                            {
                              //数据长度正确
                              DataNum = handle->para_tx[i].Send_StartAddrNumber - handle->para_tx[i].Reg_StartAddrNumber;
                              for(j=0; j<AddrNum; j++)
                                {
                                  //提取数据
                                  ((uint16_t*)handle->para_tx[i].DataSourceAddr)[DataNum+j] = (MODBUS_RX_BUF[(3+j*2)]<<8)|MODBUS_RX_BUF[(4+j*2)];
                                }
                              *num = i;
                              return TRUE;
                            }
                        }
                        break;
#endif
#ifdef FUNCTION_CODE_READ_INREG
                        case FUNCTION_CODE_READ_INREG://读输入寄存器
                          break;
#endif
#ifdef FUNCTION_CODE_WRITE_BITIO
                        case FUNCTION_CODE_WRITE_BITIO://写单个线圈寄存器
                          break;
#endif
#ifdef FUNCTION_CODE_WRITE_HOLDREG
                        case FUNCTION_CODE_WRITE_HOLDREG: //写单个保持寄存器
                        {
                          //提取首地址
                          AddrNum = (MODBUS_RX_BUF[2]<<8)|MODBUS_RX_BUF[3];
                          if(AddrNum == handle->para_tx[i].Send_StartAddrNumber)
                            {
                              //提取数据
                              AddrNum = (MODBUS_RX_BUF[4]<<8)|MODBUS_RX_BUF[5];
                              DataNum = ((uint16_t*)handle->para_tx[i].DataSourceAddr)[handle->para_tx[i].Send_StartAddrNumber - handle->para_tx[i].Reg_StartAddrNumber];
                              if(AddrNum == DataNum)
                                {
                                  *num = i;
                                  return TRUE;
                                }
                            }
                        }
                        break;
#endif
#ifdef FUNCTION_CODE_WRITE_BITIO_MULTI
                        case FUNCTION_CODE_WRITE_BITIO_MULTI://写多个线圈寄存器
                          break;
#endif
#ifdef FUNCTION_CODE_WRITE_HOLDREG_MULTI
                        case FUNCTION_CODE_WRITE_HOLDREG_MULTI://写多个保持寄存器
                          //提取首地址
                          AddrNum = (MODBUS_RX_BUF[2]<<8)|MODBUS_RX_BUF[3];
                          //提取寄存器数
                          DataNum = (MODBUS_RX_BUF[4]<<8)|MODBUS_RX_BUF[5];
                          if( ( AddrNum == handle->para_tx[i].Send_StartAddrNumber ) && \
                          ( DataNum == (handle->para_tx[i].Reg_EndAddrNumber - handle->para_tx[i].Send_StartAddrNumber + 1) ))
                            {
                              *num = i;
                              return TRUE;
                            }
                          break;
#endif
                        }
                    }
                }
            }
        }
    }

  return FALSE;
}
static uint8_t modbus_HOSTSendUsedPG(_s_modbus_dev *handle,uint16_t num)
{
  //主模式下，发送一帧数据
  uint16_t i;
  uint16_t crc;
  uint16_t AddrNum;
  uint8_t len;
  uint16_t Data;

  if((MODBUS_TX_STA&DATABUF_READY) == 0)
    {
      if(handle->para_tx[num].Reg_EndAddrNumber < handle->para_tx[num].Reg_StartAddrNumber)
        {
          handle->para_tx[num].Error_U.bit.Err_ItselfDataAddr = 1;
          return FALSE;
        }
      i = 0;
      MODBUS_TX_BUF[i++] = handle->para_tx[num].address;
      MODBUS_TX_BUF[i++] = handle->para_tx[num].function;
      switch(handle->para_tx[num].function)
        {
#ifdef FUNCTION_CODE_READ_BITIO
        case FUNCTION_CODE_READ_BITIO: //读线圈
          break;
#endif
#ifdef FUNCTION_CODE_READ_BITREG
        case FUNCTION_CODE_READ_BITREG://读离散输入寄存器
          break;
#endif
#ifdef FUNCTION_CODE_READ_HOLDREG
        case FUNCTION_CODE_READ_HOLDREG: //读保持寄存器
        {
          if((handle->para_tx[num].Send_StartAddrNumber < handle->para_tx[num].Reg_StartAddrNumber) ||
              (handle->para_tx[num].Send_StartAddrNumber > handle->para_tx[num].Reg_EndAddrNumber))
            {
              handle->para_tx[num].Error_U.bit.Err_ItselfDataAddr = 1;
              return FALSE;
            }
          AddrNum = handle->para_tx[num].Reg_EndAddrNumber - handle->para_tx[num].Send_StartAddrNumber + 1;
          if( (AddrNum < handle->para_tx[num].Send_DataLen) ||
              (handle->para_tx[num].Send_DataLen > 125) ||
              (handle->para_tx[num].Send_DataLen < 1))

            {
              handle->para_tx[num].Error_U.bit.Err_ItselfOutNum = 1;
              return FALSE;
            }
          //起始地址
          MODBUS_TX_BUF[i++] = (handle->para_tx[num].Send_StartAddrNumber>>8)&0x00ff;
          MODBUS_TX_BUF[i++] = handle->para_tx[num].Send_StartAddrNumber&0x00ff;
          //寄存器数量
          MODBUS_TX_BUF[i++] = (handle->para_tx[num].Send_DataLen>>8)&0x00ff;
          MODBUS_TX_BUF[i++] = handle->para_tx[num].Send_DataLen&0x00ff;
        }
        break;
#endif
#ifdef FUNCTION_CODE_READ_INREG
        case FUNCTION_CODE_READ_INREG://读输入寄存器
          break;
#endif
#ifdef FUNCTION_CODE_WRITE_BITIO
        case FUNCTION_CODE_WRITE_BITIO://写单个线圈寄存器
          break;
#endif
#ifdef FUNCTION_CODE_WRITE_HOLDREG
        case FUNCTION_CODE_WRITE_HOLDREG: //写单个保持寄存器
        {
          if((handle->para_tx[num].Send_StartAddrNumber < handle->para_tx[num].Reg_StartAddrNumber) ||
              (handle->para_tx[num].Send_StartAddrNumber > handle->para_tx[num].Reg_EndAddrNumber))
            {
              handle->para_tx[num].Error_U.bit.Err_ItselfDataAddr = 1;
              return FALSE;
            }
          AddrNum = handle->para_tx[num].Reg_EndAddrNumber - handle->para_tx[num].Reg_StartAddrNumber + 1;
          if(AddrNum != handle->para_tx[num].SourceDataLenMax)
            {
              handle->para_tx[num].Error_U.bit.Err_ItselfDataAddr = 1;
              return FALSE;
            }
          //起始地址
          MODBUS_TX_BUF[i++] = (handle->para_tx[num].Send_StartAddrNumber>>8)&0x00ff;
          MODBUS_TX_BUF[i++] = handle->para_tx[num].Send_StartAddrNumber&0x00ff;
          //寄存器值
          Data = ((uint16_t*)handle->para_tx[num].DataSourceAddr)[handle->para_tx[num].Send_StartAddrNumber - handle->para_tx[num].Reg_StartAddrNumber];
          MODBUS_TX_BUF[i++] = (Data>>8)&0x00ff;
          MODBUS_TX_BUF[i++] = Data&0x00ff;
        }
        break;
#endif
#ifdef FUNCTION_CODE_WRITE_BITIO_MULTI
        case FUNCTION_CODE_WRITE_BITIO_MULTI://写多个线圈寄存器
          break;
#endif
#ifdef FUNCTION_CODE_WRITE_HOLDREG_MULTI
        case FUNCTION_CODE_WRITE_HOLDREG_MULTI://写多个保持寄存器
          if((handle->para_tx[num].Send_StartAddrNumber < handle->para_tx[num].Reg_StartAddrNumber) ||
              (handle->para_tx[num].Send_StartAddrNumber > handle->para_tx[num].Reg_EndAddrNumber))
            {
              handle->para_tx[num].Error_U.bit.Err_ItselfDataAddr = 1;
              return FALSE;
            }
          AddrNum = handle->para_tx[num].Reg_EndAddrNumber - handle->para_tx[num].Send_StartAddrNumber + 1;
          if( (AddrNum > handle->para_tx[num].Send_DataLen) ||
              (handle->para_tx[num].Send_DataLen > 123) ||
              (handle->para_tx[num].Send_DataLen < 1) ||
          (AddrNum == 0) )

            {
              handle->para_tx[num].Error_U.bit.Err_ItselfOutNum = 1;
              return FALSE;
            }
          //起始地址
          MODBUS_TX_BUF[i++] = (handle->para_tx[num].Send_StartAddrNumber>>8)&0x00ff;
          MODBUS_TX_BUF[i++] = handle->para_tx[num].Send_StartAddrNumber&0x00ff;
          //寄存器数量
          MODBUS_TX_BUF[i++] = (AddrNum>>8)&0x00ff;
          MODBUS_TX_BUF[i++] = AddrNum&0x00ff;
          //字节数
          MODBUS_TX_BUF[i++] = AddrNum<<1;
          //寄存器值
          for(len=0; len<AddrNum; len++)
            {
              Data = ((uint16_t*)handle->para_tx[num].DataSourceAddr)[handle->para_tx[num].Send_StartAddrNumber - handle->para_tx[num].Reg_StartAddrNumber+len];
              MODBUS_TX_BUF[i++] = (Data>>8)&0x00ff;
              MODBUS_TX_BUF[i++] = Data&0x00ff;
            }
          break;
#endif
        default:
          handle->para_tx[num].Error_U.bit.Err_ItselfFunctionCode = 1;
          return FALSE;
        }
      crc = CRC_Compute(MODBUS_TX_BUF, i);
      MODBUS_TX_BUF[i++] = (crc>>8)&0x00ff;
      MODBUS_TX_BUF[i++] = crc&0x00ff;

      MODBUS_TX_STA = i;
      MODBUS_TX_STA |= DATABUF_READY;

      return TRUE;
    }

  return FALSE;
}
static void modbus_HOSTScan(_s_modbus_dev *handle)
{
  uint16_t counts;
  uint16_t num;
  /**************************************************************************************
  ***************************************************************************************
  **************************************接收部分*****************************************
  ***************************************************************************************
  **************************************************************************************/
  if(modbus_HOSTAnalysisUsedPG(handle,&num))
    {
      if(MODBUS_PG_TX_TxING == handle->para_tx[num].PGState_E)
        {
          handle->para_tx[num].PGState_E = MODBUS_PG_TX_SUC;
        }
    }
  /**************************************************************************************
  ***************************************************************************************
  **************************************发送部分*****************************************
  ***************************************************************************************
  **************************************************************************************/
  for (counts=0; counts<handle->pnum_tx; counts++)
    {
      if (handle->para_tx[counts].PGState_E == MODBUS_PG_TX_REQ)
        {
          //帧通讯等待状态
          if( (handle->para_tx[counts].TransType_E == MODBUS_ASYNC) ||
          ( (handle->para_tx[counts].TransType_E == MODBUS_CYCLIC) &&
          (TRUE == Tim_CheckTimeOut(handle->para_tx[counts].Timer,
                                    handle->Timer,
                                    handle->para_tx[counts].TransRate))))
            {
              //帧通讯就绪
              if(MODBUS_SEND_FREE == handle->SendState_E)
                {
                  //总线空闲，发送此帧数据
                  if(modbus_HOSTSendUsedPG(handle,counts))
                    {
                      //发送成功
                      handle->para_tx[counts].Error_U.byte = 0;
                      handle->para_tx[counts].PGState_E = MODBUS_PG_TX_TxING;
                      handle->para_tx[counts].Timer = handle->Timer;
                      handle->SendState_E = MODBUS_SEND_TXING;

#if (MODBUS_BLOCKMODE == 1)
                      break;
#endif
                    }
                  else
                    {
                      //硬件发送异常，重新处理时间时此帧一直处于就绪状态
                      handle->para_tx[counts].Timer = handle->Timer - handle->para_tx[counts].TransRate;
                    }
                }
              else
                {
                  //总线被别的帧占有，重新处理时间时此帧一直处于就绪状态
                  handle->para_tx[counts].Timer = handle->Timer - handle->para_tx[counts].TransRate;
                }
            }
        }
      else if(handle->para_tx[counts].PGState_E == MODBUS_PG_TX_TxING)
        {
          //帧通讯中
          if(Tim_CheckTimeOut(handle->para_tx[counts].Timer,handle->Timer,MODBUS_OVERTIME))
            {
              //通讯超时
              handle->para_tx[counts].Error_U.bit.Err_OverTime = 1;
              handle->para_tx[counts].txerrnum++;
              if(handle->para_tx[counts].txerrnum >= MODBUS_OVERTIME_NUM)
                {
                  //多次未响应,进入报错模式（被注销）
                  handle->para_tx[counts].Error_U.bit.Err_SendCnt = 1;
                  handle->para_tx[counts].PGState_E = MODBUS_PG_TX_ERR;
                  if(handle->para_tx[counts].OvertimeCallBack != 0)
                    {
                      //回调发送超时回调函数
                      handle->para_tx[counts].OvertimeCallBack(&handle->para_tx[counts]);
                    }
                }
              else
                {
                  //重新开始发送
#if (MODBUS_BLOCKMODE == 1)
                  handle->para_tx[counts].Timer = handle->Timer - handle->para_tx[counts].TransRate;
#else
                  handle->para_tx[counts].Timer = handle->Timer;
#endif
                  handle->para_tx[counts].PGState_E = MODBUS_PG_TX_REQ;
                }
              handle->SendState_E = MODBUS_SEND_FREE; //总线为空闲
#if (MODBUS_BLOCKMODE == 1)
              break;
#endif
            }
        }
      else if (handle->para_tx[counts].PGState_E == MODBUS_PG_TX_SUC)
        {
          //帧通讯成功
          handle->para_tx[counts].txerrnum = 0;
          if(handle->para_tx[counts].TransType_E  == MODBUS_CYCLIC)
            {
              //帧为循环模式，帧通讯状态为就绪状态
              handle->para_tx[counts].Timer = handle->Timer;
              handle->para_tx[counts].PGState_E = MODBUS_PG_TX_REQ;
            }
          else
            {
              //帧通讯状态为空闲状态
              handle->para_tx[counts].PGState_E = MODBUS_PG_TX_FREE;
            }
          handle->SendState_E = MODBUS_SEND_FREE; //总线为空闲
          if(handle->para_tx[counts].ComSucceedCallBack != 0)
            {
              //执行通讯成功回调函数
              handle->para_tx[counts].ComSucceedCallBack(&handle->para_tx[counts]);
            }
        }
    }
}
#else
static uint16_t modbus_CreateCommand(uint8_t function,//功能码 范围1-255（128-255为异常响应保留）
                                     uint16_t Reg_StartAddrNumber,   //数据起始地址
                                     uint16_t Reg_EndAddrNumber,     //数据终点地址
                                     void *DataSourceAddr,     //来源数据地址
                                     uint8_t (*ComSucceedCallBack)(struct _s_ModbusPDU *pQ)) //发送成功回调函数
{
  uint16_t BigAddr;
  uint16_t SmallAddr;
  uint16_t FrameNumber;

  if(modbus_dev_S.pnum_rx < MODBUS_NR_OF_HOST_NAMETABE)
    {
      /*************Modbus 信息*******************/
      modbus_nametab_rx[modbus_dev_S.pnum_rx].function = function;
      /*************Modbus中的数据地址信息*******************/
      if(Reg_StartAddrNumber <= Reg_EndAddrNumber)
        {
          BigAddr = Reg_EndAddrNumber;
          SmallAddr = Reg_StartAddrNumber;
        }
      else
        {
          BigAddr = Reg_StartAddrNumber;
          SmallAddr = Reg_EndAddrNumber;
        }
      modbus_nametab_rx[modbus_dev_S.pnum_rx].Reg_StartAddrNumber = SmallAddr;   //数据起始地址
      modbus_nametab_rx[modbus_dev_S.pnum_rx].Reg_EndAddrNumber = BigAddr;     //数据终点地址
      /*************对应的代码中的数据信息*******************/
      modbus_nametab_rx[modbus_dev_S.pnum_rx].DataSourceAddr = DataSourceAddr;     //来源数据地址
      /*************事件响应的回调函数*******************/
      modbus_nametab_rx[modbus_dev_S.pnum_rx].ComSucceedCallBack = ComSucceedCallBack;  //发送成功回调函数
      FrameNumber = modbus_dev_S.pnum_rx;
      modbus_dev_S.pnum_rx++;
    }
  else
    {
      FrameNumber = MODBUS_NR_OF_HOST_NAMETABE;
    }

  return FrameNumber;
}
#ifdef FUNCTION_CODE_READ_HOLDREG
static uint8_t modbus_SLAVEParsing_Function_03(_s_ModbusPDU *pQ)
{
  uint16_t Addr;
  uint16_t DataLen;
  uint16_t i;

  Addr = (MODBUS_RX_BUF[2]<<8)|MODBUS_RX_BUF[3];
  DataLen = (MODBUS_RX_BUF[4]<<8)|MODBUS_RX_BUF[5];
  if((DataLen >= 1) && (DataLen <= 125))
    {
      if((Addr >= pQ->Reg_StartAddrNumber) &&
          (Addr <= pQ->Reg_EndAddrNumber) &&
          ((Addr+DataLen-1) <= pQ->Reg_EndAddrNumber))
        {
          MODBUS_TX_BUF[2] = DataLen<<1;
          Addr = Addr - pQ->Reg_StartAddrNumber;
          for(i=0; i<DataLen; i++)
            {
              MODBUS_TX_BUF[i*2+3] = (((uint16_t*)pQ->DataSourceAddr)[Addr+i]>>8)&0x00ff;
              MODBUS_TX_BUF[i*2+4] = ((uint16_t*)pQ->DataSourceAddr)[Addr+i]&0x00ff;
            }

          return 0x00;
        }
      else
        return 0x02;//不支持的寄存器地址
    }
  else
    return 0x03;//不支持的寄存器数量
}
#endif
#ifdef FUNCTION_CODE_WRITE_HOLDREG
static uint8_t modbus_SLAVEParsing_Function_06(_s_ModbusPDU *pQ)
{
  uint16_t Addr;
  uint16_t Data;

  MODBUS_TX_BUF[2] = MODBUS_RX_BUF[2];
  MODBUS_TX_BUF[3] = MODBUS_RX_BUF[3];
  MODBUS_TX_BUF[4] = MODBUS_RX_BUF[4];
  MODBUS_TX_BUF[5] = MODBUS_RX_BUF[5];

  Addr = (MODBUS_TX_BUF[2]<<8)|MODBUS_TX_BUF[3];

  if((Addr >= pQ->Reg_StartAddrNumber) &&
      (Addr <= pQ->Reg_EndAddrNumber))
    {
      Data = (MODBUS_TX_BUF[4]<<8)|MODBUS_TX_BUF[5];
      Addr = Addr - pQ->Reg_StartAddrNumber;
      ((uint16_t*)pQ->DataSourceAddr)[Addr] = Data;

      return 0x00;
    }
  else
    return 0x02;//不支持的寄存器地址
}
#endif
#ifdef FUNCTION_CODE_WRITE_HOLDREG_MULTI
static uint8_t modbus_SLAVEParsing_Function_10(_s_ModbusPDU *pQ,uint16_t Len)
{
  uint16_t Addr;
  uint16_t Data;
  uint16_t DataLen;
  uint16_t i;

  MODBUS_TX_BUF[2] = MODBUS_RX_BUF[2];
  MODBUS_TX_BUF[3] = MODBUS_RX_BUF[3];
  MODBUS_TX_BUF[4] = MODBUS_RX_BUF[4];
  MODBUS_TX_BUF[5] = MODBUS_RX_BUF[5];

  Addr = (MODBUS_TX_BUF[2]<<8)|MODBUS_TX_BUF[3];
  DataLen = (MODBUS_TX_BUF[4]<<8)|MODBUS_TX_BUF[5];
  if((DataLen >= 1) &&
      (DataLen <= 123) &&
  ((DataLen<<1) == MODBUS_RX_BUF[6]) &&
  ((MODBUS_RX_BUF[6]+9) == Len))
    {
      if((Addr >= pQ->Reg_StartAddrNumber) &&
          (Addr <= pQ->Reg_EndAddrNumber) &&
          ((Addr+DataLen-1) <= pQ->Reg_EndAddrNumber))
        {
          Addr = Addr - pQ->Reg_StartAddrNumber;
          for(i=0; i<DataLen; i++)
            {
              Data = (MODBUS_RX_BUF[2*i+7]<<8)|MODBUS_RX_BUF[2*i+8];
              ((uint16_t*)pQ->DataSourceAddr)[Addr+i] = Data;
            }

          return 0;
        }
      else
        return 0x02;//不支持的寄存器地址
    }
  else
    return 0x03;//不支持的寄存器数量
}
#endif
static uint8_t modbus_SLAVEAnalysisUsedPG(_s_modbus_dev *handle)
{
  //从模式下，解析一帧数据
  uint8_t functioncode;
  uint8_t ErrCord;
  uint16_t Len;
  uint16_t crc;
  uint16_t j;
  uint8_t err;
  uint16_t i;

  err = 1;
  if((MODBUS_RX_STA&DATABUF_READY) &&
  (0 == MODBUS_TX_STA))
    {
      //是否有一帧数据
      Len = MODBUS_RX_STA&(~DATABUF_READY);
      MODBUS_RX_STA = 0;

      functioncode = MODBUS_RX_BUF[1]&0x7f;
      MODBUS_TX_BUF[0] = SLAVE_ADDRESS;
      MODBUS_TX_BUF[1] = functioncode;
      ErrCord = 0x01;//不支持的功能码
      err = 0;
      for(j=0; j<handle->pnum_rx; j++)
        {
          if(handle->para_rx[j].function == functioncode)
            {
              switch(functioncode)
                {
#ifdef FUNCTION_CODE_READ_BITIO
                case FUNCTION_CODE_READ_BITIO: //读线圈
                  break;
#endif
#ifdef FUNCTION_CODE_READ_BITREG
                case FUNCTION_CODE_READ_BITREG://读离散输入寄存器
                  break;
#endif
#ifdef FUNCTION_CODE_READ_HOLDREG
                case FUNCTION_CODE_READ_HOLDREG: //读保持寄存器
                {
                  ErrCord = modbus_SLAVEParsing_Function_03(&handle->para_rx[j]);
                  if(ErrCord == 0)
                    i = MODBUS_TX_BUF[2] + 3;
                }
                break;
#endif
#ifdef FUNCTION_CODE_READ_INREG
                case FUNCTION_CODE_READ_INREG://读输入寄存器
                  break;
#endif
#ifdef FUNCTION_CODE_WRITE_BITIO
                case FUNCTION_CODE_WRITE_BITIO://写单个线圈寄存器
                  break;
#endif
#ifdef FUNCTION_CODE_WRITE_HOLDREG
                case FUNCTION_CODE_WRITE_HOLDREG: //写单个保持寄存器
                {
                  ErrCord = modbus_SLAVEParsing_Function_06(&handle->para_rx[j]);
                  if(ErrCord == 0)
                    i = 6;
                }
                break;
#endif
#ifdef FUNCTION_CODE_WRITE_BITIO_MULTI
                case FUNCTION_CODE_WRITE_BITIO_MULTI://写多个线圈寄存器
                  break;
#endif
#ifdef FUNCTION_CODE_WRITE_HOLDREG_MULTI
                case FUNCTION_CODE_WRITE_HOLDREG_MULTI: //写多个保持寄存器
                {
                  ErrCord = modbus_SLAVEParsing_Function_10(&handle->para_rx[j],Len);
                  if(ErrCord == 0)
                    i = 6;
                }
                break;
#endif
                default:
                  ErrCord = 0x01;//不支持的功能码
                  break;
                }
            }
          if(ErrCord == 0)
            {
              if(handle->para_rx[j].ComSucceedCallBack != 0)
                ErrCord = handle->para_rx[j].ComSucceedCallBack(&handle->para_rx[j]);
              break;
            }
        }
      if(ErrCord != 0)
        {
          MODBUS_TX_BUF[1] |= 0x80;
          MODBUS_TX_BUF[2] = ErrCord;
          i = 3;
        }
      crc = CRC_Compute(MODBUS_TX_BUF, i);
      MODBUS_TX_BUF[i++] = (crc>>8)&0x00ff;
      MODBUS_TX_BUF[i++] = crc&0x00ff;

      MODBUS_TX_STA = i;
      MODBUS_TX_STA |= DATABUF_READY;
    }

  return err;
}
static void modbus_SLAVEScan(_s_modbus_dev *handle)
{
  modbus_SLAVEAnalysisUsedPG(handle);
}
#endif


static void modbus_init(void)
{
  memset(MODBUS_TX_BUF, 0, sizeof(uint8_t)*MODBUS_TX_LEN);
  memset(MODBUS_RX_BUF, 0, sizeof(uint8_t)*MODBUS_RX_LEN);
  MODBUS_TX_STA = 0;
  MODBUS_RX_STA = 0;
#if (MODBUS_MODE == MODBUS_MODE_HOST)
  memset(modbus_dev_S.para_tx, 0, sizeof(_s_modbus_dev)*MODBUS_NR_OF_HOST_NAMETABE);
#else
  modbus_dev_S.mode_E = MODBUS_SLAVE;
  memset(modbus_dev_S.para_rx, 0, sizeof(_s_modbus_dev)*MODBUS_NR_OF_HOST_NAMETABE);
#endif
  ModbusInitialUsedPGs();
}
static void modbus_scan(void)
{
#if (MODBUS_MODE == MODBUS_MODE_HOST)
  modbus_HOSTScan(&modbus_dev_S);
#else
  modbus_SLAVEScan(&modbus_dev_S);
#endif

}
/** @brief  uartudp receive
  * @param
  * @retval
  */
static uint8_t modbus_receive(uint8_t *str,uint16_t len)
{
  uint16_t i;
  uint8_t crc_h;
  uint8_t crc_l;
  uint16_t crc;

  if(len > 4)
    {
      /***********效验CRC************/
      crc = CRC_Compute(str, len-2);
      crc_h = (crc>>8)&0x00ff;
      crc_l = crc&0x00ff;
      if((crc_h == str[len-2]) &&
      (crc_l == str[len-1]) )
        {
#if (MODBUS_MODE == MODBUS_MODE_SLAVE)
          if(str[0] != SLAVE_ADDRESS)
            return FALSE;
#endif
          for(i=0; i<len; i++)
            {
              MODBUS_RX_BUF[i] = str[i];
            }
          MODBUS_RX_STA = len;
          MODBUS_RX_STA |= DATABUF_READY;

          return TRUE;
        }
    }

  return FALSE;
}
/** @brief  uartudp transfer
  * @param
  * @retval 1：发送成功 0：忙取消发送
  */
static uint8_t modbus_transfer(uint8_t *str,uint16_t *len) //数据发送
{
  uint16_t i;

  if(MODBUS_TX_STA&DATABUF_READY)
    {
      *len = MODBUS_TX_STA&(~DATABUF_READY);
      for(i=0; i<*len; i++)
        {
          str[i] = MODBUS_TX_BUF[i];
        }
      MODBUS_TX_STA = 0;

      return 1;
    }
  return 0;
}
static void modbus_Timer(void)
{
  modbus_dev_S.Timer++;
}






























__attribute__((weak)) void ModbusInitialUsedPGs(void)
{


}








