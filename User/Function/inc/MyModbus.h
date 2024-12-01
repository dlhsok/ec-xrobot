#ifndef __MYMODBUS_H__
#define __MYMODBUS_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
/* 宏定义 --------------------------------------------------------------------*/
#define MODBUS_RX_LEN 100
#define MODBUS_TX_LEN 200

#define MODBUS_MODE_HOST  0
#define MODBUS_MODE_SLAVE 1

#define MODBUS_MODE MODBUS_MODE_HOST //模式选择

//#define FUNCTION_CODE_READ_BITIO           0X01 //读线圈
//#define FUNCTION_CODE_READ_BITREG          0X02 //读离散输入寄存器
#define FUNCTION_CODE_READ_HOLDREG         0X03 //读保持寄存器
//#define FUNCTION_CODE_READ_INREG           0X04 //读输入寄存器
//#define FUNCTION_CODE_WRITE_BITIO          0X05 //写单个线圈寄存器
#define FUNCTION_CODE_WRITE_HOLDREG        0X06 //写单个保持寄存器
//#define FUNCTION_CODE_WRITE_BITIO_MULTI    0X0F //写多个线圈寄存器
#define FUNCTION_CODE_WRITE_HOLDREG_MULTI  0X10 //写多个保持寄存器

#define MODBUS_NR_OF_HOST_NAMETABE  4  //主从模式下帧的个数，不能为0
#define MODBUS_BLOCKMODE 0
#define MODBUS_OVERTIME  100    //超时时间，单位为modbus_dev_S.timer执行频率
#define MODBUS_OVERTIME_NUM 5  //超时次数,超过这个次数此帧会被注销

#if (MODBUS_MODE == MODBUS_MODE_SLAVE)
#include "data.h"
#define SLAVE_ADDRESS 1
#endif
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
  MODBUS_PG_TX_FREE = 0,  //空闲
  MODBUS_PG_TX_REQ,       //就绪
  MODBUS_PG_TX_TxING,     //发送中
  MODBUS_PG_TX_SUC,       //发送成功
  MODBUS_PG_TX_ERR,       //发送失败
} _e_PGState; //一帧命令的运行状态

typedef enum
{
  MODBUS_ASYNC = 0,  //异步模式
  MODBUS_CYCLIC,     //循环模式
} _e_TransType; //一帧命令的传输类型
#if (MODBUS_MODE == MODBUS_MODE_HOST)
typedef struct _s_ModbusPDU
{
  /*************Modbus 信息*******************/
  uint8_t      address;             //地址域
  uint8_t      function;            //功能码 范围1-255（128-255为异常响应保留）
  /*************帧 信息*******************/
  _e_PGState   PGState_E;           //PDU运行状态
  _e_TransType TransType_E;         //PDU传输类型
  uint8_t      txerrnum;            //发送失败次数
  uint16_t     TransRate;	          //循环速率，单位ms
  uint16_t     Timer;               //计时器
  /*************错误对应码*******************/
  union
  {
    uint16_t byte;
    struct
    {
      uint16_t Err_RX_FunctionCode :1;       //功能码异常
      uint16_t Err_RX_OutNum       :1;       //输出数量异常
      uint16_t Err_RX_DataAddr     :1;       //数据地址异常
      uint16_t Err_RX_Data         :1;       //数据异常
      uint16_t Err_RX_Else         :1;       //其他异常
      uint16_t Err_OverTime     :1;       //发送超时
      uint16_t Err_SendCnt      :1;       //异常发送次数异常
      uint16_t Err_ItselfFunctionCode :1; //自身功能码异常
      uint16_t Err_ItselfOutNum       :1; //自身输出数量异常
      uint16_t Err_ItselfDataAddr     :1; //自身数据地址异常
      uint16_t Reserve          :6;       //保留
    } bit;
  } Error_U;
  /*************Modbus中的数据地址信息*******************/
  uint16_t     Reg_StartAddrNumber;   //数据起始地址
  uint16_t     Reg_EndAddrNumber;     //数据终点地址
  uint16_t     Send_StartAddrNumber;  //数据起始地址
  uint16_t     Send_DataLen;          //数据域数据长度
  /*************对应的代码中的数据信息*******************/
  uint16_t     SourceDataLenMax;    //来源数据域数据长度最大值
  void         *DataSourceAddr;     //来源数据地址
  /*************事件响应的回调函数*******************/
  void (*ComSucceedCallBack)(struct _s_ModbusPDU *pQ);  //发送成功回调函数
  void (*OvertimeCallBack)(struct _s_ModbusPDU *pQ); //发送超时回调函数
} _s_ModbusPDU; //Modbus一帧命令的数据结构
#else
typedef struct _s_ModbusPDU
{
  /*************Modbus 信息*******************/
  uint8_t      function;            //功能码 范围1-255（128-255为异常响应保留）
  /*************Modbus中的数据地址信息*******************/
  uint16_t     Reg_StartAddrNumber;   //数据起始地址
  uint16_t     Reg_EndAddrNumber;     //数据终点地址
  /*************对应的代码中的数据信息*******************/
  void         *DataSourceAddr;     //来源数据地址
  /*************事件响应的回调函数*******************/
  uint8_t (*ComSucceedCallBack)(struct _s_ModbusPDU *pQ);
} _s_ModbusPDU; //Modbus一帧命令的数据结构
#endif
typedef enum
{
  MODBUS_UNDEFINED = 0,
  MODBUS_SLAVE,
  MODBUS_HOST,
} _e_mode;//协议模式

typedef enum
{
  MODBUS_SEND_FREE = 0, //空闲
  MODBUS_SEND_TXING,    //有命令正在通讯
} _e_SendState; //总线状态
typedef struct _s_modbus_dev
{
  _e_mode  mode_E;      //模式,保留，主模式，从模式
  _e_SendState SendState_E; //总线发送状态
  uint16_t Timer;      //运行时间
#if (MODBUS_MODE == MODBUS_MODE_HOST)
  uint16_t pnum_tx;    //命令发送个数
  _s_ModbusPDU *para_tx; //指向发送命令的指针
#else
  uint16_t pnum_rx;    //命令接收个数
  _s_ModbusPDU *para_rx; //指向接收命令的指针
#endif
  /*************指向运行的函数****************/
  void (*init)(void);
  void (*TimerActuator)(void);
  void (*scan)(void);
  uint8_t (*receive)(uint8_t *str,uint16_t len);
  uint8_t (*transfer)(uint8_t *str,uint16_t *len);
#if (MODBUS_MODE == MODBUS_MODE_HOST)
  uint16_t (*CreateSendCommand)(uint8_t address,//地址域
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
  uint16_t (*CreateSendCommand)(uint8_t function,//功能码 范围1-255（128-255为异常响应保留）
                                uint16_t Reg_StartAddrNumber,   //数据起始地址
                                uint16_t Reg_EndAddrNumber,     //数据终点地址
                                void *DataSourceAddr,     //来源数据地址
                                uint8_t (*ComSucceedCallBack)(struct _s_ModbusPDU *pQ)); //发送成功回调函数
#endif
} _s_modbus_dev;

/* 扩展变量 ------------------------------------------------------------------*/
extern uint8_t  MODBUS_RX_BUF[MODBUS_RX_LEN];
extern uint16_t MODBUS_RX_STA;
extern uint8_t  MODBUS_TX_BUF[MODBUS_TX_LEN];
extern uint16_t MODBUS_TX_STA;
#if (MODBUS_MODE == MODBUS_MODE_HOST)
extern _s_ModbusPDU modbus_nametab_tx[MODBUS_NR_OF_HOST_NAMETABE];
#else
extern _s_ModbusPDU modbus_nametab_rx[MODBUS_NR_OF_HOST_NAMETABE];
#endif
extern	_s_modbus_dev modbus_dev_S;
/* 函数声明 ------------------------------------------------------------------*/
uint16_t CRC_Compute(uint8_t *puchMsg, uint16_t usDataLen);
void ModbusInitialUsedPGs(void);
void modbus_PGEnable( uint8_t  address,
                      uint8_t  function,
                      uint16_t AddrNumber);
void modbus_PGDisable( uint8_t  address,
                       uint8_t  function,
                       uint16_t AddrNumber);
#if (MODBUS_MODE == MODBUS_MODE_HOST)
void modbus_TT_ASYNC_Enable( uint8_t  address,
                             uint8_t  function,
                             uint16_t Send_StartAddrNumber,
                             uint16_t Send_EndAddrNumber );
#endif
#endif  // __MYMODBUS_H__

