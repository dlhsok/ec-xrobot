#ifndef __SLAVE_SteppingMotor_H__
#define __SLAVE_SteppingMotor_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct SteppingMotor_T
{    
    uint8_t  CAN_Number;
    
	union 
    {
        #define PG_RXSTEPPINGMOTOR_SERIAL_NUMBER_LEN 1
        uint8_t Datum[PG_RXSTEPPINGMOTOR_SERIAL_NUMBER_LEN];
        struct{
            uint8_t Serial_Number      ; //serial number
        }DATE;
    }SteppingMotor_SerialNumber;//序号修改 

    union 
    {  
        #define PG_TXSTEPPINGMOTORDATA_LEN    4
        uint8_t Datum[PG_TXSTEPPINGMOTORDATA_LEN];
        struct{
            uint8_t  run_state;
            uint8_t  enabled_state;
            uint16_t Fault;
        }DATE;
    }SteppingMotor_UploadData;//数据上传结构体
    
    union 
    {  
        #define PG_SETSTEPPINGMOTORDATA_LEN   8
        uint8_t Datum[PG_SETSTEPPINGMOTORDATA_LEN];
        struct{
            int16_t  speed_v0;     
            int16_t  speed_vt;     
            uint16_t step_out;     
            uint16_t EnabledTime_MAX; 
        }DATE;
    }SteppingMotor_ControlData;
    
    union 
    {
        #define PG_ACKSTEPPINGMOTORDATA_LEN   1
        uint8_t Datum[PG_ACKSTEPPINGMOTORDATA_LEN];
        struct
        {
            #define ACK_START 2
            #define ACK_OK 0
            #define ACK_NO 1
            uint8_t ack;
            
        }DATE;
    }SteppingMotor_ControlDataAck;
    
    struct SteppingMotor_T* next;
} SteppingMotorDef_t;
/* 宏定义 --------------------------------------------------------------------*/
#define SLAVE_SteppingMotor_ENABLED 1
#define SLAVE_SteppingMotorPlate_Addr     5  //步进电机板地址


#define RUNSTA_STOP  0   //停止模式
#define RUNSTA_SPEED 1   //速度模式
#define RUNSTA_STEP  2   //步数模式

#define ENABLED_OFF  0   //电机失能
#define ENABLED_ON   1   //电机使能

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
uint8_t SLAVE_SteppingMotor_Init(SteppingMotorDef_t* handle,uint8_t list);
void SLAVE_SteppingMotor_NumberSet(SteppingMotorDef_t* handle,uint8_t number);
uint8_t SLAVE_SteppingMotor_ReadEnabledState(SteppingMotorDef_t* handle);
uint8_t SLAVE_SteppingMotor_ReadRunState(SteppingMotorDef_t* handle);
uint8_t SLAVE_SteppingMotor_Control(SteppingMotorDef_t* handle,
                                    int16_t  speed_v0,     //初始速度
									int16_t  speed_vt,     //最大速度，为0时为停止
                                    uint16_t step_out,     //为0是速度模式，不为0是步数模式
                                    uint16_t EnabledTime_MAX 
                                   );

#endif  // __SLAVE_SteppingMotor_H__

