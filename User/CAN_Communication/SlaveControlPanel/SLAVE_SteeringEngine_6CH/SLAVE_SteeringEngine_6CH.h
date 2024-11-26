#ifndef __SLAVE_SteeringEngine_6CH_H__
#define __SLAVE_SteeringEngine_6CH_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct SteeringEngine_6CH_T
{    
    uint8_t  CAN_Number;

	union 
	{
		#define PG_RXSTE_PARA_DATA_LEN      8
		uint8_t Datum[PG_RXSTE_PARA_DATA_LEN];
		struct
		{
			union
			{
				uint8_t byte;
				#define SETPARA_ID_FLG          0x01
				#define SETPARA_MEMORY_FLG      0x02
				#define SETPARA_ACTFULLRUN_FLG  0x04
				#define SETPARA_ACTFULLSTOP_FLG 0x08
				struct{
					uint8_t modifyidflg    :1;
					uint8_t memoryflg      :1;
					uint8_t actFullRunflg  :1;
					uint8_t actFullStopflg :1;
					uint8_t retain         :4;
				}bit;
			}mode;
			uint8_t Serial_Number;
			uint8_t actFullnum;
			uint8_t retain1;
			uint8_t retain2;
			uint8_t retain3;
			uint8_t retain4;
			uint8_t retain5;
		}DATE;
	}SteeringEngine_SetPara;
  
    union 
	{  
		#define PG_RXSTE_ServoCtrl_DATA_LEN 8
		uint8_t Datum[PG_RXSTE_ServoCtrl_DATA_LEN];
		struct{
			uint16_t ServoPwmDutySetCh1;
			uint16_t TimeCh1;
			uint16_t ServoPwmDutySetCh2;
			uint16_t TimeCh2;
		}DATE;
	}SteeringEngine_ControlData[3];

    struct SteeringEngine_6CH_T* next;
} SteeringEngine6CHDef_t;
/* 宏定义 --------------------------------------------------------------------*/
#define SLAVE_SteeringEngine6CH_ENABLED 1
#define SteeringEngine6CHPlate_Addr     4  //舵机板地址

#define SteeringEngine_CHANNEL1_CHANNEL2 0
#define SteeringEngine_CHANNEL3_CHANNEL4 1
#define SteeringEngine_CHANNEL5_CHANNEL6 2




/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/

uint8_t SLAVE_SteeringEngine6CH_Init(SteeringEngine6CHDef_t* handle,uint8_t list);
void SLAVE_SteeringEngine6CH_NumberSet(SteeringEngine6CHDef_t* handle,uint8_t number);
void SLAVE_SteeringEngine6CH_FullActRun(SteeringEngine6CHDef_t* handle,uint8_t actFullnum);
void SLAVE_SteeringEngine6CH_FullActStop(SteeringEngine6CHDef_t* handle);
void SLAVE_SteeringEngine6CH_MotorControl(SteeringEngine6CHDef_t* handle,
										  uint8_t  Channel,
                                          uint16_t ServoPwmDutySetCh1,
									      uint16_t time1,
									      uint16_t ServoPwmDutySetCh2,
									      uint16_t time2);
void SLAVE_SteeringEngine6CH_MoreMotorControl(SteeringEngine6CHDef_t* handle,
                                           uint16_t ServoPwmDutySetCh1,uint16_t time1,
									       uint16_t ServoPwmDutySetCh2,uint16_t time2,
                                           uint16_t ServoPwmDutySetCh3,uint16_t time3,
										   uint16_t ServoPwmDutySetCh4,uint16_t time4,
									       uint16_t ServoPwmDutySetCh5,uint16_t time5,
                                           uint16_t ServoPwmDutySetCh6,uint16_t time6
                                          );
void SLAVE_SteeringEngine6CH_MovementMemory(SteeringEngine6CHDef_t* handle);



#endif  // __SLAVE_SteeringEngine_6CH_H__

