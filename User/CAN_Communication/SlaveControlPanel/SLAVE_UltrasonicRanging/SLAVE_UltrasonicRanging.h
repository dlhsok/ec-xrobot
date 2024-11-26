#ifndef __SLAVE_UltrasonicRanging_H__
#define __SLAVE_UltrasonicRanging_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct UltrasonicRangingDef_t
{    
    uint8_t  CAN_Number;

	union 
	{
		#define PG_UR_RXSTE_PARA_DATA_LEN  8
		uint8_t Datum[PG_UR_RXSTE_PARA_DATA_LEN];
		struct
		{
			union
			{
				uint8_t byte;
				#define SET_PARA_ID_FLG          0x01
				#define SET_PARA_ENABLED_FLG     0x02
				#define SET_PARA_MEMORY_FLG      0x04
				struct{
					uint8_t modifyidflg    :1;
					uint8_t enabledflg     :1;
					uint8_t memoryflg      :1;
					uint8_t retain         :5;
				}bit;
			}mode;
			uint8_t Serial_Number;
			union
			{
				uint8_t byte;
				struct{
					uint8_t enabled1    :1;
					uint8_t enabled2    :1;
					uint8_t enabled3    :1;
					uint8_t enabled4    :1;
					uint8_t enabled5    :1;
					uint8_t enabled6    :1;
					uint8_t retain      :2;
				}bit;
			}enabled;
			
			uint8_t retain1;
			uint8_t retain2;
			uint8_t retain3;
			uint8_t retain4;
			uint8_t retain5;
		}DATE;
	}UltrasonicRanging_SetPara;
  
	union 
	{  
		#define PG_TXDISTANCE_DATA1_LEN 8
		uint8_t Datum[PG_TXDISTANCE_DATA1_LEN];
		struct{
			uint16_t Distance1;
			uint16_t Distance2;
			uint16_t Distance3;
			uint16_t Distance4;
		}DATE;
	}UltrasonicRanging_UploadData1;
	union 
	{  
		#define PG_TXDISTANCE_DATA2_LEN 4
		uint8_t Datum[PG_TXDISTANCE_DATA2_LEN];
		struct{
			uint16_t Distance5;
			uint16_t Distance6;
		}DATE;
	}UltrasonicRanging_UploadData2;

    struct UltrasonicRangingDef_t* next;
} UltrasonicRangingDef_t;
/* 宏定义 --------------------------------------------------------------------*/
#define SLAVE_UltrasonicRanging_ENABLED 1
#define UltrasonicRanging_Addr     8  //超声波地址



/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
uint8_t SLAVE_UltrasonicRanging_Init(UltrasonicRangingDef_t* handle,uint8_t list);
void SLAVE_UltrasonicRanging_NumberSet(UltrasonicRangingDef_t* handle,uint8_t number);
uint16_t SLAVE_UltrasonicRanging_DistanceRead(UltrasonicRangingDef_t* handle,uint8_t ch);
void SLAVE_UltrasonicRanging_MovementMemory(UltrasonicRangingDef_t* handle);
void SLAVE_UltrasonicRanging_CHEnabled(UltrasonicRangingDef_t* handle,
	                                   uint8_t ch1,
                                       uint8_t ch2,
                                       uint8_t ch3,
                                       uint8_t ch4,
                                       uint8_t ch5,
                                       uint8_t ch6);
#endif  // __SLAVE_UltrasonicRanging_H__

