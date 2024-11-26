#ifndef __SLAVE_ExpansionBoardInput_H__
#define __SLAVE_ExpansionBoardInput_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct SingleHighSignalData_T
{
    uint8_t  Enabled;
    uint8_t  sta;
    uint16_t cnt;   
}SingleInputSignalDataDef_t;


typedef struct ExpansionBoardInput_T
{    
    uint8_t  CAN_Number;
    
    union  
    {
        #define PG_RXExpansionBoard_SERIAL_NUMBER_LEN 1
        uint8_t Datum[PG_RXExpansionBoard_SERIAL_NUMBER_LEN];
        struct{
            uint8_t Serial_Number      ; //serial number
        }DATE;
    }ExpansionBoardInput_SerialNumber;//序号修改 
    
    union 
    {  
        #define PG_TXExpansionBoardDATA_LEN 2
        uint8_t Datum[PG_TXExpansionBoardDATA_LEN];
        struct{
            uint16_t sensor0   :1;
            uint16_t sensor1   :1;
            uint16_t sensor2   :1;
            uint16_t sensor3   :1;
            uint16_t sensor4   :1;
            uint16_t sensor5   :1;
            uint16_t sensor6   :1;
            uint16_t sensor7   :1;
            uint16_t sensor8   :1;
            uint16_t sensor9   :1;
            uint16_t sensor10  :1;
            uint16_t sensor11  :1;
            uint16_t sensor12  :1;
            uint16_t sensor13  :1;
            uint16_t reserved  :2;
        }DATE;
    }ExpansionBoardInput_UploadData;//扩展版数据上传结构体
    
    SingleInputSignalDataDef_t SingleInputSignalData[14];
    
    struct ExpansionBoardInput_T* next;
} ExpansionBoardInputDef_t;
/* 宏定义 --------------------------------------------------------------------*/
#define SLAVE_ExpansionBoardInput_ENABLED 1
#define ExpansionBoardInput_Addr     6  //循迹板地址

#define ExpansionBoardInput_Sensor_1  0
#define ExpansionBoardInput_Sensor_2  1
#define ExpansionBoardInput_Sensor_3  2
#define ExpansionBoardInput_Sensor_4  3
#define ExpansionBoardInput_Sensor_5  4
#define ExpansionBoardInput_Sensor_6  5
#define ExpansionBoardInput_Sensor_7  6
#define ExpansionBoardInput_Sensor_8  7
#define ExpansionBoardInput_Sensor_9  8
#define ExpansionBoardInput_Sensor_10 9
#define ExpansionBoardInput_Sensor_11 10
#define ExpansionBoardInput_Sensor_12 11
#define ExpansionBoardInput_Sensor_13 12
#define ExpansionBoardInput_Sensor_14 13

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
uint8_t SLAVE_ExpansionBoardInput_Init(ExpansionBoardInputDef_t* handle,uint8_t list);
void SLAVE_ExpansionBoardInput_NumberSet(ExpansionBoardInputDef_t* handle,uint8_t number);
uint8_t SLAVE_ExpansionBoardInput_SensorRead(ExpansionBoardInputDef_t* handle,uint8_t list);
void SLAVE_ExpansionBoardInput_CANRXDispose(void *head,uint32_t Id);



#endif  // __SLAVE_ExpansionBoardInput_H__

