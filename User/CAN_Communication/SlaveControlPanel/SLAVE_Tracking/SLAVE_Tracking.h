#ifndef __SLAVE_Tracking_H__
#define __SLAVE_Tracking_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct Tracking_T
{    
    uint8_t  CAN_Number;
	uint8_t  AdcModeEnable;
    union 
    {
        #define PG_SETPATROLPLATEADCDATA_LEN 2
        uint8_t Datum[PG_SETPATROLPLATEADCDATA_LEN];
        struct{
            uint8_t  mode;
            uint8_t  list;
        }DATE;
    }Tracking_ModeAndSerialNumber;//模式和序号修改 
    
    union 
    {  
        #define PG_ACKPATROLPLATEADCDATA_LEN 8
        uint8_t Datum[PG_ACKPATROLPLATEADCDATA_LEN];
        struct{
            uint8_t  adc_val[PG_ACKPATROLPLATEADCDATA_LEN];
        }DATE;
    }Tracking_UploadADCData;//巡线板ADC数据响应结构体
    
    union 
    {  
        #define PG_TXPATROLPLATEDATA_LEN 4
        uint8_t Datum[PG_TXPATROLPLATEDATA_LEN];
        struct{
            uint8_t  SignalData;
            uint8_t  mode;
            uint16_t Fault;
        }DATE;
    }Tracking_UploadData;//巡线板数据上传结构体

    struct Tracking_T* next;
} TrackingDef_t;
/* 宏定义 --------------------------------------------------------------------*/
#define SLAVE_Tracking_ENABLED 1
#define TrackingPlate_Addr     3  //循迹板地址

#define TRACKING_AdcModeEnable_FALSE 0
#define TRACKING_AdcModeEnable_TRUE  1

#define TRACKING_MODE_WORK  0  //工作模式
#define TRACKING_MODE_INIT  1  //系数设置初始化
#define TRACKING_MODE_CALC  2  //系数设置读取计算
#define TRACKING_MODE_FLASH 3  //进行FLASH储存
#define TRACKING_MODE_IDSET 4  //ID号设置模式

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
uint8_t SLAVE_Tracking_Init(TrackingDef_t* handle,uint8_t list,uint8_t signalinit);  //初始循迹条数据
uint8_t SLAVE_Tracking_BoolRead(TrackingDef_t* handle);  //读取循迹条布尔量信号
uint8_t SLAVE_Tracking_ModeRead(TrackingDef_t* handle);  //读取循迹条工作模式
void SLAVE_Tracking_NumberSet(TrackingDef_t* handle,uint8_t number);//修改循迹条ID号（目前修改后不能直接连接）
uint8_t SLAVE_Tracking_AdcModeEnable(TrackingDef_t* handle,uint8_t **data_head);//开启循迹条AD数值反馈模式
void SLAVE_Tracking_AdcModeDisable(TrackingDef_t* handle);//关闭循迹条AD数值反馈模式
void SLAVE_Tracking_CheckModeEnable(TrackingDef_t* handle);



#endif  // __SLAVE_Tracking_H__

