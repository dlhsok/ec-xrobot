#ifndef __CAN_Communication_CANCONFIG_H__
#define __CAN_Communication_CANCONFIG_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include <stdint.h>
#include "string.h"
#include "mytype.h"
/* USER包含头文件 ------------------------------------------------------------*/
#include "SLAVE_Tracking/SLAVE_Tracking.h"
#include "SLAVE_DCMotorMiniwatt/SLAVE_DCMotorMiniwatt.h"
//#include "SLAVE_SteeringEngine/SLAVE_SteeringEngine.h"
//#include "SLAVE_SteppingMotor/SLAVE_SteppingMotor.h"
//#include "SLAVE_ExpansionBoardInput/SLAVE_ExpansionBoardInput.h"
//#include "SLAVE_ExpansionBoardOut/SLAVE_ExpansionBoardOut.h"
//#include "SLAVE_BLHDCMotorMiniwatt/SLAVE_BLHDCMotorMiniwatt.h"
//#include "SLAVE_BLDCMotorMiniwatt/SLAVE_BLDCMotorMiniwatt.h"
//#include "SLAVE_SteeringEngine_6CH/SLAVE_SteeringEngine_6CH.h"
//#include "SLAVE_RoboMasterC620/SLAVE_RoboMasterC620.h"
#include "SLAVE_UltrasonicRanging/SLAVE_UltrasonicRanging.h"
/* 宏定义 --------------------------------------------------------------------*/
#define NR_OF_TX_PG 80//帧个数
#define TX_PGN_USER 0 //帧起始地址

#define DEVICE_ID   1    //装置地址id
#define DEVICE_LIST 0  //装置地址序列

#define PG_TX_FREE     0      // buffer free, no transmission stands on 
#define PG_TX_REQ      1      // a TX PG is entered into buffer but controler isn't activated 
#define PG_TX_TxING    2      // stands on in the controler and trasmission is active 
#define PG_TX_SUC      3      // the TX interrupt occurs - message was successful sent 

#define TT_ASYNC    0       // 异步模式
#define TT_CYCLIC   1       // 循环模式 
//#define TT_PACKAGE  2       // 包模式

#define MODE_BRO 0  //广播模式
#define MODE_REQ 1  //请求模式
#define MODE_ACK 2  //答应模式


#define BAG_NOT     0
#define BAG_ING     1
#define BAG_OK      2
/* USER自定义 ----------------------------------------------------------------*/
#define PatrolPlate_Addr    3  //巡线板地址
#define SteeringPlate_Addr  4  //舵机控制板地址
#define SteppingPlate_Addr  5  //步进电机控制板地址
#define ALL_Addr            0xFF//发向全部设备(广播模式)
// Define the buffer for the PGs

// Application Data Length

// Application Data typedef


/* 类型定义 ------------------------------------------------------------------*/
/************************* variable and array define ***************************/
typedef struct
{ 
	uint8_t  ch;  //0为CAN1,1为CAN2
	uint8_t  IDE; //0为扩展帧，1为标准帧，默认为扩展帧
	uint8_t  TxDataLength     :4;  //number of databytes 
	uint8_t  RxDataLength     :4;  //number of databytes 
	uint16_t Enable           :1;  //Enable PG 
	uint16_t PGState          :2;  //Object used 
	uint16_t TransType        :2;  //TT_CYCLIC, TT_ASYNC,TT_PACKAGE
	uint16_t mode             :2;  //MODE_BRO,MODE_REQ,MODE_ACK
	uint16_t BagState         :2;  //BAG_NOT
	uint16_t LogBootFail_cnt  :4;//板子与板子之间的通讯错误
	uint16_t can_err          :3;     //板子自身发送错误	
	uint16_t TransRate;  //Trans. rate in ms, only for cyclic 
	uint16_t Timer;      //Timer in ms 
	uint32_t Source;     //PG source ID
	uint32_t Target;     //PG target ID    
	void *Datum;    //Pointer to specific data struct
	void *RxDatum;  
}PgnCanTxDataType;
/* 扩展变量 ------------------------------------------------------------------*/
extern PgnCanTxDataType TxPGArray[NR_OF_TX_PG];
/* 函数声明 ------------------------------------------------------------------*/
void InitialUsedPGs(void);
void GetReferenceFromDevice(void);
void GetReferenceToDevice(uint32_t id);





#endif  // __CAN_Communication_CANCONFIG_H__

