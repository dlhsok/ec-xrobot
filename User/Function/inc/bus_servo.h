#ifndef _BUS_SERVO_H_
#define _BUS_SERVO_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
/* Private macros ------------------------------------------------------------*/
//SCS    SMSBL
#define SERVO_POTENTIOMETER      0 //高字节在前低字节在后
#define SERVO_MAGNETICENCODER    1 //低字节在前高字节在后

#define SERVO_UART_Tx_BuffSize   43

#define SERVO_FH1_DATA           0xFF
#define SERVO_FH2_DATA           0xFF

#define SERVO_ID_BRO             0xFE

#define SERVO_INS_PING           0x01  //（查询）查询工作状态
#define SERVO_INS_READ           0x02  //（读）查询控制表里的字符
#define SERVO_INS_WRITE          0x03  //（写）往控制表里写入字符
#define SERVO_INS_REGWRITE       0x04  //（异步写）类似于WRITE DATA，但是控制字符写入后并不马上动作，直到ACTION指令到达
#define SERVO_INS_ACTION         0x05  //（执行异步写）触发REG WRITE写入的动
#define SERVO_INS_SYCNREAD       0x82  //（同步读）用于同时查询多个舵机
#define SERVO_INS_SYCNWRITE      0x83  //（同步写）用于同时控制多个舵机
#define SERVO_INS_RESET          0x06  //（复位）把控制表复位为出厂值

#define SERVO_CTRLLIST_SETID     0x05  //保存ID号的地址
#define SERVO_CTRLLIST_TARLOC    0x2A  //目标位置的首地址
/* Private types -------------------------------------------------------------*/
typedef union
{
	uint8_t Datum[SERVO_UART_Tx_BuffSize];
	struct{
		uint8_t FH1;         //帧头1
		uint8_t FH2;         //帧头2
		uint8_t id;          //ID
		uint8_t length;      //数据长度
		uint8_t instruction; //指令
		uint8_t data[SERVO_UART_Tx_BuffSize-5];
	}DATE;
}BusServo_TypeDef;
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
