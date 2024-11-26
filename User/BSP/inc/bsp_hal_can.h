#ifndef __BSP_HAL_CAN_H__
#define __BSP_HAL_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include "mytype.h"
/* Private macros ------------------------------------------------------------*/
#ifndef  FALSE
    #define  FALSE    0
#endif
#ifndef  TRUE
    #define  TRUE     1
#endif
#define CAN_QUEUE_NUM 20
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint8_t  Channel;
	uint8_t  IDE;
	uint8_t  RTR;
	uint8_t  DLC;
	uint8_t  Data[8];
	uint32_t Id;
}CanRxMemberType; //can接收成员结构
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Bsp_Cancom_ConfigurationInit(void);
void Bsp_Cancom_FilterInit(void);
uint32_t Bsp_Cancom_ReadERR(void);
uint8_t CANSend(uint32_t Source,
	            uint32_t IDE,
                uint32_t RTR,
                uint8_t* Datum,
                uint8_t DataLength,
                uint8_t ch) ;
uint8_t CAN_OutQueue(CanRxMemberType * pVal);
void QueueOverflow_Dispose(CanRxMemberType *pQ);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  // __BSP_HAL_CAN_H__

