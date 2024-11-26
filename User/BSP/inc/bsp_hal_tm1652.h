#ifndef __BSP_HAL_TM1652_H__
#define __BSP_HAL_TM1652_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include "mytype.h"
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#define TM1652_CLK_H()        HAL_GPIO_WritePin(TM1652_CLK_GPIO_Port, TM1652_CLK_Pin, GPIO_PIN_SET)
#define TM1652_CLK_L()        HAL_GPIO_WritePin(TM1652_CLK_GPIO_Port, TM1652_CLK_Pin, GPIO_PIN_RESET)
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void bsp_tm1652_init(void);
void bsp_tm1652_SendData( uint8_t smg[], uint8_t len );
void bsp_tm1652_SendComm( void );
void bsp_tm1652_write(uint8_t dat);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  // __BSP_HAL_TM1652_H__

