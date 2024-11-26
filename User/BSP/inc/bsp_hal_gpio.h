#ifndef __BSP_HAL_GPIO_H__
#define __BSP_HAL_GPIO_H__

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
#define LED_ON         GPIO_PIN_RESET
#define LED_OFF        GPIO_PIN_SET

#ifdef LED1_GPIO_Port
#define LED1(x)        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, x)
#define LED1_Toggle()  HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin)
#endif

#ifdef LED2_GPIO_Port
#define LED2(x)        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, x)
#define LED2_Toggle()  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin)
#endif

#ifdef LED3_GPIO_Port
#define LED3(x)        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, x)
#define LED3_Toggle()  HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin)
#endif
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#ifdef LED1_GPIO_Port
void led1_level(uint8_t level);
#endif
#ifdef LED2_GPIO_Port
void led2_level(uint8_t level);
#endif
#ifdef LED3_GPIO_Port
void led3_level(uint8_t level);
#endif
#ifdef KEY1_GPIO_Port
uint8_t KEY_1(void);
#endif
#ifdef KEY2_GPIO_Port
uint8_t KEY_2(void);
#endif
#ifdef KEY3_GPIO_Port
uint8_t KEY_3(void);
#endif
#ifdef KEY4_GPIO_Port
uint8_t KEY_4(void);
#endif
#ifdef KEY5_GPIO_Port
uint8_t KEY_5(void);
#endif
#ifdef KEY6_GPIO_Port
uint8_t KEY_6(void);
#endif
#ifdef KEY7_GPIO_Port
uint8_t KEY_7(void);
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  // __BSP_HAL_GPIO_H__

