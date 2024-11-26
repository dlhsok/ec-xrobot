#ifndef __MYTYPE__
#define __MYTYPE__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#if defined(STM32F407xx)
    #include "stm32f4xx_hal.h"
#elif defined(STM32F103xB) || defined(STM32F103xE)
    #include "stm32f1xx_hal.h"
#elif defined(STM32F030x6)
    #include "stm32f0xx_hal.h"
#endif
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
/* 如果使用OS,则包括下面的头文件即可 */
/**
 * SYS_SUPPORT_OS用于定义系统文件夹是否支持OS
 * 0,不支持OS
 * 1,支持OS
 */
#ifndef SYS_SUPPORT_OS
#define SYS_SUPPORT_OS         1
#endif
#if SYS_SUPPORT_OS
#include <rtthread.h>
#define My_mDelay(x) HAL_Delay(x)
#endif
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#define ABS(x) ((x)>0? (x):(-(x)))
#ifndef  FALSE
	#define  FALSE    0
#endif
#ifndef  TRUE
	#define  TRUE     1
#endif

#define my_malloc(x)                    rt_malloc(x)
#define my_free(x)                      rt_free(x)
/* Exported types ------------------------------------------------------------*/
typedef uint8_t 	u8;
typedef uint16_t 	u16;
typedef uint32_t 	u32;

typedef int8_t 		s8;
typedef int16_t 	s16;
typedef int32_t		s32;

typedef volatile uint8_t 	vu8;
typedef volatile uint16_t 	vu16;
typedef volatile uint32_t 	vu32;

typedef volatile int8_t 	vs8;
typedef volatile int16_t 	vs16;
typedef volatile int32_t	vs32;

typedef void(*VoidFuncVoid)(void);
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
