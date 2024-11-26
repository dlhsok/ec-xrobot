#ifndef __BSP__
#define __BSP__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include "mytype.h"
#include "bsp_hal_gpio.h"
#include "bsp_hal_uartdma.h"
#include "bsp_hal_time_CMPx_BEEP.h"
#include "CAN_Communication.h"
#include "bsp_hal_st7735s.h"
#include "bsp_hal_adc.h"
#include "bsp_hal_tm1652.h"
#include "bsp_hal_flash.h"
#ifdef DEBUG
	#include <stdio.h>
#endif
/* Private macros ------------------------------------------------------------*/
#ifdef DEBUG
    #define PRINTF printf
#else
    #define PRINTF(format, ...)
#endif


#if defined(__ARMCC_VERSION)
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#else
extern int __bss_end;
#define HEAP_BEGIN      ((void *)&__bss_end)
#endif

#define STM32_SRAM_SIZE      (128)
#define STM32_SRAM_END       (0x20000000 + STM32_SRAM_SIZE * 1024)
#define HEAP_END             STM32_SRAM_END
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
uint8_t Tim_CheckTimeOut(uint32_t TimeStart, uint32_t TimeNow, uint32_t TimeOut);
void HAL_usDelay(uint32_t nus);
void HAL_xmsDelay(uint32_t nms);






#ifdef __cplusplus
}
#endif /* __cplusplus */



#endif
