#ifndef __BSP_HAL_PPM_H__
#define __BSP_HAL_PPM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "mytype.h"
/* 宏定义 --------------------------------------------------------------------*/
#define READ_CH_H GPIO_PIN_SET
#define READ_CH_L GPIO_PIN_RESET
#define Read_CH() HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)

#define PPM_STICK_OFFSET     (1120u)     // 拨杆中间值
#define PPM_RESOLUTION       (420.0f)    // 遥控取值范围 做转化用
#define PPM_DEADBAND         (20)        // 遥控器死区，因为遥控器的拨杆在中位时不一定为0
/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
	__IO uint8_t  ppm_capture_sta;
	__IO uint16_t OutTime;
	__IO uint16_t CaptureValNew;
	__IO uint16_t CaptureValBuff[9];
	__IO uint16_t CaptureValSum;
	__IO uint16_t PPM_rx[9];
}_s_PPM_Data;
/* 扩展变量 ------------------------------------------------------------------*/
extern _s_PPM_Data PPM_DataStruct;
/* 函数声明 ------------------------------------------------------------------*/
void PPM_TimerInputCapture_Init(void);
uint8_t PPM_ReadData(int16_t *ch1,
				  int16_t *ch2,
				  int16_t *ch3,
				  int16_t *ch4,
				  uint8_t *sw);
void PPM_OutTime_1msScan(void);


#endif  // __BSP_HAL_PPM_H__

