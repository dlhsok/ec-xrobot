#ifndef __BSP_HAL_ADC_H__
#define __BSP_HAL_ADC_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "mytype.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/
#define ADCBUF_TYPE uint32_t
#define ADC_NUM 10
#define ADCBUF_LEN  8*ADC_NUM
/* 扩展变量 ------------------------------------------------------------------*/
extern ADCBUF_TYPE ADC_ConvertedValue[ADCBUF_LEN];
/* 函数声明 ------------------------------------------------------------------*/
void Bsp_ADC1_SetHandler(VoidFuncVoid func);
void Bsp_ADC_Init(void);
float BatVoltage_Read(void);





#endif  // __BSP_HAL_ADC_H__

