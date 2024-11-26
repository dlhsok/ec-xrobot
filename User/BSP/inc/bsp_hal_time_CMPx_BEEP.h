#ifndef __BSP_LL_TIME_CMPx_BEEP_H__
#define __BSP_LL_TIME_CMPx_BEEP_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "mytype.h"

/* 类型定义 ------------------------------------------------------------------*/
/**************C调各音符频率对照表*****************/
// 定义低音音名（数值单位：Hz） 
#define L1 262
#define L2 294
#define L3 330
#define L4 349
#define L5 392
#define L6 440
#define L7 494
// 定义中音音名（数值单位：Hz） 
#define M1 523
#define M2 587
#define M3 659
#define M4 698
#define M5 784
#define M6 880
#define M7 988
// 定义高音音名（数值单位：Hz） 
#define H1 1047
#define H2 1175
#define H3 1319
#define H4 1397
#define H5 1568
#define H6 1760
#define H7 1976
/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void buzzerQuiet(void);
void buzzerSound(uint16_t usFreq);

#endif  // __BSP_LL_TIME_CMPx_BEEP_H__

