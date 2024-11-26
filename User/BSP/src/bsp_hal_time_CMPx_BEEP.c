/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_hal_time_CMPx_BEEP.h"
#include "tim.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define PWM_TIMEX        htim12
#define PWM_CHANNEL      TIM_CHANNEL_1
#define BEEP_Pin         GPIO_PIN_14
#define BEEP_GPIO_Port   GPIOB
/* 私有变量 ------------------------------------------------------------------*/


/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
// 蜂鸣器停止发声
void buzzerQuiet(void)
{
	HAL_TIM_PWM_Stop(&PWM_TIMEX, PWM_CHANNEL);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = BEEP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
}

//蜂鸣器发出指定频率的声音
//usFreq是发声频率，取值 (系统时钟/65536)+1 ～ 20000，单位：Hz
void buzzerSound(uint16_t usFreq)	 
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	unsigned long  ulVal;

	if((usFreq<=8000000/65536UL)||(usFreq>20000))
	{
		buzzerQuiet();// 蜂鸣器静音
	}
	else
	{
		GPIO_InitStruct.Pin = BEEP_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
        HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);

		ulVal=14000000/usFreq;
		__HAL_TIM_SET_AUTORELOAD(&PWM_TIMEX,ulVal);//设置自动重装载寄存器周期的值（音调）
		__HAL_TIM_SET_COMPARE(&PWM_TIMEX, PWM_CHANNEL, ulVal/2);//音量
		HAL_TIM_PWM_Start(&PWM_TIMEX,PWM_CHANNEL);
	}  
}













