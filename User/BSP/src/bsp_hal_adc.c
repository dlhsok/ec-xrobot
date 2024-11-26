/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_hal_adc.h"
#include "adc.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
static VoidFuncVoid ADC1CallBackFunc = NULL;
uint32_t  AdcVal_mut = 0;
uint32_t  AdcVal_MEAN = 0;
__IO float  BatVoltage = 25.2f;
/* 扩展变量 ------------------------------------------------------------------*/
ADCBUF_TYPE ADC_ConvertedValue[ADCBUF_LEN];
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
void Bsp_ADC1_SetHandler(VoidFuncVoid func)
{
	ADC1CallBackFunc = func;
}
void Bsp_ADC_Init(void)
{
    uint8_t i;
	
    for(i=0;i<ADCBUF_LEN;i++)
    {
        ADC_ConvertedValue[i] = 0;
    }
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_ConvertedValue,sizeof(ADC_ConvertedValue)/4); 
}
float BatVoltage_Read(void)
{
	BatVoltage = AdcVal_MEAN * 8.1712f / 1000.0f;
	return BatVoltage;
}
/**
  * 函数功能: AD转换结束中断回调函数
  * 输入参数: hadc：AD设备类型句柄
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{  
	uint32_t ch;
    uint16_t i;
	
	ch = 0;
    for( i=0; i<ADC_NUM; i++ )
        ch += ADC_ConvertedValue[i];
    ch /= ADC_NUM;
	ch = (ch * 3300) / 4095;
	AdcVal_mut += ( ch + ( ( AdcVal_MEAN << 5 ) - AdcVal_MEAN) );
	AdcVal_MEAN = AdcVal_mut >> 5;
	AdcVal_mut = AdcVal_mut - ( AdcVal_MEAN << 5 );
	
    if(NULL != ADC1CallBackFunc)
    {
        ADC1CallBackFunc();
    }
}














