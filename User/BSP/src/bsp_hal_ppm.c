/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_hal_ppm.h"
#include "tim.h"
/* 私有宏定义 ----------------------------------------------------------------*/
#define  PPM_TIMER  htim1
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
_s_PPM_Data PPM_DataStruct = {0};
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/***************输入捕获初始化************************/
void PPM_TimerInputCapture_Init(void)
{
	TIM_RESET_CAPTUREPOLARITY(&PPM_TIMER,TIM_CHANNEL_2);   //一定要先清除原来的设置！！
	TIM_SET_CAPTUREPOLARITY(&PPM_TIMER,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//配置TIM通道上升沿捕获
    HAL_TIM_IC_Start_IT(&PPM_TIMER,TIM_CHANNEL_2);
}
uint8_t PPM_ReadData(int16_t *ch1,
				  int16_t *ch2,
				  int16_t *ch3,
				  int16_t *ch4,
				  uint8_t *sw) 
{
	int16_t ppm_rx[7] = {0,0,0,0,0,0,0};
	uint8_t i;
	PPM_OutTime_1msScan();
	if(PPM_DataStruct.PPM_rx[3] > 600)
	{
		for(i=0; i<7; i++)
		{
			if(i == 2)
			    ppm_rx[i] = (int16_t)PPM_STICK_OFFSET - PPM_DataStruct.PPM_rx[i+1];
			else 
				ppm_rx[i] = PPM_DataStruct.PPM_rx[i+1] - (int16_t)PPM_STICK_OFFSET;
			/* prevent remote control zero deviation */
			if(ABS(ppm_rx[i]) < PPM_DEADBAND)
				ppm_rx[i] = 0;
		}
		
		if(ppm_rx[4] > 0)*sw |= 0x04;
		else*sw &= 0xfb;
		if(ppm_rx[4] < 0)*sw |= 0x08;
		else*sw &= 0xf7;
		
		if(ppm_rx[6] > 100) *sw |= 0x01;
		else *sw &= 0xfe;
		if(ppm_rx[6] < -100) *sw |= 0x02;
		else *sw &= 0xfd;
	}
	
	if(PPM_DataStruct.PPM_rx[3] > 600)
	{
		*ch1 = ppm_rx[0];
		*ch2 = ppm_rx[1];
		*ch3 = ppm_rx[2];
		*ch4 = ppm_rx[3];
		return 0;
	}
	else
	{
		return 1;
	}
}
void PPM_OutTime_1msScan(void)
{
	if(PPM_DataStruct.OutTime < 1000)
	{
		PPM_DataStruct.OutTime++;
		if(PPM_DataStruct.OutTime >= 500)
		{
			PPM_DataStruct.PPM_rx[3] = 480;
		}
	}
}
/***********************************************************************************************

                                       应用事件（中断）函数

************************************************************************************************/
/***********TIM输入捕获中断****************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(PPM_TIMER.Instance == htim->Instance)
	{
		if(HAL_TIM_ACTIVE_CHANNEL_2 == htim->Channel)
		{
			PPM_DataStruct.CaptureValNew = htim->Instance->CNT;
			if(READ_CH_H == Read_CH())
			{//上升沿
				if((PPM_DataStruct.ppm_capture_sta&0x40) == 0)
				{
					htim->Instance->CNT = 0;
					PPM_DataStruct.ppm_capture_sta |= 0x40;//标记捕获到了上升沿
				}
				TIM_RESET_CAPTUREPOLARITY(&PPM_TIMER,TIM_CHANNEL_2);   //一定要先清除原来的设置！！
				TIM_SET_CAPTUREPOLARITY(&PPM_TIMER,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);//定时器通道设置为下降沿捕获
			}
			else
			{//下降沿
				if(PPM_DataStruct.ppm_capture_sta & 0x40)
				{
					PPM_DataStruct.ppm_capture_sta |= 0X80;		//标记成功捕获到一次高电平脉宽
					htim->Instance->CNT = 0;
				}
				TIM_RESET_CAPTUREPOLARITY(&PPM_TIMER,TIM_CHANNEL_2);   //一定要先清除原来的设置！！
				TIM_SET_CAPTUREPOLARITY(&PPM_TIMER,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//配置TIM通道上升沿捕获
			}
			
			if(PPM_DataStruct.ppm_capture_sta&0X80)
			{
				PPM_DataStruct.ppm_capture_sta &= 0x3f;
				if((PPM_DataStruct.CaptureValNew > 2000) && (PPM_DataStruct.ppm_capture_sta != 0))
				{
					PPM_DataStruct.ppm_capture_sta = 0;
				}
				PPM_DataStruct.CaptureValBuff[PPM_DataStruct.ppm_capture_sta] = PPM_DataStruct.CaptureValNew;
				PPM_DataStruct.ppm_capture_sta++;
				if(PPM_DataStruct.ppm_capture_sta == 9)
				{
					PPM_DataStruct.CaptureValSum = 0;
					for(uint8_t i = 0; i<9; i++)
					{
						PPM_DataStruct.CaptureValSum += PPM_DataStruct.CaptureValBuff[i];
					}
					PPM_DataStruct.ppm_capture_sta = 0;
					
					if(ABS((int16_t)(PPM_DataStruct.CaptureValSum - 16300)) < 1000)
					{
						for(uint8_t i = 0; i<9; i++)
						{
							PPM_DataStruct.PPM_rx[i] = PPM_DataStruct.CaptureValBuff[i];
						}
						PPM_DataStruct.OutTime = 0;
					}
				}
			}
		} 
	}
}






