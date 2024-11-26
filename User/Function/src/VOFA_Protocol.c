/* 包含头文件 ----------------------------------------------------------------*/
#include "VOFA_Protocol.h"
#include "bsp_hal_uart3dma232AndTTL.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define VOFA_UART_Transmit(pData, Size) Bsp_UART3_Transmit(pData, Size)
/* 私有变量 ------------------------------------------------------------------*/
uint8_t tempData[256];                    //定义的传输Buffer
/* 扩展变量 ------------------------------------------------------------------*/
//DEBUG
float tempFloat[8];
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void VOFA_Write(float *qtempFloat,uint16_t len)
{
	if(len > 31) len = 31;//最大可以发送31个数据
	
	len *= 4;
	memcpy(tempData, (uint8_t *)qtempFloat, len);//通过拷贝把数据重新整理
	
	tempData[len] = 0x00;                    //写如结尾数据
	tempData[len+1] = 0x00;
	tempData[len+2] = 0x80;
	tempData[len+3] = 0x7f;
	
	VOFA_UART_Transmit(tempData,len+4);
}















