/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "VOFA_Protocol.h"
#include "bsp_hal_uart3dma232AndTTL.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/
#define VOFA_UART_Transmit(pData, Size) Bsp_UART3_Transmit(pData, Size)
/* ˽�б��� ------------------------------------------------------------------*/
uint8_t tempData[256];                    //����Ĵ���Buffer
/* ��չ���� ------------------------------------------------------------------*/
//DEBUG
float tempFloat[8];
/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/
void VOFA_Write(float *qtempFloat,uint16_t len)
{
	if(len > 31) len = 31;//�����Է���31������
	
	len *= 4;
	memcpy(tempData, (uint8_t *)qtempFloat, len);//ͨ��������������������
	
	tempData[len] = 0x00;                    //д���β����
	tempData[len+1] = 0x00;
	tempData[len+2] = 0x80;
	tempData[len+3] = 0x7f;
	
	VOFA_UART_Transmit(tempData,len+4);
}














