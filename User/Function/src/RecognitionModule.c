/* 包含头文件 ----------------------------------------------------------------*/
#include "RecognitionModule.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define ORDER_LEN     2
#define STARTQR_      0
#define STOPORDER_    1
#define STARTCOLOR_   2
#define STARTCIRCLE_  3
#define STARTLOC_     4
/* 私有变量 ------------------------------------------------------------------*/
static const uint8_t order[5][ORDER_LEN] = 
{
	{0x1b,0x31},//二维码识别
	{0x1b,0x30},//停止识别
	{0x1b,0x32},//颜色识别
	{0x1b,0x33},//圆识别
	{0x1b,0x34},//定位识别
};
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
void RecognitionModule_Init(RecognitionModule_s *hand,
							void (*pQ_Transmit)(uint8_t *pData, uint16_t Size))
{
	hand->err = 0;
	hand->time = 0;
	hand->RecognitionModuleSte = RM_leisure;
	hand->RecognitionModule_Transmit = pQ_Transmit;
}
void RecognitionModule_Scan1Ms(RecognitionModule_s *hand)
{
	if(hand->RecognitionModule_Transmit != 0)
	{
		switch(hand->RecognitionModuleSte)
		{
			case RM_QRstart:   //发送开始命令
				hand->RecognitionModule_Transmit((uint8_t*)order[STARTQR_],ORDER_LEN);
			    hand->time = 0;
			    hand->state = RM_QRstart;
			    hand->RecognitionModuleSte = RM_starting;
				break;
			case RM_Colorstart:   //发送开始命令
				hand->RecognitionModule_Transmit((uint8_t*)order[STARTCOLOR_],ORDER_LEN);
			    hand->time = 0;
			    hand->state = RM_Colorstart;
			    hand->RecognitionModuleSte = RM_starting;
				break;
			case RM_Circlestart:   //发送开始命令
				hand->RecognitionModule_Transmit((uint8_t*)order[STARTCIRCLE_],ORDER_LEN);
			    hand->time = 0;
			    hand->state = RM_Circlestart;
			    hand->RecognitionModuleSte = RM_starting;
				break;
			case RM_Locstart:   //发送开始命令
				hand->RecognitionModule_Transmit((uint8_t*)order[STARTLOC_],ORDER_LEN);
			    hand->time = 0;
			    hand->state = RM_Locstart;
			    hand->RecognitionModuleSte = RM_starting;
				break;
			case RM_starting:  //等待命令接收回应
				if(hand->time >= 50)
				{
					hand->err = ERR_disconnect;
					hand->RecognitionModule_Transmit((uint8_t*)order[STOPORDER_],ORDER_LEN);
					hand->RecognitionModuleSte = RM_error;
				}
				break;
		#if 0
			case RM_Identify:  //正在识别中
				if(hand->state == RM_QRstart)
				{//识别二维码中
					if(hand->time >= 3000)
					{//二维码识别超时
						hand->err = ERR_Identification_failure;
						hand->RecognitionModule_Transmit((uint8_t*)order[STOPORDER_],ORDER_LEN);
						hand->state = RM_leisure;
						hand->RecognitionModuleSte = RM_error;
					}
				}	
				break;
		#endif
			default:
				break;
		}
		hand->time++;
	}
}
void RecognitionModule_ReceivingProcess(RecognitionModule_s *hand,uint8_t *datahend,uint16_t datalen)
{
	if((1 == datalen)&&(RM_starting == hand->RecognitionModuleSte))
	{
		if(*datahend == 0x06)
		{
			hand->time = 0;
			hand->RecognitionModuleSte = RM_Identify;
		}
	}
	else if(datalen >= 2)
	{
		if(datahend[datalen-1] == 0x0d)
		{
			hand->RecognitionModuleSte = RM_succeed;
		}
	}
}
void RecognitionModule_Start(RecognitionModule_s *hand)
{
	if((hand->RecognitionModuleSte == RM_leisure)
	 ||(hand->RecognitionModuleSte == RM_error)
   	 ||(hand->RecognitionModuleSte == RM_succeed))
	{
		hand->RecognitionModuleSte = RM_QRstart;
	}
}
void ColorRecognitionModule_Start(RecognitionModule_s *hand)
{
	if((hand->RecognitionModuleSte == RM_leisure)
	 ||(hand->RecognitionModuleSte == RM_error)
   	 ||(hand->RecognitionModuleSte == RM_succeed))
	{
		hand->RecognitionModuleSte = RM_Colorstart;
	}
}
void RecognitionModule_Stop(RecognitionModule_s *hand)
{
	if(hand->RecognitionModuleSte != RM_leisure)
	{
		hand->RecognitionModule_Transmit((uint8_t*)order[STOPORDER_],ORDER_LEN);
		hand->RecognitionModuleSte = RM_leisure;
		hand->state = RM_leisure;
	}
}
