/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "RecognitionModule.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define ORDER_LEN     2
#define STARTQR_      0
#define STOPORDER_    1
#define STARTCOLOR_   2
#define STARTCIRCLE_  3
#define STARTLOC_     4
/* ˽�б��� ------------------------------------------------------------------*/
static const uint8_t order[5][ORDER_LEN] = 
{
	{0x1b,0x31},//��ά��ʶ��
	{0x1b,0x30},//ֹͣʶ��
	{0x1b,0x32},//��ɫʶ��
	{0x1b,0x33},//Բʶ��
	{0x1b,0x34},//��λʶ��
};
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
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
			case RM_QRstart:   //���Ϳ�ʼ����
				hand->RecognitionModule_Transmit((uint8_t*)order[STARTQR_],ORDER_LEN);
			    hand->time = 0;
			    hand->state = RM_QRstart;
			    hand->RecognitionModuleSte = RM_starting;
				break;
			case RM_Colorstart:   //���Ϳ�ʼ����
				hand->RecognitionModule_Transmit((uint8_t*)order[STARTCOLOR_],ORDER_LEN);
			    hand->time = 0;
			    hand->state = RM_Colorstart;
			    hand->RecognitionModuleSte = RM_starting;
				break;
			case RM_Circlestart:   //���Ϳ�ʼ����
				hand->RecognitionModule_Transmit((uint8_t*)order[STARTCIRCLE_],ORDER_LEN);
			    hand->time = 0;
			    hand->state = RM_Circlestart;
			    hand->RecognitionModuleSte = RM_starting;
				break;
			case RM_Locstart:   //���Ϳ�ʼ����
				hand->RecognitionModule_Transmit((uint8_t*)order[STARTLOC_],ORDER_LEN);
			    hand->time = 0;
			    hand->state = RM_Locstart;
			    hand->RecognitionModuleSte = RM_starting;
				break;
			case RM_starting:  //�ȴ�������ջ�Ӧ
				if(hand->time >= 50)
				{
					hand->err = ERR_disconnect;
					hand->RecognitionModule_Transmit((uint8_t*)order[STOPORDER_],ORDER_LEN);
					hand->RecognitionModuleSte = RM_error;
				}
				break;
		#if 0
			case RM_Identify:  //����ʶ����
				if(hand->state == RM_QRstart)
				{//ʶ���ά����
					if(hand->time >= 3000)
					{//��ά��ʶ��ʱ
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
