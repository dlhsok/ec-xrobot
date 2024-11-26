/* Includes ------------------------------------------------------------------*/
#include "thread_comm_test.h"
#include "bsp.h"
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* �����߳̿��ƿ�ָ�� */
rt_thread_t thread_comm = RT_NULL;
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/***********************************************************************************************

                                       Ӧ���¼����жϣ�����

************************************************************************************************/
/**
  * @brief  ���ڽ�������¼�
  * @param  None
  * @retval None
  */
void XferExternalUart2Rx_Handler(char *pdata, uint16_t len)
{
	pdata[len] = 'u';
	pdata[len+1] = 'a';
	pdata[len+2] = 'r';
	pdata[len+3] = 't';
	pdata[len+4] = '2';
	pdata[len+5] = '\r';
	pdata[len+6] = '\n';
	Bsp_UARTMixed_TxTrigger( &muart2, pdata, len+7);
}
void XferExternalUart3Rx_Handler(char *pdata, uint16_t len)
{//������ɻص�����ָ��
	pdata[len] = 'u';
	pdata[len+1] = 'a';
	pdata[len+2] = 'r';
	pdata[len+3] = 't';
	pdata[len+4] = '3';
	pdata[len+5] = '\r';
	pdata[len+6] = '\n';
//	Bsp_UARTMixed_TxTrigger( &muart3, pdata, len+7);
}
void XferExternalUart4Rx_Handler(char *pdata, uint16_t len)
{
	pdata[len] = 'u';
	pdata[len+1] = 'a';
	pdata[len+2] = 'r';
	pdata[len+3] = 't';
	pdata[len+4] = '4';
	pdata[len+5] = '\r';
	pdata[len+6] = '\n';
	Bsp_UARTMixed_TxTrigger( &muart4, pdata, len+7);
}
void XferExternalUart5Rx_Handler(char *pdata, uint16_t len)
{
	pdata[len] = 'u';
	pdata[len+1] = 'a';
	pdata[len+2] = 'r';
	pdata[len+3] = 't';
	pdata[len+4] = '5';
	pdata[len+5] = '\r';
	pdata[len+6] = '\n';
	Bsp_UARTMixed_TxTrigger( &muart5, pdata, len+7);
}
/***********************************************************************************************

                                       Ӧ��������

************************************************************************************************/

void comm_task(void *pvParameters)
{
	CanRxMemberType CanRxVal;
	while(1)
    {
		if( CAN_OutQueue(&CanRxVal) )
		{
			CANSend(CanRxVal.Id,CanRxVal.IDE,CanRxVal.RTR,CanRxVal.Data,CanRxVal.DLC,CanRxVal.Channel);
		}
		My_mDelay( 1 );
    }
}
/***********************************************************************************************

                                       Ӧ��������

************************************************************************************************/
int Task_COMM_create(void)
{
	Bsp_Cancom_FilterInit();
	Bsp_UartMixed_Init( &muart2, XferExternalUart2Rx_Handler, 0 );
//	Bsp_UartMixed_Init( &muart3, XferExternalUart3Rx_Handler, 0 );
	Bsp_UartMixed_Init( &muart4, XferExternalUart4Rx_Handler, 0 );
	Bsp_UartMixed_Init( &muart5, XferExternalUart5Rx_Handler, 0 );
    thread_comm = rt_thread_create("comm",           /* �߳����� */
								   comm_task,        /* �߳���ں��� */
								   RT_NULL,          /* �߳���ں������� */
								   512,              /* �߳�ջ��С */
								   1,                /* �̵߳����ȼ� */
								   20);              /* �߳�ʱ��Ƭ */
	if(thread_comm != RT_NULL)
	{
		rt_thread_startup(thread_comm);
		rt_kprintf("thread_comm startup!\n");
	}
	
	return 0;
}
INIT_APP_EXPORT(Task_COMM_create);



















