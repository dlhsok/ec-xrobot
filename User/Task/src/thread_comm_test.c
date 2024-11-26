/* Includes ------------------------------------------------------------------*/
#include "thread_comm_test.h"
#include "bsp.h"
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 定义线程控制块指针 */
rt_thread_t thread_comm = RT_NULL;
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/***********************************************************************************************

                                       应用事件（中断）函数

************************************************************************************************/
/**
  * @brief  串口接收完成事件
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
{//接收完成回调函数指针
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

                                       应用任务函数

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

                                       应用主函数

************************************************************************************************/
int Task_COMM_create(void)
{
	Bsp_Cancom_FilterInit();
	Bsp_UartMixed_Init( &muart2, XferExternalUart2Rx_Handler, 0 );
//	Bsp_UartMixed_Init( &muart3, XferExternalUart3Rx_Handler, 0 );
	Bsp_UartMixed_Init( &muart4, XferExternalUart4Rx_Handler, 0 );
	Bsp_UartMixed_Init( &muart5, XferExternalUart5Rx_Handler, 0 );
    thread_comm = rt_thread_create("comm",           /* 线程名字 */
								   comm_task,        /* 线程入口函数 */
								   RT_NULL,          /* 线程入口函数参数 */
								   512,              /* 线程栈大小 */
								   1,                /* 线程的优先级 */
								   20);              /* 线程时间片 */
	if(thread_comm != RT_NULL)
	{
		rt_thread_startup(thread_comm);
		rt_kprintf("thread_comm startup!\n");
	}
	
	return 0;
}
INIT_APP_EXPORT(Task_COMM_create);



















