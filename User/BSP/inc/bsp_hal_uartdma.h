#ifndef __BSP_HAL_UARTDMA_H__
#define __BSP_HAL_UARTDMA_H__
#ifdef __cplusplus
extern "C" {
#endif
/* 包含头文件 ----------------------------------------------------------------*/
//Application_User_Core
#include "usart.h"
//BSP
#include "mytype.h"
/* 宏定义 --------------------------------------------------------------------*/
//#define Enabled_MUart1
#define Enabled_MUart2
//#define Enabled_MUart3
#define Enabled_MUart4
#define Enabled_MUart5
//#define Enabled_MUart6
//发送模式
#define	MUART_TX_MODE_POPUP			0	//弹出模式: 遇到串口非就绪->直接放弃发送
#define	MUART_TX_MODE_BLOCKING	    1	//阻塞模式: 遇到串口非就绪->等待串口就绪
#define MUART_MODE	 		        MUART_TX_MODE_POPUP
//收发缓冲区
#define UART_Rx_BuffSize	40		//收发缓冲区容量
#define UART_Tx_BuffSize	64		//收发缓冲区容量
/* 类型定义 ------------------------------------------------------------------*/
/** 
  * UART混合功能结构体定义
**/
typedef struct
{
	//缓冲区
	char		buff_tx[UART_Tx_BuffSize];				//发送缓冲区
	char		buff_rx[UART_Rx_BuffSize];				//接收缓冲区
	char		buff_rx_transfer[UART_Rx_BuffSize];	    //接收中转缓冲区
	//统计
	uint16_t 	len_tx;				//收发数据量
	uint16_t	len_rx;
	uint32_t	count_tx;			//收发字节数量统计
	uint32_t	count_rx;
	//实例
	UART_HandleTypeDef	*puart;		//串口实例(HAL库定义的结构体)
	DMA_HandleTypeDef	*pdma_tx;	//DMA发送通道实例(HAL库定义的结构体)
	DMA_HandleTypeDef	*pdma_rx;	//DMA接收通道实例(HAL库定义的结构体)
	//回调函数指针
	void		(*Xfer_External_RxCallback)(char *data, uint16_t len);	//接收完成回调函数指针
	void		(*Xfer_External_TxCallback)(void);						//发送完成回调函数指针
}UartMixed_TypeDef;
/* 扩展变量 ------------------------------------------------------------------*/
//声明引用
#ifdef Enabled_MUart1
extern UartMixed_TypeDef	muart1;
#endif
#ifdef Enabled_MUart2
extern UartMixed_TypeDef	muart2;
#endif
#ifdef Enabled_MUart3
extern UartMixed_TypeDef	muart3;
#endif
#ifdef Enabled_MUart4
extern UartMixed_TypeDef	muart4;
#endif
#ifdef Enabled_MUart5
extern UartMixed_TypeDef	muart5;
#endif
#ifdef Enabled_MUart6
extern UartMixed_TypeDef	muart6;
#endif 
/* 函数声明 ------------------------------------------------------------------*/
void Bsp_UartMixed_Init( UartMixed_TypeDef *muart, void(*xerc)(char*, uint16_t), void(*xetc)(void) );
void Bsp_UartMixed_Begin(UartMixed_TypeDef *muart);
void Bsp_UartMixed_Stop(UartMixed_TypeDef *muart);
HAL_StatusTypeDef Bsp_UARTMixed_TxTrigger(UartMixed_TypeDef *muart, char *pData, uint16_t len);




#ifdef __cplusplus
}
#endif

#endif  // __BSP_HAL_UARTDMA_H__

