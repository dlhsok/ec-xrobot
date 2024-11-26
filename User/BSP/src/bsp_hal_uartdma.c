/* 包含头文件 ----------------------------------------------------------------*/
//Oneself
#include "bsp_hal_uartdma.h"
//C基本库
#include <stdbool.h>	//Bool数据类型
/* 私有宏定义 ----------------------------------------------------------------*/
/**
  * @brief  混合串口实例定义
*/
#ifdef Enabled_MUart1
	UartMixed_TypeDef	muart1;
#endif
#ifdef Enabled_MUart2
	UartMixed_TypeDef	muart2;
#endif
#ifdef Enabled_MUart3
	UartMixed_TypeDef	muart3;
#endif
#ifdef Enabled_MUart4
	UartMixed_TypeDef	muart4;
	#define UART4_DIR_TX() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET)
    #define UART4_DIR_RX() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET)
#endif
#ifdef Enabled_MUart5
	UartMixed_TypeDef	muart5;
#endif
#ifdef Enabled_MUart6
	UartMixed_TypeDef	muart6;
#endif
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
 
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*
if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
{//检测串口是否产生了空闲中断
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    //调用串口接收函数(在uart_mixed中实例化)
    HAL_UART_RxCpltCallback(&huart2);
}
*/
/**
  * @brief  总线方向控制
  * @call   <- HAL_UART_TxCpltCallback(HAL)
  * @call   <- UART_Mixed_TxTrigger(Mixed)
  * @param  muart: 混合串口指针
  * @param  dir: 方向 (true:发送 false:接收)
  * @retval NULL 
**/
void Bsp_UARTDir_Control(UartMixed_TypeDef *muart, bool dir)
{
#if defined(Enabled_MUart1) && defined(UART1_DIR_TX) && defined(UART1_DIR_RX)
	if(muart == &muart1){
		if(dir)		UART1_DIR_TX();
		if(!dir)	UART1_DIR_RX();
	}
#endif
#if defined(Enabled_MUart2) && defined(UART2_DIR_TX) && defined(UART2_DIR_RX)
	if(muart == &muart2){
		if(dir)		UART2_DIR_TX();
		if(!dir)	UART2_DIR_RX();
	}
#endif
#if defined(Enabled_MUart3) && defined(UART3_DIR_TX) && defined(UART3_DIR_RX)
	if(muart == &muart3){
		if(dir)		UART3_DIR_TX();
		if(!dir)	UART3_DIR_RX();
	}
#endif
#if defined(Enabled_MUart4) && defined(UART4_DIR_TX) && defined(UART4_DIR_RX)
	if(muart == &muart4){
		if(dir)		UART4_DIR_TX();
		if(!dir)	UART4_DIR_RX();
	}
#endif
#if defined(Enabled_MUart5) && defined(UART5_DIR_TX) && defined(UART5_DIR_RX)
	if(muart == &muart5){
		if(dir)		UART5_DIR_TX();
		if(!dir)	UART5_DIR_RX();
	}
#endif
#if defined(Enabled_MUart6) && defined(UART6_DIR_TX) && defined(UART6_DIR_RX)
	if(muart == &muart6){
		if(dir)		UART6_DIR_TX();
		if(!dir)	UART6_DIR_RX();
	}
#endif
}
/*********************************************************************/
/****************************    初始化   ****************************/
/*********************************************************************/
/**
  * @brief  混合串口使能初始化
  * @param  muart: 混合串口指针
  * @param  xerc:  接收完成回调函数
  * @param  xetc:  发送完成回调函数
  * @retval NULL 
**/
void Bsp_UartMixed_Init( UartMixed_TypeDef *muart, void(*xerc)(char*, uint16_t), void(*xetc)(void) )
{				     
	memset(muart, 0, sizeof(UartMixed_TypeDef));
	//实例
#ifdef Enabled_MUart1
	if(muart == &muart1){
    	muart1.puart = &huart1;		
	    muart1.pdma_tx = &hdma_usart1_tx;	
	    muart1.pdma_rx = &hdma_usart1_rx;	
	}
#endif
#ifdef Enabled_MUart2
	if(muart == &muart2){
	    muart2.puart = &huart2;		
	    muart2.pdma_tx = &hdma_usart2_tx;	
	    muart2.pdma_rx = &hdma_usart2_rx;	
	}
#endif		
#ifdef Enabled_MUart3
	if(muart == &muart3){
	    muart3.puart = &huart3;		
	    muart3.pdma_tx = &hdma_usart3_tx;	
	    muart3.pdma_rx = &hdma_usart3_rx;	
	}
#endif		
#ifdef Enabled_MUart4
	if(muart == &muart4){
	    muart4.puart = &huart4;		
	    muart4.pdma_tx = &hdma_uart4_tx;
	    muart4.pdma_rx = &hdma_uart4_rx;	
	}
#endif		
#ifdef Enabled_MUart5
	if(muart == &muart5){	
	    muart5.puart = &huart5;		
	    muart5.pdma_tx = &hdma_uart5_tx;	
	    muart5.pdma_rx = &hdma_uart5_rx;	
	}
#endif	
#ifdef Enabled_MUart6
	if(muart == &muart6){	
	    muart6.puart = &huart6;		
	    muart6.pdma_tx = &hdma_usart6_tx;	
	    muart6.pdma_rx = &hdma_usart6_rx;	
	}
#endif
	//接收回调函数指针
	muart->Xfer_External_RxCallback = xerc;
	muart->Xfer_External_TxCallback = xetc;

	//开始传输
	Bsp_UartMixed_Begin(muart);
}
/*********************************************************************/
/***************************   混合库功能函数   **********************/
/*********************************************************************/
/**
  * @brief  混合串口开始
  * @param  muart: 混合串口指针
  * @retval NULL 
**/
void Bsp_UartMixed_Begin(UartMixed_TypeDef *muart)
{
	if(muart->Xfer_External_RxCallback != NULL)
	{
		if(HAL_UART_Receive_DMA(muart->puart, (uint8_t*)muart->buff_rx, UART_Rx_BuffSize) != HAL_OK)	//开启DMA接收
			Error_Handler();
		__HAL_UART_ENABLE_IT(muart->puart, UART_IT_IDLE);	//使能UART空闲中断
	}
	Bsp_UARTDir_Control(muart, false);
}
/**
  * @brief  混合串口停止
  * @param  muart: 混合串口指针
  * @retval NULL 
**/
void Bsp_UartMixed_Stop(UartMixed_TypeDef *muart)
{
	__HAL_UART_DISABLE_IT(muart->puart, UART_IT_IDLE);	//禁用UART空闲中断
	HAL_UART_DMAStop(muart->puart);						//停止DMA传输
	//HAL_UART_Abort(muart->puart);						//中止正在进行的传输
}
/**
  * @brief  由UART_HandleTypeDef(HAL)实例寻找对应的UartMixed_TypeDef(Mixed)实例
  * @call   <- HAL_UART_RxCpltCallback(HAL)
  * @call   <- HAL_UART_TxCpltCallback(HAL)
  * @param  huart: 串口指针
  * @retval 混合串口实例
**/
static UartMixed_TypeDef *Seek_UART_Mixed(UART_HandleTypeDef *huart)
{
	//寻找混合结构
	if(0)
		return NULL;
#ifdef Enabled_MUart1
	else if(muart1.puart == huart)
		return &muart1;
#endif
#ifdef Enabled_MUart2
	else if(muart2.puart == huart)
		return &muart2;
#endif
#ifdef Enabled_MUart3
	else if(muart3.puart == huart)
		return &muart3;
#endif
#ifdef Enabled_MUart4
	else if(muart4.puart == huart)
		return &muart4;
#endif
#ifdef Enabled_MUart5
	else if(muart5.puart == huart)
		return &muart5;
#endif
#ifdef Enabled_MUart6
	else if(muart6.puart == huart)
		return &muart6;
#endif
	else
		return NULL;
}
/**
  * @brief  串口混合Tx触发函数
  * @param  mhuart: 混合串口指针
  * @param  pData: 源数组
  * @param  len: 发送数量
  * @return HAL_StatusTypeDef
**/
HAL_StatusTypeDef Bsp_UARTMixed_TxTrigger(UartMixed_TypeDef *muart, char *pData, uint16_t len)
{
	char *pdata;
	
#if(MUART_MODE == MUART_TX_MODE_POPUP)
	//检查串口是否就绪
	if(muart->puart->gState != HAL_UART_STATE_READY)
		return HAL_BUSY;
#elif(MUART_MODE == MUART_TX_MODE_BLOCKING)
	//等待就绪
	while(muart->puart->gState != HAL_UART_STATE_READY);
#endif
	muart->len_tx = len;
	//转移数据到发送缓冲区
	if(muart->len_tx > UART_Tx_BuffSize)
		pdata = pData;
	else
	{
		memcpy(muart->buff_tx, pData, muart->len_tx);
		pdata = muart->buff_tx;
	}
	
	//停止串口接收
	HAL_UART_AbortReceive(muart->puart);
	
	//开始DMA发送
	HAL_StatusTypeDef hal_status;
	Bsp_UARTDir_Control(muart, true);	//设置方向为发送
	hal_status = HAL_UART_Transmit_DMA(muart->puart, (uint8_t*)pdata, muart->len_tx);
	if(hal_status == HAL_OK)
	{
		muart->count_tx += muart->len_tx;
		
		return hal_status;
	}
	else
	{
		Bsp_UARTDir_Control(muart, false);	//发送失败-设置方向为接收
		
		return hal_status;
	}
}
/*********************************************************************/
/***************************   HAl库回调函数   ***********************/
/*********************************************************************/
/**
  * @brief  HAL库串口Rx传输完成回调函数(源HAL库虚函数)
  * @call   <- USART1_IRQHandler
  * @call   <- UART_DMAReceiveCplt <- HAL_DMA_IRQHandler
  * @param  huart: 串口指针
  * @retval NULL
  须在中断加入：
  if(__HAL_UART_GET_FLAG(&huartx, UART_FLAG_IDLE))
  {//检测串口是否产生了空闲中断
    __HAL_UART_CLEAR_IDLEFLAG(&huartx);
    //调用串口接收函数(在uart_mixed中实例化)
    HAL_UART_RxCpltCallback(&huartx);
  }
**/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UartMixed_TypeDef *muart;
	
	muart = Seek_UART_Mixed(huart);
	if(!muart)
		return;
	
	//停止串口接收
	HAL_UART_AbortReceive(muart->puart);
	
	//拷贝数据到接收中转区
	muart->len_rx = UART_Rx_BuffSize - __HAL_DMA_GET_COUNTER(muart->pdma_rx);	//获得接收到的数据量
	memcpy(muart->buff_rx_transfer, muart->buff_rx, muart->len_rx);					//拷贝
	muart->count_rx += muart->len_rx;
	
	//开始DMA接收
	HAL_UART_Receive_DMA(muart->puart, (uint8_t*)muart->buff_rx, UART_Rx_BuffSize);
	
	if(muart->len_rx > 0)
	{
		//调用外部接收完成回调函数
		if(muart->Xfer_External_RxCallback != NULL)
			muart->Xfer_External_RxCallback((muart->buff_rx_transfer), (muart->len_rx));
	}
}
/**
  * @brief  HAL库串口Tx传输完成回调函数(源HAL库虚函数)
  * @call   <- UART_DMATransmitCplt <- HAL_DMA_IRQHandler
  * @param  huart: 串口指针
  * @retval NULL 
**/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UartMixed_TypeDef *muart;
	muart = Seek_UART_Mixed(huart);
	if(!muart)
		return;
	
	Bsp_UARTDir_Control(muart, false);	//设置方向为接收
	
	//开始DMA接收
	HAL_UART_Receive_DMA(muart->puart, (uint8_t*)muart->buff_rx, UART_Rx_BuffSize);
	
	//调用外部发送完成回调函数
	if(muart->Xfer_External_TxCallback != NULL)
		muart->Xfer_External_TxCallback();
}
