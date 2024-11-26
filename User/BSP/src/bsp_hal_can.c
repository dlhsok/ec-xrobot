/* Includes ------------------------------------------------------------------*/
#include "bsp_hal_can.h"
#include "can.h"
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
typedef struct
{
	uint8_t front;
    uint8_t rear;
	CanRxMemberType data[CAN_QUEUE_NUM];
}CanRxQueueType;
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CanRxQueueType CanRxQueueStruct;//队列数据
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @brief  判断队列是否已满
  * @param  
  * @retval QUEUE_TRUE;  已满
            QUEUE_FALSE; 未满
  */
static uint8_t CAN_FullQueue(CanRxQueueType *pQ)
{
	if(((pQ->rear + 1)%CAN_QUEUE_NUM) == pQ->front  )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
/** @brief  判断队列是否为空
  * @param  
  * @retval QUEUE_TRUE; 空
            QUEUE_FALSE;没空
  */
static uint8_t CAN_EmputQueue(CanRxQueueType *pQ)
{
	if ( pQ->front == pQ->rear )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
/** @brief  入队
  * @param  val:入队值
  * @retval QUEUE_FALSE;  失败
            QUEUE_TRUE;   成功
  */
static uint8_t CAN_EnterQueue(CanRxQueueType *pQ,CanRxMemberType val)
{
	if ( CAN_FullQueue(pQ) )
	{
		return FALSE;
	}
	else
	{
		pQ->data[pQ->rear] = val;
		pQ->rear = (pQ->rear+1) % CAN_QUEUE_NUM;

		return TRUE;
	}
}
/** @brief  出队
  * @param  *val:出队数据存放地址
  * @retval QUEUE_FALSE;  失败
            QUEUE_TRUE;   成功
  */
uint8_t CAN_OutQueue(CanRxMemberType * pVal)
{
	if ( CAN_EmputQueue(&CanRxQueueStruct) )
	{
		return FALSE;
	}
	else
	{
		*pVal = CanRxQueueStruct.data[CanRxQueueStruct.front];
		CanRxQueueStruct.front = (CanRxQueueStruct.front+1) % CAN_QUEUE_NUM;

		return TRUE;
	}
}
static void HAL_USER_CANFilterConfig(void)
{
    CAN_FilterTypeDef  sFilterConfig;
    
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
    
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	
	/* Start the CAN peripheral */
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
	
	/* Activate CAN RX notification */
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}
    sFilterConfig.FilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	
	/* Start the CAN peripheral */
	if (HAL_CAN_Start(&hcan2) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
	
	/* Activate CAN RX notification */
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}
}
void Bsp_Cancom_ConfigurationInit(void)
{
	MX_CAN1_Init();//1000k
	MX_CAN2_Init();//500k
}
void Bsp_Cancom_FilterInit(void)
{
    HAL_USER_CANFilterConfig();
}

uint32_t Bsp_Cancom_ReadERR(void)
{
	return (hcan1.ErrorCode+hcan2.ErrorCode);
}

/** @brief  发送一帧数据
  * @param  
  * @retval 1：发送失败   0：发送成功
  */
uint8_t CANSend(uint32_t Source,
	            uint32_t IDE,
                uint32_t RTR,
                uint8_t* Datum,
                uint8_t DataLength,
                uint8_t ch)       
{
	uint8_t err;
	CAN_TxHeaderTypeDef   TxHeader;
    uint32_t              TxMailbox;
	
	CAN_HandleTypeDef *hcan;
	
	if(DataLength > 8)
	{
		DataLength = 8;
	}
	TxHeader.ExtId = Source; 
	TxHeader.StdId = Source; 
	TxHeader.IDE = IDE; 
	TxHeader.RTR = RTR;    
	TxHeader.DLC = DataLength;
	if(ch == 0) hcan = &hcan1;
	else hcan = &hcan2;
	err = HAL_CAN_AddTxMessage(hcan, &TxHeader, Datum, &TxMailbox);
	
	if(err != HAL_OK)
	{
		HAL_CAN_ResetError(hcan);
	}
	
	return err;
}
__weak void QueueOverflow_Dispose(CanRxMemberType *pQ)
{
	UNUSED(pQ);
}
/** @brief  CAN接收中断回调函数
  * @param  
  * @retval 
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t               RxData[8];
	CanRxMemberType       val_buf;
	uint8_t i;
	/* Get RX message */
	if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	{
		if( CanHandle->Instance == CAN1 )
			val_buf.Channel = 0;
		else
			val_buf.Channel = 1;
		//接收一帧数据进行入队
	    val_buf.IDE = RxHeader.IDE;//标识符
	    val_buf.RTR = RxHeader.RTR;//消息类型
	    val_buf.DLC = RxHeader.DLC;//数据长度
		if(val_buf.IDE == CAN_ID_EXT) //ID
			val_buf.Id = RxHeader.ExtId;//扩展ID
		else
			val_buf.Id = RxHeader.StdId;//标准ID
		for(i=0; i<8; i++) //数据
		{
		   if(i<val_buf.DLC)
				val_buf.Data[i] = RxData[i];
		   else
			   val_buf.Data[i] = 0;
		}
		i = CAN_EnterQueue(&CanRxQueueStruct,val_buf);
		//队列溢出
        if(FALSE == i)
        {
           //...
			QueueOverflow_Dispose(&val_buf);
        }
	}
}
