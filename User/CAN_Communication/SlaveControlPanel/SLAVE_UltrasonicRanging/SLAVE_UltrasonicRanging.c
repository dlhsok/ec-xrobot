/* 包含头文件 ----------------------------------------------------------------*/
#include "SLAVE_UltrasonicRanging/SLAVE_UltrasonicRanging.h"
#include "CAN_Communication.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static UltrasonicRangingDef_t* head_handle = NULL;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
static uint16_t SLAVE_UltrasonicRanging_ProtocolDefinitions(UltrasonicRangingDef_t* handle,
                                                   uint16_t cannum,
                                                   PgnCanTxDataType *head)
{
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
    handle->CAN_Number = cannum_buff; //第一个帧的CAN编号，后面以次累加计算
	
	head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
								   UltrasonicRanging_Addr,
								   handle->UltrasonicRanging_SetPara.DATE.Serial_Number,
								   0,0,
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,  
                                   handle->UltrasonicRanging_SetPara.Datum,PG_UR_RXSTE_PARA_DATA_LEN,
								   handle->UltrasonicRanging_SetPara.Datum,PG_UR_RXSTE_PARA_DATA_LEN); //参数修改 
    head[cannum_buff-1].ch = 1;
    head[cannum_buff++] = xCreate( ALL_Addr,0,
								   UltrasonicRanging_Addr,
	                              handle->UltrasonicRanging_SetPara.DATE.Serial_Number,
	                               0,0,   
                                   FALSE,PG_TX_FREE,MODE_BRO,TT_CYCLIC,10,0,   
                                   0,0,
	                               handle->UltrasonicRanging_UploadData1.Datum,PG_TXDISTANCE_DATA1_LEN); //数据1反馈 
    head[cannum_buff-1].ch = 1;
    head[cannum_buff++] = xCreate( ALL_Addr,0,
								   UltrasonicRanging_Addr,
	                               handle->UltrasonicRanging_SetPara.DATE.Serial_Number,
	                               0,1,   
                                   FALSE,PG_TX_FREE,MODE_BRO,TT_CYCLIC,10,0,   
                                   0,0,
	                               handle->UltrasonicRanging_UploadData2.Datum,PG_TXDISTANCE_DATA2_LEN); //数据2反馈  
    head[cannum_buff-1].ch = 1;                              
    return cannum_buff;
}

uint16_t SLAVE_UltrasonicRanging_CANinit(uint16_t cannum,void *head)
{
    UltrasonicRangingDef_t* target;
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
	for(target=head_handle; target; target=target->next) 
    {
        cannum_buff = SLAVE_UltrasonicRanging_ProtocolDefinitions(target,cannum_buff,((PgnCanTxDataType*)head));
	}
    
    return cannum_buff;
}

uint8_t SLAVE_UltrasonicRanging_Init(UltrasonicRangingDef_t* handle,uint8_t list)
{
	UltrasonicRangingDef_t* target = head_handle;
    
    memset(handle, 0, sizeof(UltrasonicRangingDef_t));
    
    handle->UltrasonicRanging_SetPara.DATE.Serial_Number = list;
    
	while(target) 
    {
		if(target == handle) 
            return 0;	//already exist.
		target = target->next;
	}
	handle->next = head_handle;
	head_handle = handle;
    
	return 1;
}

void SLAVE_UltrasonicRanging_NumberSet(UltrasonicRangingDef_t* handle,uint8_t number)
{
    if(number > 15)
    {
        number = 15;
    }
    
    handle->UltrasonicRanging_SetPara.DATE.Serial_Number = number;
	handle->UltrasonicRanging_SetPara.DATE.mode.bit.modifyidflg = 1;
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number);
}

uint16_t SLAVE_UltrasonicRanging_DistanceRead(UltrasonicRangingDef_t* handle,uint8_t ch)
{
	uint16_t Distance;
	
	if(ch == 0) Distance = handle->UltrasonicRanging_UploadData1.DATE.Distance1;
	else if(ch == 1) Distance = handle->UltrasonicRanging_UploadData1.DATE.Distance2;
	else if(ch == 2) Distance = handle->UltrasonicRanging_UploadData1.DATE.Distance3;
	else if(ch == 3) Distance = handle->UltrasonicRanging_UploadData1.DATE.Distance4;
	else if(ch == 4) Distance = handle->UltrasonicRanging_UploadData2.DATE.Distance5;
	else Distance = handle->UltrasonicRanging_UploadData2.DATE.Distance6;
	
    return Distance;
}
void SLAVE_UltrasonicRanging_MovementMemory(UltrasonicRangingDef_t* handle)
{
	handle->UltrasonicRanging_SetPara.DATE.mode.bit.memoryflg = 1;
	CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_UltrasonicRanging_CHEnabled(UltrasonicRangingDef_t* handle,
	                                   uint8_t ch1,
                                       uint8_t ch2,
                                       uint8_t ch3,
                                       uint8_t ch4,
                                       uint8_t ch5,
                                       uint8_t ch6)
{
    handle->UltrasonicRanging_SetPara.DATE.enabled.bit.enabled1 = (ch1 != 0);
	handle->UltrasonicRanging_SetPara.DATE.enabled.bit.enabled2 = (ch2 != 0);
	handle->UltrasonicRanging_SetPara.DATE.enabled.bit.enabled3 = (ch3 != 0);
	handle->UltrasonicRanging_SetPara.DATE.enabled.bit.enabled4 = (ch4 != 0);
	handle->UltrasonicRanging_SetPara.DATE.enabled.bit.enabled5 = (ch5 != 0);
	handle->UltrasonicRanging_SetPara.DATE.enabled.bit.enabled6 = (ch6 != 0);
	handle->UltrasonicRanging_SetPara.DATE.mode.bit.enabledflg = 1;
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
