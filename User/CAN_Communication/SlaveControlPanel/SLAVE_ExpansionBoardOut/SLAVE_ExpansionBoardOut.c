/* 包含头文件 ----------------------------------------------------------------*/
#include "SLAVE_ExpansionBoardOut/SLAVE_ExpansionBoardOut.h"
#include "string.h"
#include "CAN_Communication.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static ExpansionBoardOutDef_t* head_handle = NULL;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
static uint16_t SLAVE_ExpansionBoardOut_ProtocolDefinitions(ExpansionBoardOutDef_t* handle,
                                                   uint16_t cannum,
                                                   PgnCanTxDataType *head)
{
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
    handle->CAN_Number = cannum_buff; //第一个帧的CAN编号，后面以次累加计算
    
    head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
								   ExpansionBoardOut_Addr,
	                               handle->ExpansionBoardOut_SerialNumber.DATE.Serial_Number,
	                               1,1,
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,  
                                   handle->ExpansionBoardOut_SerialNumber.Datum,PG_RXExpansionBoardOut_SERIAL_NUMBER_LEN,
	                               handle->ExpansionBoardOut_SerialNumber.Datum,PG_RXExpansionBoardOut_SERIAL_NUMBER_LEN); //序号修改    
    head[cannum_buff - 1].ch = 1;                               
    
    head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
								   ExpansionBoardOut_Addr,
	                               handle->ExpansionBoardOut_SerialNumber.DATE.Serial_Number,
	                               0,0,
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,  
                                   handle->ExpansionBoardOut_ControlData.Datum,PG_RXSTEExpansionOut_LEN,
	                               handle->ExpansionBoardOut_ControlData.Datum,PG_RXSTEExpansionOut_LEN); //控制数据修改    
    head[cannum_buff - 1].ch = 1;                               
    
    return cannum_buff;
}

uint16_t SLAVE_ExpansionBoardOut_CANinit(uint16_t cannum,void *head)
{
    ExpansionBoardOutDef_t* target;
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
	for(target=head_handle; target; target=target->next) 
    {
        cannum_buff = SLAVE_ExpansionBoardOut_ProtocolDefinitions(target,cannum_buff,((PgnCanTxDataType*)head));
	}
    
    return cannum_buff;
}

uint8_t SLAVE_ExpansionBoardOut_Init(ExpansionBoardOutDef_t* handle,uint8_t list)
{
	ExpansionBoardOutDef_t* target = head_handle;
    
    memset(handle, 0, sizeof(ExpansionBoardOutDef_t));
    
    handle->ExpansionBoardOut_SerialNumber.DATE.Serial_Number = list;
    
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


void SLAVE_ExpansionBoardOut_NumberSet(ExpansionBoardOutDef_t* handle,uint8_t number)
{
    if(number > 15)
    {
        number = 15;
    }
    
    handle->ExpansionBoardOut_SerialNumber.DATE.Serial_Number = number;
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number);
}

void SLAVE_ExpansionBoardOut_OutControl(ExpansionBoardOutDef_t* handle,
                                        uint8_t out1,
                                        uint8_t out2,
                                        uint8_t out3,
                                        uint8_t out4,
										uint8_t out5,
                                        uint8_t out6
                                        )
{
    if(out1 > 1) out1 = 1; 
    handle->ExpansionBoardOut_ControlData.DATE.out1 = out1;
    if(out2 > 1) out2 = 1;         
    handle->ExpansionBoardOut_ControlData.DATE.out2 = out2;
    if(out3 > 1) out3 = 1;       
    handle->ExpansionBoardOut_ControlData.DATE.out3 = out3;
    if(out4 > 1) out4 = 1;      
    handle->ExpansionBoardOut_ControlData.DATE.out4 = out4;
	if(out5 > 1) out5 = 1;      
    handle->ExpansionBoardOut_ControlData.DATE.out5 = out5;
	if(out6 > 1) out6 = 1;      
    handle->ExpansionBoardOut_ControlData.DATE.out6 = out6;
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number+1);
}

void SLAVE_ExpansionBoardOut_MovementMemory(ExpansionBoardOutDef_t* handle)
{
    handle->ExpansionBoardOut_ControlData.DATE.MemoryFlg = 1;
    CAN_TT_ASYNC_Enable(handle->CAN_Number+1);
}
