/* 包含头文件 ----------------------------------------------------------------*/
#include "SLAVE_Tracking/SLAVE_Tracking.h"
#include "CAN_Communication.h"


/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static TrackingDef_t* head_handle = NULL;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
static uint16_t SLAVE_Tracking_ProtocolDefinitions(TrackingDef_t* handle,
                                                   uint16_t cannum,
                                                   PgnCanTxDataType *head)
{
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
    handle->CAN_Number = cannum_buff; //第一个帧的CAN编号，后面以次累加计算
    
    head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
								   TrackingPlate_Addr,
	                               handle->Tracking_ModeAndSerialNumber.DATE.list,
	                               0,0,
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,  
                                   handle->Tracking_ModeAndSerialNumber.Datum,PG_SETPATROLPLATEADCDATA_LEN,
								   handle->Tracking_ModeAndSerialNumber.Datum,PG_SETPATROLPLATEADCDATA_LEN); //序号修改     
    head[cannum_buff - 1].ch = 1;                                
    
    head[cannum_buff++] = xCreate( ALL_Addr,0,
								   TrackingPlate_Addr,
	                               handle->Tracking_ModeAndSerialNumber.DATE.list,
	                               0,0,   
                                   FALSE,PG_TX_FREE,MODE_BRO,TT_CYCLIC,10,0,   
                                   0,0,
	                               handle->Tracking_UploadData.Datum,PG_TXPATROLPLATEDATA_LEN); //数据反馈     
    head[cannum_buff - 1].ch = 1;                                
    
    head[cannum_buff++] = xCreate( ALL_Addr,0,
	                               TrackingPlate_Addr,
								   handle->Tracking_ModeAndSerialNumber.DATE.list,
								   1,1,   
                                   FALSE,PG_TX_FREE,MODE_BRO,TT_CYCLIC,10,0,   
                                   0,0,
								   handle->Tracking_UploadADCData.Datum,PG_ACKPATROLPLATEADCDATA_LEN); //数据反馈    
    head[cannum_buff - 1].ch = 1;                                
    
    return cannum_buff;
}

uint16_t SLAVE_Tracking_CANinit(uint16_t cannum,void *head)
{
    TrackingDef_t* target;
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
	for(target=head_handle; target; target=target->next) 
    {
        cannum_buff = SLAVE_Tracking_ProtocolDefinitions(target,cannum_buff,((PgnCanTxDataType*)head));
	}
    
    return cannum_buff;
}

uint8_t SLAVE_Tracking_Init(TrackingDef_t* handle,uint8_t list,uint8_t signalinit)
{
	TrackingDef_t* target = head_handle;
    
    memset(handle, 0, sizeof(TrackingDef_t));
    
    handle->Tracking_ModeAndSerialNumber.DATE.list = list;
    
    handle->Tracking_UploadData.DATE.SignalData = signalinit;
    
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

uint8_t SLAVE_Tracking_BoolRead(TrackingDef_t* handle)
{
    return handle->Tracking_UploadData.DATE.SignalData;
}

uint8_t SLAVE_Tracking_ModeRead(TrackingDef_t* handle)
{
    return handle->Tracking_UploadData.DATE.mode;
}

void SLAVE_Tracking_NumberSet(TrackingDef_t* handle,uint8_t number)
{
    if(number > 15)
    {
        number = 15;
    }
    
    handle->Tracking_ModeAndSerialNumber.DATE.mode = 1;
    handle->Tracking_ModeAndSerialNumber.DATE.list = number;
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number);
}

uint8_t SLAVE_Tracking_AdcModeEnable(TrackingDef_t* handle,uint8_t **data_head)
{
    *data_head = &handle->Tracking_UploadADCData.DATE.adc_val[0];
    
    handle->Tracking_ModeAndSerialNumber.DATE.mode = 0;
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number);
    
    return PG_ACKPATROLPLATEADCDATA_LEN;
}

void SLAVE_Tracking_AdcModeDisable(TrackingDef_t* handle)
{
    handle->Tracking_ModeAndSerialNumber.DATE.mode = 3;
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_Tracking_CheckModeEnable(TrackingDef_t* handle)
{
    handle->Tracking_ModeAndSerialNumber.DATE.mode = 2;
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number);
}


