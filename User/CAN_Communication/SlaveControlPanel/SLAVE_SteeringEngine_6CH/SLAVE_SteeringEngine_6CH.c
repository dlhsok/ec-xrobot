/* 包含头文件 ----------------------------------------------------------------*/
#include "SLAVE_SteeringEngine_6CH/SLAVE_SteeringEngine_6CH.h"
#include "string.h"
#include "CAN_Communication.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static SteeringEngine6CHDef_t* head_handle = NULL;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
static uint16_t SLAVE_SteeringEngine_6CH_ProtocolDefinitions(SteeringEngine6CHDef_t* handle,
															 uint16_t cannum,
															 PgnCanTxDataType *head)
{
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
    handle->CAN_Number = cannum_buff; //第一个帧的CAN编号，后面以次累加计算
    
    head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
								   SteeringEngine6CHPlate_Addr,
								   handle->SteeringEngine_SetPara.DATE.Serial_Number,
								   5,5,
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,  
                                   handle->SteeringEngine_SetPara.Datum,PG_RXSTE_PARA_DATA_LEN,
								   handle->SteeringEngine_SetPara.Datum,PG_RXSTE_PARA_DATA_LEN); //参数修改    
    head[cannum_buff-1].ch = 1;       
    head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
	                               SteeringEngine6CHPlate_Addr,
								   handle->SteeringEngine_SetPara.DATE.Serial_Number,
								   2,2,   
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,    
                                   handle->SteeringEngine_ControlData[0].Datum,PG_RXSTE_ServoCtrl_DATA_LEN,
								   handle->SteeringEngine_ControlData[0].Datum,PG_RXSTE_ServoCtrl_DATA_LEN); //控制数据帧初始化     
    head[cannum_buff-1].ch = 1;   
    head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
	                               SteeringEngine6CHPlate_Addr,
								   handle->SteeringEngine_SetPara.DATE.Serial_Number,
								   3,3,   
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,    
                                   handle->SteeringEngine_ControlData[1].Datum,PG_RXSTE_ServoCtrl_DATA_LEN,
								   handle->SteeringEngine_ControlData[1].Datum,PG_RXSTE_ServoCtrl_DATA_LEN);
    head[cannum_buff-1].ch = 1; 
    head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
	                               SteeringEngine6CHPlate_Addr,
								   handle->SteeringEngine_SetPara.DATE.Serial_Number,
								   4,4,   
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,    
                                   handle->SteeringEngine_ControlData[2].Datum,PG_RXSTE_ServoCtrl_DATA_LEN,
								   handle->SteeringEngine_ControlData[2].Datum,PG_RXSTE_ServoCtrl_DATA_LEN);
    head[cannum_buff-1].ch = 1; 
	
    return cannum_buff;
}

uint16_t SLAVE_SteeringEngine6CH_CANinit(uint16_t cannum,void *head)
{
    SteeringEngine6CHDef_t* target;
    uint16_t cannum_buff;
	
    cannum_buff = cannum;
	for(target=head_handle; target; target=target->next) 
    {
        cannum_buff = SLAVE_SteeringEngine_6CH_ProtocolDefinitions(target,cannum_buff,((PgnCanTxDataType*)head));
	}
   
    return cannum_buff;
}

uint8_t SLAVE_SteeringEngine6CH_Init(SteeringEngine6CHDef_t* handle,uint8_t list)
{
	SteeringEngine6CHDef_t* target = head_handle;
    
    memset(handle, 0, sizeof(SteeringEngine6CHDef_t));
    
    handle->SteeringEngine_SetPara.DATE.Serial_Number = list;
    
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
void SLAVE_SteeringEngine6CH_NumberSet(SteeringEngine6CHDef_t* handle,uint8_t number)
{
    if(number > 15)
    {
        number = 15;
    }
    
    handle->SteeringEngine_SetPara.DATE.Serial_Number = number;
	handle->SteeringEngine_SetPara.DATE.mode.bit.modifyidflg = 1;
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_SteeringEngine6CH_MovementMemory(SteeringEngine6CHDef_t* handle)
{
	handle->SteeringEngine_SetPara.DATE.mode.bit.memoryflg = 1;
	CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_SteeringEngine6CH_FullActRun(SteeringEngine6CHDef_t* handle,uint8_t actFullnum)
{
	handle->SteeringEngine_SetPara.DATE.mode.bit.actFullRunflg = 1;
	handle->SteeringEngine_SetPara.DATE.actFullnum = actFullnum;
	CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_SteeringEngine6CH_FullActStop(SteeringEngine6CHDef_t* handle)
{
	handle->SteeringEngine_SetPara.DATE.mode.bit.actFullStopflg = 1;
	CAN_TT_ASYNC_Enable(handle->CAN_Number);
}
void SLAVE_SteeringEngine6CH_MotorControl(SteeringEngine6CHDef_t* handle,
                                       uint8_t  Channel,
                                       uint16_t ServoPwmDutySetCh1,
									   uint16_t time1,
									   uint16_t ServoPwmDutySetCh2,
									   uint16_t time2) 
{
	if(ServoPwmDutySetCh1 < 500)
	{
		ServoPwmDutySetCh1 = 0;
	}
	if(ServoPwmDutySetCh1 > 2500)
	{
		ServoPwmDutySetCh1 = 2500;
	}
	if(time1 < 20)
	{
		time1 = 20;
	}
	if(time1 > 30000)
	{
		time1 = 30000;
	}
	
	if(ServoPwmDutySetCh2 < 500)
	{
		ServoPwmDutySetCh2 = 0;
	}
	if(ServoPwmDutySetCh2 > 2500)
	{
		ServoPwmDutySetCh2 = 2500;
	}
	if(time2 < 20)
	{
		time2 = 20;
	}
	if(time2 > 30000)
	{
		time2 = 30000;
	}
    switch(Channel)
    {
        case SteeringEngine_CHANNEL1_CHANNEL2:
            handle->SteeringEngine_ControlData[0].DATE.ServoPwmDutySetCh1 = ServoPwmDutySetCh1;
            handle->SteeringEngine_ControlData[0].DATE.TimeCh1 = time1;
		    handle->SteeringEngine_ControlData[0].DATE.ServoPwmDutySetCh2 = ServoPwmDutySetCh2;
            handle->SteeringEngine_ControlData[0].DATE.TimeCh2 = time2;
		    
		    CAN_TT_ASYNC_Enable(handle->CAN_Number+1);
            break;
        case SteeringEngine_CHANNEL3_CHANNEL4:
            handle->SteeringEngine_ControlData[1].DATE.ServoPwmDutySetCh1 = ServoPwmDutySetCh1;
            handle->SteeringEngine_ControlData[1].DATE.TimeCh1 = time1;
		    handle->SteeringEngine_ControlData[1].DATE.ServoPwmDutySetCh2 = ServoPwmDutySetCh2;
            handle->SteeringEngine_ControlData[1].DATE.TimeCh2 = time2;
		    
		    CAN_TT_ASYNC_Enable(handle->CAN_Number+2);
            break;
        case SteeringEngine_CHANNEL5_CHANNEL6:
            handle->SteeringEngine_ControlData[2].DATE.ServoPwmDutySetCh1 = ServoPwmDutySetCh1;
            handle->SteeringEngine_ControlData[2].DATE.TimeCh1 = time1;
		    handle->SteeringEngine_ControlData[2].DATE.ServoPwmDutySetCh2 = ServoPwmDutySetCh2;
            handle->SteeringEngine_ControlData[2].DATE.TimeCh2 = time2;
		
		    CAN_TT_ASYNC_Enable(handle->CAN_Number+3);
            break;
        default:
            break;
    }
}

void SLAVE_SteeringEngine6CH_MoreMotorControl(SteeringEngine6CHDef_t* handle,
                                           uint16_t ServoPwmDutySetCh1,uint16_t time1,
									       uint16_t ServoPwmDutySetCh2,uint16_t time2,
                                           uint16_t ServoPwmDutySetCh3,uint16_t time3,
										   uint16_t ServoPwmDutySetCh4,uint16_t time4,
									       uint16_t ServoPwmDutySetCh5,uint16_t time5,
                                           uint16_t ServoPwmDutySetCh6,uint16_t time6
                                          )
{
    if(ServoPwmDutySetCh1 < 500)
	{
		ServoPwmDutySetCh1 = 0;
	}
	if(ServoPwmDutySetCh1 > 2500)
	{
		ServoPwmDutySetCh1 = 2500;
	}
	if(time1 < 20)
	{
		time1 = 20;
	}
	if(time1 > 30000)
	{
		time1 = 30000;
	}
	if(ServoPwmDutySetCh2 < 500)
	{
		ServoPwmDutySetCh2 = 0;
	}
	if(ServoPwmDutySetCh2 > 2500)
	{
		ServoPwmDutySetCh2 = 2500;
	}
	if(time2 < 20)
	{
		time2 = 20;
	}
	if(time2 > 30000)
	{
		time2 = 30000;
	}
	handle->SteeringEngine_ControlData[0].DATE.ServoPwmDutySetCh1 = ServoPwmDutySetCh1;
	handle->SteeringEngine_ControlData[0].DATE.TimeCh1 = time1;
	handle->SteeringEngine_ControlData[0].DATE.ServoPwmDutySetCh2 = ServoPwmDutySetCh2;
	handle->SteeringEngine_ControlData[0].DATE.TimeCh2 = time2;
	
	if(ServoPwmDutySetCh3 < 500)
	{
		ServoPwmDutySetCh3 = 0;
	}
	if(ServoPwmDutySetCh3 > 2500)
	{
		ServoPwmDutySetCh3 = 2500;
	}
	if(time3 < 20)
	{
		time3 = 20;
	}
	if(time3 > 30000)
	{
		time3 = 30000;
	}
	if(ServoPwmDutySetCh4 < 500)
	{
		ServoPwmDutySetCh4 = 0;
	}
	if(ServoPwmDutySetCh4 > 2500)
	{
		ServoPwmDutySetCh4 = 2500;
	}
	if(time4 < 20)
	{
		time4 = 20;
	}
	if(time4 > 30000)
	{
		time4 = 30000;
	}
	handle->SteeringEngine_ControlData[1].DATE.ServoPwmDutySetCh1 = ServoPwmDutySetCh3;
	handle->SteeringEngine_ControlData[1].DATE.TimeCh1 = time3;
	handle->SteeringEngine_ControlData[1].DATE.ServoPwmDutySetCh2 = ServoPwmDutySetCh4;
	handle->SteeringEngine_ControlData[1].DATE.TimeCh2 = time4;
	
	if(ServoPwmDutySetCh5 < 500)
	{
		ServoPwmDutySetCh5 = 0;
	}
	if(ServoPwmDutySetCh5 > 2500)
	{
		ServoPwmDutySetCh5 = 2500;
	}
	if(time5 < 20)
	{
		time5 = 20;
	}
	if(time5 > 30000)
	{
		time5 = 30000;
	}
	if(ServoPwmDutySetCh6 < 500)
	{
		ServoPwmDutySetCh6 = 0;
	}
	if(ServoPwmDutySetCh6 > 2500)
	{
		ServoPwmDutySetCh6 = 2500;
	}
	if(time6 < 20)
	{
		time6 = 20;
	}
	if(time6 > 30000)
	{
		time6 = 30000;
	}
	handle->SteeringEngine_ControlData[2].DATE.ServoPwmDutySetCh1 = ServoPwmDutySetCh5;
	handle->SteeringEngine_ControlData[2].DATE.TimeCh1 = time5;
	handle->SteeringEngine_ControlData[2].DATE.ServoPwmDutySetCh2 = ServoPwmDutySetCh6;
	handle->SteeringEngine_ControlData[2].DATE.TimeCh2 = time6;
	
	CAN_TT_ASYNC_Enable(handle->CAN_Number+1);
    CAN_TT_ASYNC_Enable(handle->CAN_Number+2);
	CAN_TT_ASYNC_Enable(handle->CAN_Number+3);
}






