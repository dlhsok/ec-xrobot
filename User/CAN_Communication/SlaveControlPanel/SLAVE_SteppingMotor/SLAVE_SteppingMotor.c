/* 包含头文件 ----------------------------------------------------------------*/
#include "SLAVE_SteppingMotor/SLAVE_SteppingMotor.h"
#include "string.h"
#include "CAN_Communication.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static SteppingMotorDef_t* head_handle = NULL;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
static uint16_t SLAVE_SteppingMotor_ProtocolDefinitions(SteppingMotorDef_t* handle,
                                                   uint16_t cannum,
                                                   PgnCanTxDataType *head)
{
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
    handle->CAN_Number = cannum_buff; //第一个帧的CAN编号，后面以次累加计算
    
    head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
								   SLAVE_SteppingMotorPlate_Addr,
	                               handle->SteppingMotor_SerialNumber.DATE.Serial_Number,
	                               1,1,
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,  
                                   handle->SteppingMotor_SerialNumber.Datum,PG_RXSTEPPINGMOTOR_SERIAL_NUMBER_LEN,
	                               handle->SteppingMotor_SerialNumber.Datum,PG_RXSTEPPINGMOTOR_SERIAL_NUMBER_LEN); //序号修改    
    head[cannum_buff-1].ch = 1;                              
    head[cannum_buff++] = xCreate( ALL_Addr,0,
	                               SLAVE_SteppingMotorPlate_Addr,
	                               handle->SteppingMotor_SerialNumber.DATE.Serial_Number,
	                               0,0,   
                                   FALSE,PG_TX_FREE,MODE_BRO,TT_CYCLIC,10,0,   
                                   0,0,
	                               handle->SteppingMotor_UploadData.Datum,PG_TXSTEPPINGMOTORDATA_LEN); //数据反馈       
    head[cannum_buff-1].ch = 1;                              
    head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
	                               SLAVE_SteppingMotorPlate_Addr,
								   handle->SteppingMotor_SerialNumber.DATE.Serial_Number,
								   0,0,   
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,   
                                   handle->SteppingMotor_ControlData.Datum,PG_SETSTEPPINGMOTORDATA_LEN,
								   handle->SteppingMotor_ControlDataAck.Datum,PG_ACKSTEPPINGMOTORDATA_LEN); //控制数据帧初始化                                       
    head[cannum_buff-1].ch = 1;
	
    return cannum_buff;
}

uint16_t SLAVE_SteppingMotorg_CANinit(uint16_t cannum,void *head)
{
    SteppingMotorDef_t* target;
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
	for(target=head_handle; target; target=target->next) 
    {
        cannum_buff = SLAVE_SteppingMotor_ProtocolDefinitions(target,cannum_buff,((PgnCanTxDataType*)head));
	}
    
    return cannum_buff;
}

uint8_t SLAVE_SteppingMotor_Init(SteppingMotorDef_t* handle,uint8_t list)
{
	SteppingMotorDef_t* target = head_handle;
    
    memset(handle, 0, sizeof(SteppingMotorDef_t));
    
    handle->SteppingMotor_SerialNumber.DATE.Serial_Number = list;
    
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


void SLAVE_SteppingMotor_NumberSet(SteppingMotorDef_t* handle,uint8_t number)
{
    if(number > 15)
    {
        number = 15;
    }
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number);
}


uint8_t SLAVE_SteppingMotor_ReadEnabledState(SteppingMotorDef_t* handle)
{
    return handle->SteppingMotor_UploadData.DATE.enabled_state;
}

uint8_t SLAVE_SteppingMotor_ReadRunState(SteppingMotorDef_t* handle)
{
    return handle->SteppingMotor_UploadData.DATE.run_state;
}

uint8_t SLAVE_SteppingMotor_Control(SteppingMotorDef_t* handle,
                                    int16_t  speed_v0,     
                                    int16_t  speed_vt,     
                                    uint16_t step_out,     
                                    uint16_t EnabledTime_MAX
                                   )
{
    uint8_t err;
    
    err = 0;
    if((handle->SteppingMotor_UploadData.DATE.run_state == RUNSTA_STEP)&&(speed_vt != 0))
    {
        err = 1;
    }
    else if ((handle->SteppingMotor_UploadData.DATE.run_state == RUNSTA_SPEED)
           &&(step_out != 0))
    {
        err = 1;
    }
    else
    {
        handle->SteppingMotor_ControlDataAck.DATE.ack = ACK_START;
        handle->SteppingMotor_ControlData.DATE.speed_v0 = speed_v0;
        handle->SteppingMotor_ControlData.DATE.speed_vt = speed_vt;
        handle->SteppingMotor_ControlData.DATE.step_out = step_out;
        handle->SteppingMotor_ControlData.DATE.EnabledTime_MAX = EnabledTime_MAX;
    }
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number+2);
    
    return err;
}










