/* 包含头文件 ----------------------------------------------------------------*/
#include "SLAVE_ExpansionBoardInput/SLAVE_ExpansionBoardInput.h"
#include "string.h"
#include "CAN_Communication.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static ExpansionBoardInputDef_t* head_handle = NULL;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
static uint16_t SLAVE_ExpansionBoardInput_ProtocolDefinitions(ExpansionBoardInputDef_t* handle,
                                                   uint16_t cannum,
                                                   PgnCanTxDataType *head)
{
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
    handle->CAN_Number = cannum_buff; //第一个帧的CAN编号，后面以次累加计算
    
    head[cannum_buff++] = xCreate( DEVICE_ID,DEVICE_LIST,
								   ExpansionBoardInput_Addr,
	                               handle->ExpansionBoardInput_SerialNumber.DATE.Serial_Number,
	                               0,0,
                                   TRUE,PG_TX_FREE,MODE_REQ,TT_ASYNC,10,0,  
                                   handle->ExpansionBoardInput_SerialNumber.Datum,PG_RXExpansionBoard_SERIAL_NUMBER_LEN,
	                               handle->ExpansionBoardInput_SerialNumber.Datum,PG_RXExpansionBoard_SERIAL_NUMBER_LEN); //序号修改     
    head[cannum_buff-1].ch = 1;                              
    head[cannum_buff++] = xCreate( ALL_Addr,0,
	                               ExpansionBoardInput_Addr,
	                               handle->ExpansionBoardInput_SerialNumber.DATE.Serial_Number,
	                               0,0,   
                                   FALSE,PG_TX_FREE,MODE_BRO,TT_CYCLIC,10,0,   
                                   0,0,
	                               handle->ExpansionBoardInput_UploadData.Datum,PG_TXExpansionBoardDATA_LEN); //数据反馈     
    head[cannum_buff-1].ch = 1;
	
    return cannum_buff;
}

uint16_t SLAVE_ExpansionBoardInput_CANinit(uint16_t cannum,void *head)
{
    ExpansionBoardInputDef_t* target;
    uint16_t cannum_buff;
    cannum_buff = cannum;
    
	for(target=head_handle; target; target=target->next) 
    {
        cannum_buff = SLAVE_ExpansionBoardInput_ProtocolDefinitions(target,cannum_buff,((PgnCanTxDataType*)head));
	}
    
    return cannum_buff;
}

uint8_t SLAVE_ExpansionBoardInput_Init(ExpansionBoardInputDef_t* handle,uint8_t list)
{
	ExpansionBoardInputDef_t* target = head_handle;
    
    memset(handle, 0, sizeof(ExpansionBoardInputDef_t));
    
    handle->ExpansionBoardInput_SerialNumber.DATE.Serial_Number = list;
    
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


void SLAVE_ExpansionBoardInput_NumberSet(ExpansionBoardInputDef_t* handle,uint8_t number)
{
    if(number > 15)
    {
        number = 15;
    }
    
    handle->ExpansionBoardInput_SerialNumber.DATE.Serial_Number = number;
    
    CAN_TT_ASYNC_Enable(handle->CAN_Number);
}

uint8_t SLAVE_ExpansionBoardInput_SensorRead(ExpansionBoardInputDef_t* handle,uint8_t list)
{
    uint8_t val;
    
    switch(list)
    {
        case ExpansionBoardInput_Sensor_1:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor0;
            break;
        case ExpansionBoardInput_Sensor_2:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor1;
            break;
        case ExpansionBoardInput_Sensor_3:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor2;
            break;
        case ExpansionBoardInput_Sensor_4:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor3;
            break;
        case ExpansionBoardInput_Sensor_5:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor4;
            break;
        case ExpansionBoardInput_Sensor_6:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor5;
            break;
        case ExpansionBoardInput_Sensor_7:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor6;
            break;
        case ExpansionBoardInput_Sensor_8:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor7;
            break;
        case ExpansionBoardInput_Sensor_9:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor8;
            break;
        case ExpansionBoardInput_Sensor_10:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor9;
            break;
        case ExpansionBoardInput_Sensor_11:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor10;
            break;
        case ExpansionBoardInput_Sensor_12:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor11;
            break;
        case ExpansionBoardInput_Sensor_13:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor12;
            break;
        case ExpansionBoardInput_Sensor_14:
            val = handle->ExpansionBoardInput_UploadData.DATE.sensor13;
            break;
        default:
            val = 255;
            break;
    }
    
    return val;
}


void SLAVE_ExpansionBoardInput_CANRXDispose(void *head,uint32_t Id)
{
    uint8_t i; 
    uint16_t valbuff;
    ExpansionBoardInputDef_t* target;
    
    for(target=head_handle; target; target=target->next) 
    {
        if(Id == ((PgnCanTxDataType*)head)[target->CAN_Number+1].Target)
        {
            for(i=0; i<14; i++)
            {
                if(target->SingleInputSignalData[i].Enabled == 0)
                {
                    valbuff = *((uint16_t*)target->ExpansionBoardInput_UploadData.Datum);
                    if((target->SingleInputSignalData[i].sta == 0)&&
                       ((valbuff&(0x0001<<i)) != 0))
                      
                    {
                        target->SingleInputSignalData[i].sta = 1;
                    }
                    else if((target->SingleInputSignalData[i].sta == 1)&&
                       ((valbuff&(0x0001<<i)) == 0))
                        
                    {
                        target->SingleInputSignalData[i].sta = 0;
                        target->SingleInputSignalData[i].cnt++;
                    }
                }
            }
        }
	}
}




