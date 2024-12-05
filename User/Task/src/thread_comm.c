/* Includes ------------------------------------------------------------------*/
#include "thread_comm.h"
#include "bsp.h"
#include "data.h"
#include "CheckCalc.h"
#include "MyModbus.h"
/* Private macros ------------------------------------------------------------*/
//MODBUS发送数据缓冲区长度
#define MODBUS_USART_TX_LEN  	400
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 定义线程控制块指针 */
rt_thread_t thread_comm = RT_NULL;
//MODBUS发送数据缓冲区
uint8_t  MODBUS_USART_TX_BUF[MODBUS_USART_TX_LEN];
uint16_t MODBUS_USART_TX_STA = 0;
/* Private functions ---------------------------------------------------------*/
static void RecognitionModule_Transmit(uint8_t *pData, uint16_t Size)
{
  Bsp_UARTMixed_TxTrigger(&muart5, (char*)pData, Size);
}
static void JY61_DataAnalysis(char *pdata, uint16_t len, uint8_t* pmark, int16_t* pstcAcc, int16_t* pstcGyro, int16_t* pstcAngle, int16_t* pstcTemp )
{
  uint8_t i;

  for(i = 0; i < 3; i++)
  {
    if( 0x55 == pdata[i * 11] )
    {
      //帧头
      if( checksum((uint8_t*)&pdata[i * 11], 10) == pdata[i * 11 + 10] )
      {
        //和校验
        switch(pdata[i * 11 + 1])
        {
        case 0x51:
          pstcAcc[0] = pdata[i * 11 + 2] | (pdata[i * 11 + 3] << 8);
          pstcAcc[1] = pdata[i * 11 + 4] | (pdata[i * 11 + 5] << 8);
          pstcAcc[2] = pdata[i * 11 + 6] | (pdata[i * 11 + 7] << 8);
          *pstcTemp = pdata[i * 11 + 8] | (pdata[i * 11 + 9] << 8);
          *pmark |= 0x01;
          break;
        case 0x52:
          pstcGyro[0] = pdata[i * 11 + 2] | (pdata[i * 11 + 3] << 8);
          pstcGyro[1] = pdata[i * 11 + 4] | (pdata[i * 11 + 5] << 8);
          pstcGyro[2] = pdata[i * 11 + 6] | (pdata[i * 11 + 7] << 8);
          *pmark |= 0x02;
          break;
        case 0x53:
          pstcAngle[0] = pdata[i * 11 + 2] | (pdata[i * 11 + 3] << 8);
          pstcAngle[1] = pdata[i * 11 + 4] | (pdata[i * 11 + 5] << 8);
          pstcAngle[2] = pdata[i * 11 + 6] | (pdata[i * 11 + 7] << 8);
          *pmark |= 0x04;
          break;
        }
      }
    }
  }
}
static void HWT101_DataAnalysis(char *pdata, uint16_t len, uint8_t* pmark, int16_t* pstcGyro, int16_t* pstcAngle )
{
  uint8_t i;

  for(i = 0; i < 2; i++)
  {
    if( 0x55 == pdata[i * 11] )
    {
      //帧头
      if( checksum((uint8_t*)&pdata[i * 11], 10) == pdata[i * 11 + 10] )
      {
        //和校验
        switch(pdata[i * 11 + 1])
        {
        case 0x52:
          *pstcGyro = pdata[i * 11 + 6] | (pdata[i * 11 + 7] << 8);
          *pmark |= 0x02;
          break;
        case 0x53:
          *pstcAngle = pdata[i * 11 + 6] | (pdata[i * 11 + 7] << 8);
          *pmark |= 0x04;
          break;
        }
      }
    }
  }
}
void OvertimeHandler_ClearFlag(struct _s_ModbusPDU *pQ)
{
  if(pQ->PGState_E == MODBUS_PG_TX_ERR)
  {
    pQ->PGState_E = MODBUS_PG_TX_REQ;
  }
}
void ModbusInitialUsedPGs(void)
{
  modbus_dev_S.CreateSendCommand( DEVICE_ROBOTARM_ID,                           //地址域
                                  FUNCTION_CODE_WRITE_HOLDREG_MULTI,            //功能码0x10 写多个保持寄存器
                                  MODBUS_PG_TX_REQ,                             //PDU运行状态
                                  MODBUS_CYCLIC,                                //PDU传输类型
                                  200,                                          //循环速率，单位ms
                                  0,                                            //计时器
                                  0x0000,                                       //数据起始地址
                                  0x0007,                                       //数据终点地址
                                  (void*)&RobotArmData_Struct.ServoPwmDuty[0],  //来源数据地址
                                  0,                                            //发送成功回调函数
                                  OvertimeHandler_ClearFlag );
  modbus_dev_S.CreateSendCommand( DEVICE_ROBOTARM_ID,                           //地址域
                                  FUNCTION_CODE_WRITE_HOLDREG_MULTI,            //功能码0x10 写多个保持寄存器
                                  MODBUS_PG_TX_REQ,                             //PDU运行状态
                                  MODBUS_CYCLIC,                                //PDU传输类型
                                  100,                                          //循环速率，单位ms
                                  100,                                           //计时器
                                  0x1000,                                       //数据起始地址
                                  0x100B,                                       //数据终点地址
                                  (void*)&RobotArmData_Struct.MotorAngle_Target[0],  //来源数据地址
                                  0,                                            //发送成功回调函数
                                  OvertimeHandler_ClearFlag );
  modbus_dev_S.CreateSendCommand( DEVICE_ROBOTARM_ID,                           //地址域
                                  FUNCTION_CODE_READ_HOLDREG,                   //功能码0X03 读保持寄存器
                                  MODBUS_PG_TX_REQ,                             //PDU运行状态
                                  MODBUS_CYCLIC,                                //PDU传输类型
                                  200,                                          //循环速率，单位ms
                                  0,                                            //计时器
                                  0x0008,                                       //数据起始地址
                                  0x0008,                                       //数据终点地址
                                  (void*)&RobotArmData_Struct.UploadData_U.WORD,//来源数据地址
                                  0,                                            //发送成功回调函数
                                  OvertimeHandler_ClearFlag );
}
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
  if( ( len == 33 ) || ( len == 22 ) || ( len == 11 ) )
    JY61_DataAnalysis( pdata, len, &GyroData_Struct.mark, GyroData_Struct.stcAcc, GyroData_Struct.stcGyro, GyroData_Struct.stcAngle, &GyroData_Struct.stcTemp );
  else if(len == 22)
    HWT101_DataAnalysis( pdata, len, &GyroData_Struct.mark, &GyroData_Struct.stcGyro[2], &GyroData_Struct.stcAngle[2] );

  if( ( GyroData_Struct.mark & 0x01 ) == 0x01 )
  {
    GyroData_Struct.temp = GyroData_Struct.stcTemp / 340.0f + 36.53f;
    GyroData_Struct.ax = (float)GyroData_Struct.stcAcc[0] * 16.0f / 32768.0f; //(g 为重力加速度，可取 9.8m/s 2 )
    GyroData_Struct.ay = (float)GyroData_Struct.stcAcc[1] * 16.0f / 32768.0f; //(g 为重力加速度，可取 9.8m/s 2 )
    GyroData_Struct.az = (float)GyroData_Struct.stcAcc[2] * 16.0f / 32768.0f; //(g 为重力加速度，可取 9.8m/s 2 )
    GyroData_Struct.mark &= (0xfe);
  }
  if( ( GyroData_Struct.mark & 0x02 ) == 0x02 )
  {
    GyroData_Struct.wx = (float)GyroData_Struct.stcGyro[0] * 2000 / 32768.0f;
    GyroData_Struct.wy = (float)GyroData_Struct.stcGyro[1] * 2000 / 32768.0f;
    GyroData_Struct.wz = (float)GyroData_Struct.stcGyro[2] * 2000 / 32768.0f;
    GyroData_Struct.mark &= (0xfd);
  }
  if( ( GyroData_Struct.mark & 0x04 ) == 0x04 )
  {
    GyroData_Struct.Roll  = GyroData_Struct.stcAngle[0] * 180 / 32768.0f; //滚转角（x 轴）
    GyroData_Struct.Pitch = GyroData_Struct.stcAngle[1] * 180 / 32768.0f; //俯仰角（y 轴）
    GyroData_Struct.Yaw   = GyroData_Struct.stcAngle[2] * 180 / 32768.0f; //偏航角（z 轴）
    GyroData_Struct.mark &= (0xfb);
  }
}
void XferExternalUart4Rx_Handler(char *pdata, uint16_t len)
{
  modbus_dev_S.receive((uint8_t*)pdata, len);
}
void XferExternalUart5Rx_Handler(char *pdata, uint16_t len)
{
  uint8_t i;
  char *pdat;
	static uint32_t receive_count;

  RecognitionModule_ReceivingProcess(&RecognitionModule_t, (uint8_t*)pdata, len);
  if( RecognitionModule_t.RecognitionModuleSte == RM_succeed )
  {
    //接收成功
    for(i = 0; i < (Recognition_RX_LEN - 1); i++)
      Recognition_Buffer[i] = pdata[i];
    Recognition_Buffer[i] = 0;
    bool_recognitionflag = 1;

    pdat = (char*)Recognition_Buffer;
    while(1)
    {
      if( ( *pdat == '(' ) || ( *pdat == '\0' ) )
        break;
      else
        pdat++;
    }
    pdat++;
    BrickData_Struct.x = atoi(pdat);
    while(1)
    {
      if( ( *pdat == ',' ) || ( *pdat == '\0' ) )
        break;
      else
        pdat++;
    }
    pdat++;
    BrickData_Struct.y = atoi(pdat);
    while(1)
    {
      if( ( *pdat == ',' ) || ( *pdat == '\0' ) )
        break;
      else
        pdat++;
    }
    pdat++;
    BrickData_Struct.yaw = atoi(pdat);
		receive_count++;
		if(receive_count > BRICK_DATA_BUFF_DELAY_INDEX){
			BrickData_Struct.rec_flg = true;
		}
		else
			BrickData_Struct.rec_flg = false;
		
		BrickData_Struct_Buff[brick_data_buff_push_index] = BrickData_Struct;
		
		brick_data_buff_pop_index = brick_data_buff_push_index - BRICK_DATA_BUFF_DELAY_INDEX;
		brick_data_buff_push_index++;
		if(brick_data_buff_pop_index < 0)
		{
			brick_data_buff_pop_index+=BRICK_DATA_BUFF_LEN;
		}
		if(brick_data_buff_push_index == BRICK_DATA_BUFF_LEN)
			brick_data_buff_push_index = 0;
  }
}


/***********************************************************************************************

                                       应用任务函数

************************************************************************************************/
void comm_task(void *pvParameters)
{
#ifdef __SLAVE_SteeringEngine_6CH_H__
  uint8_t time = 0;
#endif
  while(1)
  {
#ifdef __SLAVE_SteeringEngine_6CH_H__
    if(++time == 20)
    {
      time = 0;
      SLAVE_SteeringEngine6CH_MoreMotorControl( &Servo_S,
          AngleValue[0], AngleTime[0],
          AngleValue[1], AngleTime[1],
          AngleValue[2], AngleTime[2],
          AngleValue[3], AngleTime[3],
          AngleValue[4], AngleTime[4],
          AngleValue[5], AngleTime[5] );
    }
#endif
    //=========CAN通讯协议============================
    CANCommunication_Scan();            //最好也1ms进行扫描
    //=========modbus通讯协议============================
    modbus_dev_S.TimerActuator(); //modbus计时器
    modbus_dev_S.scan();//通讯协议栈数据处理
    if(modbus_dev_S.transfer(MODBUS_USART_TX_BUF, &MODBUS_USART_TX_STA))
    {
      //数据协议栈数据发送
      Bsp_UARTMixed_TxTrigger(&muart4, (char*)MODBUS_USART_TX_BUF, MODBUS_USART_TX_STA);
    }
    //=========识别模块通讯协议============================
    RecognitionModule_Scan1Ms(&RecognitionModule_t);
    //=========数据储存============================
    Application_DataFlash_Modification();
    //延时1MS
    My_mDelay( 0 );
  }
}
/***********************************************************************************************

                                       应用主函数

************************************************************************************************/
int Task_COMM_create(void)
{
  /*********************直流电机初始化***************************/
#ifdef __SLAVE_DCMotorMiniwatt_H__
  SLAVE_DCMotorMiniwatt_Init(&DCMotorMiniwatt1_S, 1); //ID号1
  SLAVE_DCMotorMiniwatt_Init(&DCMotorMiniwatt2_S, 2); //ID号2
  SLAVE_DCMotorMiniwatt_Init(&DCMotorMiniwatt3_S, 3); //ID号3
  SLAVE_DCMotorMiniwatt_Init(&DCMotorMiniwatt4_S, 4); //ID号4
#endif
  /*********************舵机初始化***************************/
#ifdef __SLAVE_SteeringEngine_6CH_H__
  SLAVE_SteeringEngine6CH_Init(&Servo_S, 1); //舵机模块参数初始化
#endif
  /*********************巡线初始化***************************/
#ifdef __SLAVE_Tracking_H__
  SLAVE_Tracking_Init(&Tracking_Device1, 1, 0xff); //初始化巡线条模块,ID为1
  SLAVE_Tracking_Init(&Tracking_Device2, 2, 0xff); //初始化巡线条模块,ID为2
  SLAVE_Tracking_Init(&Tracking_Device3, 3, 0xff); //初始化巡线条模块,ID为3
  SLAVE_Tracking_Init(&Tracking_Device4, 4, 0xff); //初始化巡线条模块,ID为4

#endif
  /*********************超声波初始化***************************/
#ifdef __SLAVE_UltrasonicRanging_H__
  SLAVE_UltrasonicRanging_Init(&UltrasonicRanging_S, 1); //初始化超声波模块,ID为1
#endif
  /**********************CAN初始化***************************/
  /* CAN1 1000kbps * CAN2 500kbps */
  CANCommunication_Init();
  SLAVE_UltrasonicRanging_CHEnabled(&UltrasonicRanging_S, 1, 1, 1, 1, 0, 0);
  /**********************MODBUS初始化**************************/
  modbus_dev_S.init();
  /**********************串口初始化***************************/
  Bsp_UartMixed_Init( &muart2, XferExternalUart2Rx_Handler, 0 );
  Bsp_UartMixed_Init( &muart4, XferExternalUart4Rx_Handler, 0 );
  Bsp_UartMixed_Init( &muart5, XferExternalUart5Rx_Handler, 0 );
  /**********************识别模块通讯初始化**************************/
  RecognitionModule_Init(&RecognitionModule_t, RecognitionModule_Transmit);
  /**********************创建任务****************************/
  thread_comm = rt_thread_create("comm",           /* 线程名字 */
                                 comm_task,        /* 线程入口函数 */
                                 RT_NULL,          /* 线程入口函数参数 */
                                 1024,              /* 线程栈大小 */
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



















