/* Includes ------------------------------------------------------------------*/
#include "thread_comm.h"
#include "bsp.h"
#include "data.h"
#include "CheckCalc.h"
#include "MyModbus.h"
/* Private macros ------------------------------------------------------------*/
//MODBUS�������ݻ���������
#define MODBUS_USART_TX_LEN  	400
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* �����߳̿��ƿ�ָ�� */
rt_thread_t thread_comm = RT_NULL;
//MODBUS�������ݻ�����
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
      //֡ͷ
      if( checksum((uint8_t*)&pdata[i * 11], 10) == pdata[i * 11 + 10] )
      {
        //��У��
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
      //֡ͷ
      if( checksum((uint8_t*)&pdata[i * 11], 10) == pdata[i * 11 + 10] )
      {
        //��У��
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
  modbus_dev_S.CreateSendCommand( DEVICE_ROBOTARM_ID,                           //��ַ��
                                  FUNCTION_CODE_WRITE_HOLDREG_MULTI,            //������0x10 д������ּĴ���
                                  MODBUS_PG_TX_REQ,                             //PDU����״̬
                                  MODBUS_CYCLIC,                                //PDU��������
                                  200,                                          //ѭ�����ʣ���λms
                                  0,                                            //��ʱ��
                                  0x0000,                                       //������ʼ��ַ
                                  0x0007,                                       //�����յ��ַ
                                  (void*)&RobotArmData_Struct.ServoPwmDuty[0],  //��Դ���ݵ�ַ
                                  0,                                            //���ͳɹ��ص�����
                                  OvertimeHandler_ClearFlag );
  modbus_dev_S.CreateSendCommand( DEVICE_ROBOTARM_ID,                           //��ַ��
                                  FUNCTION_CODE_WRITE_HOLDREG_MULTI,            //������0x10 д������ּĴ���
                                  MODBUS_PG_TX_REQ,                             //PDU����״̬
                                  MODBUS_CYCLIC,                                //PDU��������
                                  100,                                          //ѭ�����ʣ���λms
                                  100,                                           //��ʱ��
                                  0x1000,                                       //������ʼ��ַ
                                  0x100B,                                       //�����յ��ַ
                                  (void*)&RobotArmData_Struct.MotorAngle_Target[0],  //��Դ���ݵ�ַ
                                  0,                                            //���ͳɹ��ص�����
                                  OvertimeHandler_ClearFlag );
  modbus_dev_S.CreateSendCommand( DEVICE_ROBOTARM_ID,                           //��ַ��
                                  FUNCTION_CODE_READ_HOLDREG,                   //������0X03 �����ּĴ���
                                  MODBUS_PG_TX_REQ,                             //PDU����״̬
                                  MODBUS_CYCLIC,                                //PDU��������
                                  200,                                          //ѭ�����ʣ���λms
                                  0,                                            //��ʱ��
                                  0x0008,                                       //������ʼ��ַ
                                  0x0008,                                       //�����յ��ַ
                                  (void*)&RobotArmData_Struct.UploadData_U.WORD,//��Դ���ݵ�ַ
                                  0,                                            //���ͳɹ��ص�����
                                  OvertimeHandler_ClearFlag );
}
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/***********************************************************************************************

                                       Ӧ���¼����жϣ�����

************************************************************************************************/
/**
  * @brief  ���ڽ�������¼�
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
    GyroData_Struct.ax = (float)GyroData_Struct.stcAcc[0] * 16.0f / 32768.0f; //(g Ϊ�������ٶȣ���ȡ 9.8m/s 2 )
    GyroData_Struct.ay = (float)GyroData_Struct.stcAcc[1] * 16.0f / 32768.0f; //(g Ϊ�������ٶȣ���ȡ 9.8m/s 2 )
    GyroData_Struct.az = (float)GyroData_Struct.stcAcc[2] * 16.0f / 32768.0f; //(g Ϊ�������ٶȣ���ȡ 9.8m/s 2 )
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
    GyroData_Struct.Roll  = GyroData_Struct.stcAngle[0] * 180 / 32768.0f; //��ת�ǣ�x �ᣩ
    GyroData_Struct.Pitch = GyroData_Struct.stcAngle[1] * 180 / 32768.0f; //�����ǣ�y �ᣩ
    GyroData_Struct.Yaw   = GyroData_Struct.stcAngle[2] * 180 / 32768.0f; //ƫ���ǣ�z �ᣩ
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
    //���ճɹ�
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

                                       Ӧ��������

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
    //=========CANͨѶЭ��============================
    CANCommunication_Scan();            //���Ҳ1ms����ɨ��
    //=========modbusͨѶЭ��============================
    modbus_dev_S.TimerActuator(); //modbus��ʱ��
    modbus_dev_S.scan();//ͨѶЭ��ջ���ݴ���
    if(modbus_dev_S.transfer(MODBUS_USART_TX_BUF, &MODBUS_USART_TX_STA))
    {
      //����Э��ջ���ݷ���
      Bsp_UARTMixed_TxTrigger(&muart4, (char*)MODBUS_USART_TX_BUF, MODBUS_USART_TX_STA);
    }
    //=========ʶ��ģ��ͨѶЭ��============================
    RecognitionModule_Scan1Ms(&RecognitionModule_t);
    //=========���ݴ���============================
    Application_DataFlash_Modification();
    //��ʱ1MS
    My_mDelay( 0 );
  }
}
/***********************************************************************************************

                                       Ӧ��������

************************************************************************************************/
int Task_COMM_create(void)
{
  /*********************ֱ�������ʼ��***************************/
#ifdef __SLAVE_DCMotorMiniwatt_H__
  SLAVE_DCMotorMiniwatt_Init(&DCMotorMiniwatt1_S, 1); //ID��1
  SLAVE_DCMotorMiniwatt_Init(&DCMotorMiniwatt2_S, 2); //ID��2
  SLAVE_DCMotorMiniwatt_Init(&DCMotorMiniwatt3_S, 3); //ID��3
  SLAVE_DCMotorMiniwatt_Init(&DCMotorMiniwatt4_S, 4); //ID��4
#endif
  /*********************�����ʼ��***************************/
#ifdef __SLAVE_SteeringEngine_6CH_H__
  SLAVE_SteeringEngine6CH_Init(&Servo_S, 1); //���ģ�������ʼ��
#endif
  /*********************Ѳ�߳�ʼ��***************************/
#ifdef __SLAVE_Tracking_H__
  SLAVE_Tracking_Init(&Tracking_Device1, 1, 0xff); //��ʼ��Ѳ����ģ��,IDΪ1
  SLAVE_Tracking_Init(&Tracking_Device2, 2, 0xff); //��ʼ��Ѳ����ģ��,IDΪ2
  SLAVE_Tracking_Init(&Tracking_Device3, 3, 0xff); //��ʼ��Ѳ����ģ��,IDΪ3
  SLAVE_Tracking_Init(&Tracking_Device4, 4, 0xff); //��ʼ��Ѳ����ģ��,IDΪ4

#endif
  /*********************��������ʼ��***************************/
#ifdef __SLAVE_UltrasonicRanging_H__
  SLAVE_UltrasonicRanging_Init(&UltrasonicRanging_S, 1); //��ʼ��������ģ��,IDΪ1
#endif
  /**********************CAN��ʼ��***************************/
  /* CAN1 1000kbps * CAN2 500kbps */
  CANCommunication_Init();
  SLAVE_UltrasonicRanging_CHEnabled(&UltrasonicRanging_S, 1, 1, 1, 1, 0, 0);
  /**********************MODBUS��ʼ��**************************/
  modbus_dev_S.init();
  /**********************���ڳ�ʼ��***************************/
  Bsp_UartMixed_Init( &muart2, XferExternalUart2Rx_Handler, 0 );
  Bsp_UartMixed_Init( &muart4, XferExternalUart4Rx_Handler, 0 );
  Bsp_UartMixed_Init( &muart5, XferExternalUart5Rx_Handler, 0 );
  /**********************ʶ��ģ��ͨѶ��ʼ��**************************/
  RecognitionModule_Init(&RecognitionModule_t, RecognitionModule_Transmit);
  /**********************��������****************************/
  thread_comm = rt_thread_create("comm",           /* �߳����� */
                                 comm_task,        /* �߳���ں��� */
                                 RT_NULL,          /* �߳���ں������� */
                                 1024,              /* �߳�ջ��С */
                                 1,                /* �̵߳����ȼ� */
                                 20);              /* �߳�ʱ��Ƭ */
  if(thread_comm != RT_NULL)
  {
    rt_thread_startup(thread_comm);
    rt_kprintf("thread_comm startup!\n");
  }

  return 0;
}
INIT_APP_EXPORT(Task_COMM_create);



















