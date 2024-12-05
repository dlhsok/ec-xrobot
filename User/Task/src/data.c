/* Includes ------------------------------------------------------------------*/
#include "data.h"
#include "CheckCalc.h"
#include <rthw.h>
/* Private macros ------------------------------------------------------------*/
#define VER_LEN                20  //�汾�ֽڳ���
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
const FlashData_Typedef FactoryData_S =
{
  .FH1                  	  = 0x5A,
  .FH2                      = 0xA5,
  .Servo_RotationClawInit   = 1420,
  .Servo_Claw_S             = 1750,
  .Servo_Claw_J             = 1300,
  .FlashData_Len            = sizeof( FlashData_Typedef ), //�洢���ݵĳ���
  .Bit16CRC                 = 0x4203,
};
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
FlashData_Typedef FlashData_Struct;
AppData_Typedef   AppData_Struct;
/*******ֱ���������ģ����������*******/
#ifdef __SLAVE_DCMotorMiniwatt_H__
DCMotorMiniwattDef_t DCMotorMiniwatt_S[4];
#endif
/*******�������ģ����������*******/
#ifdef __SLAVE_SteeringEngine_6CH_H__
SteeringEngine6CHDef_t Servo_S;
uint16_t AngleValue[6] = {1500, 1500, 1500, 1500, 1500, 1500};
uint16_t AngleTime[6]  = {20, 20, 20, 20, 20, 20};
#endif
/*******Ѳ����ģ����������*******/
#ifdef __SLAVE_Tracking_H__
TrackingDef_t Tracking_Device[4];
#endif
/*******������ģ����������*******/
#ifdef __SLAVE_UltrasonicRanging_H__
UltrasonicRangingDef_t UltrasonicRanging_S;
#endif
/*******��������������*******/
GyroData_Typedef GyroData_Struct = {0};
/*******ң����������*******/
_s_RemoteCtrl_Info sRemoteCtrl_Info = {0};
/*******��е����������*******/
RobotArmData_Typedef RobotArmData_Struct =
{
  .ServoPwmDuty    = {1500, 1500, 1500},
  .ServoTime       = {20, 20, 20},
  .ID              = DEVICE_ROBOTARM_ID,
  .SCARAflg_U.WORD = 0,
  .MotorAngle_Target[0] = 0,
  .MotorAngle_Target[1] = 40,
  .MotorAngle_Target[2] = -84.5f,
  /***********��е������ϵ***********/
  .SCARA_Cartesian[0] = 270.0f,
  .SCARA_Cartesian[1] = 0.0f,
  .SCARA_Cartesian[2] = 50.0f,
};
/*******ʶ��ģ����������*******/
RecognitionModule_s RecognitionModule_t;
uint8_t Recognition_Buffer[Recognition_RX_LEN] = {0};
uint8_t bool_recognitionflag = 0;
/*******ש��������*******/
BrickData_TypeDef BrickData_Struct = {0}; // ש�鵱ǰ����
BrickData_TypeDef BrickData_Struct_Buff[BRICK_DATA_BUFF_LEN] = {0}; // ש�����ݻ����������ڶ���ʱ����
int16_t brick_data_buff_pop_index = 0; // ����BRICK_DATA_BUFF_DELAY_INDEX��Ӧ�������±�
int16_t brick_data_buff_push_index = 0; // �������ݶ�Ӧ�������±�
int BRICK_DATA_BUFF_DELAY_INDEX = 0;
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  �޸Ĺ������ݴ�flash
  * @param  None
  * @retval 1���ɹ�  0��ʧ��
  */
uint8_t Application_DataFlash_Modification(void)
{
  uint8_t succeed = 0;

  if(DataStorageFlg_Start == AppData_Struct.DataStorage_Flg)
  {
    AppData_Struct.DataStorage_Flg = DataStorageFlg_Leisure;
    FlashData_Struct.FlashData_Len = FactoryData_S.FlashData_Len;
    rt_base_t level;
    level = rt_hw_interrupt_disable();
    FlashData_Struct.Bit16CRC = CRC16Modbus_Compute(&FlashData_Struct.FH1, FlashData_Struct.FlashData_Len - 2);
    stmflash_write(STM_FLASH_BASE, (uint32_t *)&FlashData_Struct.FH1, FlashData_Struct.FlashData_Len / 4);
    rt_hw_interrupt_enable(level);
  }
  else if(AppData_Struct.DataStorage_Flg > DataStorageFlg_ERRADDR)
  {
    //ֵΪ�Ƿ�ֵ
    AppData_Struct.DataStorage_Flg = DataStorageFlg_Leisure;
  }

  return succeed;
}
int AppData_Init(void)
{
  uint16_t i;

  /****************��һ�δ���flash**********************/
  for(i = 0; i < (FactoryData_S.FlashData_Len / 4); i++)
  {
    if(0xFFFFFFFF != ((uint32_t*)STM_FLASH_BASE)[i])
    {
      break;
    }
  }
  if(i >= (FactoryData_S.FlashData_Len / 4))
  {
    FlashData_Struct = FactoryData_S;
    FlashData_Struct.Bit16CRC = CRC16Modbus_Compute(&FlashData_Struct.FH1, FlashData_Struct.FlashData_Len - 2);
    stmflash_write(STM_FLASH_BASE, (uint32_t *)&FlashData_Struct.FH1, FlashData_Struct.FlashData_Len / 4);
  }
  /****************��ȡ����**********************/
  FlashData_Struct = *((FlashData_Typedef*)STM_FLASH_BASE);
  i = CRC16Modbus_Compute(&FlashData_Struct.FH1, FlashData_Struct.FlashData_Len - 2);
  if( ( FlashData_Struct.FH1 != 0x5A ) \
      || ( FlashData_Struct.FH2 != 0xA5 ) \
      || ( i != FlashData_Struct.Bit16CRC ) )
  {
    //���ݴ���
    FlashData_Struct = FactoryData_S;
    FlashData_Struct.Bit16CRC = CRC16Modbus_Compute(&FlashData_Struct.FH1, FlashData_Struct.FlashData_Len - 2);
    stmflash_write(STM_FLASH_BASE, (uint32_t *)&FlashData_Struct.FH1, FlashData_Struct.FlashData_Len / 4);
  }
  RobotArmData_Struct.ServoPwmDuty[0] = Claw_S;
  RobotArmData_Struct.ServoPwmDuty[1] = Rotation_Claw_Init;
  RobotArmData_Struct.ServoPwmDuty[2] = 1500;

  return 0;
}
INIT_BOARD_EXPORT(AppData_Init);













