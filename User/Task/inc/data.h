#ifndef __DATA_H
#define __DATA_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include "mytype.h"
#include "bsp.h"
#include "RecognitionModule.h"
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#ifndef  FALSE
    #define  FALSE    0
#endif
#ifndef  TRUE
    #define  TRUE     1
#endif

#define DataStorageFlg_Leisure 0   //����
#define DataStorageFlg_Start   1   //���д洢
#define DataStorageFlg_ERR     2   //ʧ��
#define DataStorageFlg_ERRADDR 3   //��ַ����

#ifdef __SLAVE_DCMotorMiniwatt_H__
#define DCMotorMiniwatt1_S DCMotorMiniwatt_S[0]
#define DCMotorMiniwatt2_S DCMotorMiniwatt_S[1]
#define DCMotorMiniwatt3_S DCMotorMiniwatt_S[2]
#define DCMotorMiniwatt4_S DCMotorMiniwatt_S[3]
#endif
#ifdef __SLAVE_Tracking_H__
#define Tracking_Device1 Tracking_Device[0]
#define Tracking_Device2 Tracking_Device[1]
#define Tracking_Device3 Tracking_Device[2]
#define Tracking_Device4 Tracking_Device[3]
#endif
#ifdef __SLAVE_SteeringEngine_6CH_H__
#define SERVO_NUM_MAX 6
#define SERVO_CH1     0
#define SERVO_CH2     1
#define SERVO_CH3     2
#define SERVO_CH4     3
#define SERVO_CH5     4
#define SERVO_CH6     5

#define SERVO_CLAW_J        2300
#define SERVO_CLAW_S        1800
#define SERVO_CLAW_S_MAX    1400

#define SERVO_CLAW SERVO_CH1 //��צ���
#define SERVO_ARM2 SERVO_CH2 //��2�����������צ�ı�
#define SERVO_ARM1 SERVO_CH3 //��1����������ײ�ѡ����
#define SERVO_YAW  SERVO_CH4 //�ײ���ת���
#endif
/******************************************************************************
 *              �����ٶ�����                                             *
 ******************************************************************************/
#define MAX_WHEEL_RPM               (200.0f)
#define RC_CHASSIS_MAX_SPEED_X      (500.0f)     //X�᷽������ٶ�(mm/s)
#define RC_CHASSIS_MAX_SPEED_Y      (500.0f)     //Y�᷽������ٶ�(mm/s)
#define RC_CHASSIS_MAX_SPEED_R      (150.0f)      //��ת����ٶ�(deg/s)
#define RC_GIMBAL_MOVE_RATIO_YAW    (0.12f)        //yaw�ƶ�����
/******************************************************************************
 *              �����������                                             *
 ******************************************************************************/
#define SERVO_CH_NUM            3
#define Recognition_RX_LEN  	20
#define Rotation_Claw_Init      FlashData_Struct.Servo_RotationClawInit
#define Claw_S                  FlashData_Struct.Servo_Claw_S
#define Claw_J                  FlashData_Struct.Servo_Claw_J
/******************************************************************************
 *              ��е�۲�������                                             *
 ******************************************************************************/
#define DEVICE_ROBOTARM_ID      2
#define DEVICE_ROBOTARM_MAX_SPEED_X      (100.0f)     //X�᷽������ٶ�(mm/s)
#define DEVICE_ROBOTARM_MAX_SPEED_Y      (100.0f)     //Y�᷽������ٶ�(mm/s)
#define DEVICE_ROBOTARM_LOCATION_TRACKER_RATIO (100.0f) // λ�ø������Ŵ���
/******************************************************************************
 *              �����ת��������                                             *
 ******************************************************************************/
#define FWD                         (0) //��ת
#define REV                         (1) //��ת
#define DCMOTOR1_DIR                FWD
#define DCMOTOR2_DIR                REV
#define DCMOTOR3_DIR                REV
#define DCMOTOR4_DIR                FWD


/* Exported types ------------------------------------------------------------*/
extern DCMotorMiniwattDef_t DCMotorMiniwatt_S[4];
typedef struct
{
	uint8_t  FH1;
	uint8_t  FH2;
    uint16_t Servo_RotationClawInit;
	uint16_t Servo_Claw_S;
	uint16_t Servo_Claw_J;
	uint16_t FlashData_Len;    //�洢���ݵĳ���
	uint16_t Bit16CRC;
}FlashData_Typedef; //������4���ֽڵı���
typedef struct
{
	uint8_t  DataStorage_Flg;    //���ݴ洢���
}AppData_Typedef;
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t yaw;
	bool rec_flg; // ���ܵ������ݱ�־λ
}BrickData_TypeDef;
typedef struct
{
	__IO uint16_t ServoPwmDuty[SERVO_CH_NUM];	//���PWM������
	__IO uint16_t ServoTime[SERVO_CH_NUM];	    //���ʱ��
	__IO uint16_t ID;    //ID
	__IO union
	{//��е�ۿ��Ʊ��
		uint16_t WORD;
		struct
		{
			uint16_t YV1                  :1;
			uint16_t YV2                  :1;
			uint16_t YV3                  :1;
			uint16_t YV4                  :1;
			uint16_t DazzleLight          :1;
			uint16_t retain0              :1;
			uint16_t AirCtrl              :1;  //���ÿ���
			uint16_t SCARAReset_Enabled   :1;  //��λʹ��
			uint16_t AngleCtrl_Enabled    :1;  //�Ƕȿ���ʹ��
			uint16_t AS5600_Enabled       :1;  //AS5600ʹ��
			uint16_t XYZ_Enabled          :1;  //����ϵʹ��
			uint16_t Scram_Enabled        :1;  //��ͣ
			uint16_t retain1              :4;
		}bit;
	}SCARAflg_U;
	__IO union
	{//��е��״̬��־
		uint16_t WORD;
		struct
		{
			uint16_t IN1                  :1;
			uint16_t IN2                  :1;
			uint16_t IN3                  :1;
			uint16_t IN4                  :1;
			uint16_t Motor1_RunSta        :1;
			uint16_t Motor2_RunSta        :1;
			uint16_t Motor3_RunSta        :1;
			uint16_t Motor1_EnabledSta    :1;
			uint16_t Motor2_EnabledSta    :1;
			uint16_t Motor3_EnabledSta    :1;
			uint16_t AS5600_1_InitSta     :1;
			uint16_t AS5600_2_InitSta     :1;
			uint16_t AS5600_3_InitSta     :1;
			uint16_t MotorCtrl_flg        :1; //�������ģʽ�µ�����Ʊ��
			uint16_t W25QxxErr_flg        :1;
			uint16_t retain               :1;
		}bit;
	}UploadData_U;
	float MotorAngle_Target[3]; //�Ƕȿ���ֵ
	float SCARA_Cartesian[3];//��е������ϵ
}RobotArmData_Typedef;
typedef struct 
{
	uint8_t  mark;
	int16_t stcAcc[3];  //���ٶ�
	int16_t stcGyro[3]; //���ٶ�
	int16_t stcAngle[3];//�Ƕ�
	int16_t  stcTemp;
	float temp;
	float ax;
	float ay;
	float az;
	float wx;
	float wy;
	float wz;
	float Roll;
	float Pitch;
	float Yaw;
}GyroData_Typedef;
typedef struct
{
	uint8_t sta;
	/* ���� */
    uint8_t sw;
	/* ҡ�� */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
	/* ң��ȡֵ��Χ ��ת���� */
	__IO float RC_Resolution;
	/* ��̬ */
	__IO struct
    {
        float vx;
        float vy;
        float vyaw;
		float vpitch;
    } sPosition;
}_s_RemoteCtrl_Info;
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern FlashData_Typedef FlashData_Struct;
extern AppData_Typedef   AppData_Struct;
/*******ֱ���������ģ����������*******/
#ifdef __SLAVE_DCMotorMiniwatt_H__
extern DCMotorMiniwattDef_t DCMotorMiniwatt_S[4];
#endif
/*******�������ģ����������*******/
#ifdef __SLAVE_SteeringEngine_6CH_H__
extern SteeringEngine6CHDef_t Servo_S;
extern uint16_t AngleValue[6];
extern uint16_t AngleTime[6];
#endif
/*******Ѳ����ģ����������*******/
#ifdef __SLAVE_Tracking_H__
extern TrackingDef_t Tracking_Device[4];
#endif
/*******������ģ����������*******/
#ifdef __SLAVE_UltrasonicRanging_H__
extern UltrasonicRangingDef_t UltrasonicRanging_S;
#endif
/*******��������������*******/
extern GyroData_Typedef GyroData_Struct;
/*******ң��������������*******/
extern _s_RemoteCtrl_Info sRemoteCtrl_Info;
/*******��е��������������*******/
extern RobotArmData_Typedef RobotArmData_Struct;
/*******ʶ��ģ����������*******/
extern RecognitionModule_s RecognitionModule_t;
extern uint8_t Recognition_Buffer[Recognition_RX_LEN];
extern uint8_t bool_recognitionflag;
/*******ש��������*******/
extern BrickData_TypeDef BrickData_Struct;
/* Exported functions --------------------------------------------------------*/
int AppData_Init(void);
uint8_t Application_DataFlash_Modification(void);




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
