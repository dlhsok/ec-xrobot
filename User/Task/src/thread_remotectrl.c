/* Includes ------------------------------------------------------------------*/
#include "thread_remotectrl.h"
#include "bsp.h"
#include "data.h"
/* Private macros ------------------------------------------------------------*/
// ң��ȡֵ��Χ ��ת����
#define PS2_RESOLUTION       (128.0f)
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* �����߳̿��ƿ�ָ�� */
rt_thread_t thread_remotectrl = RT_NULL;
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/***********************************************************************************************

                                       Ӧ�ú���

************************************************************************************************/
uint8_t PS2RC_ReadData( int16_t *ch1,
				        int16_t *ch2,
				        int16_t *ch3,
				        int16_t *ch4,
				        uint8_t *sw )
{
	PS2_ReadData();
	if(PS2_Data.DATE.ID == 0X73)
	{
		LED2_Toggle();
		*ch1 = PS2_Data.DATE.PSS_RX - 128;
		*ch2 = 128 - PS2_Data.DATE.PSS_RY;
		*ch3 = PS2_Data.DATE.PSS_LX - 128;
		*ch4 = PS2_Data.DATE.PSS_LY - 128;
		*sw = 0;
		if(PS2_Data.DATE.key2_U.bit.LEFT1 == 0)
			*sw |= 0x01;
		if(PS2_Data.DATE.key2_U.bit.RIGHT1 == 0)
			*sw |= 0x02;
		if(PS2_Data.DATE.key2_U.bit.LEFT2 == 0)
			*sw |= 0x04;
		if(PS2_Data.DATE.key2_U.bit.RIGHT2 == 0)
			*sw |= 0x08;
		if(PS2_Data.DATE.key1_U.bit.UP == 0)
		    *sw |= 0x10;
		if(PS2_Data.DATE.key1_U.bit.DOWN == 0)
		    *sw |= 0x20;
		if(PS2_Data.DATE.key2_U.bit.TRI == 0)
		    *sw |= 0x40;
		if(PS2_Data.DATE.key2_U.bit.FORK == 0)
		    *sw |= 0x80;
		//������ݻ�����
		PS2_ClearData();
		
		return 0;
	}
	else
	{//�ж��ֱ����Ǻ��ģʽ��ָʾ��LEDϨ��
		PS2_ClearData();
		LED2(LED_OFF);
		
		return 1;
	}
}
/***********************************************************************************************

                                       Ӧ��������

************************************************************************************************/
void remotectrl_task(void *pvParameters)
{
	uint8_t err = 0;
	
	while(1)
    {
		sRemoteCtrl_Info.RC_Resolution = PS2_RESOLUTION;
		err = PS2RC_ReadData( &sRemoteCtrl_Info.ch1,
		                      &sRemoteCtrl_Info.ch2,
		                      &sRemoteCtrl_Info.ch3,
		                      &sRemoteCtrl_Info.ch4,
		                      &sRemoteCtrl_Info.sw );
		//ң��ֵת������������
		if(err)
		{
			sRemoteCtrl_Info.ch1 = 0;
			sRemoteCtrl_Info.ch2 = 0;
			sRemoteCtrl_Info.ch3 = 0;
			sRemoteCtrl_Info.ch4 = 0;
			sRemoteCtrl_Info.sta = 0;
		}
		else
		{
			sRemoteCtrl_Info.sta = 1;
		}
		sRemoteCtrl_Info.sPosition.vy = (float)(-sRemoteCtrl_Info.ch4 * RC_CHASSIS_MAX_SPEED_Y / sRemoteCtrl_Info.RC_Resolution);
		sRemoteCtrl_Info.sPosition.vx = (float)(sRemoteCtrl_Info.ch3 * RC_CHASSIS_MAX_SPEED_X / sRemoteCtrl_Info.RC_Resolution);
		sRemoteCtrl_Info.sPosition.vyaw = (float)(-sRemoteCtrl_Info.ch1 * RC_GIMBAL_MOVE_RATIO_YAW);
		My_mDelay(49);
    }
}
/***********************************************************************************************

                                       Ӧ��������

************************************************************************************************/
int Task_RemoteCtrl_create(void)
{
	/**********************PS2ң�س�ʼ��***********************/
	PS2_SetInit();
	/**********************��������***************************/
    thread_remotectrl = rt_thread_create("remotectrl",      /* �߳����� */
								          remotectrl_task,  /* �߳���ں��� */
								          RT_NULL,          /* �߳���ں������� */
								          512,              /* �߳�ջ��С */
								          4,                /* �̵߳����ȼ� */
								          20);              /* �߳�ʱ��Ƭ */
	if(thread_remotectrl != RT_NULL)
	{
		rt_thread_startup(thread_remotectrl);
		rt_kprintf("thread_remotectrl startup!\n");
	}
	
	return 0;
}
INIT_APP_EXPORT(Task_RemoteCtrl_create);



















