/* Includes ------------------------------------------------------------------*/
#include "thread_remotectrl.h"
#include "bsp.h"
#include "data.h"
/* Private macros ------------------------------------------------------------*/
// 遥控取值范围 做转化用
#define PS2_RESOLUTION       (128.0f)
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 定义线程控制块指针 */
rt_thread_t thread_remotectrl = RT_NULL;
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/***********************************************************************************************

                                       应用函数

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
		//清除数据缓冲区
		PS2_ClearData();
		
		return 0;
	}
	else
	{//判断手柄不是红灯模式，指示灯LED熄灭
		PS2_ClearData();
		LED2(LED_OFF);
		
		return 1;
	}
}
/***********************************************************************************************

                                       应用任务函数

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
		//遥控值转换到底盘数据
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

                                       应用主函数

************************************************************************************************/
int Task_RemoteCtrl_create(void)
{
	/**********************PS2遥控初始化***********************/
	PS2_SetInit();
	/**********************创建任务***************************/
    thread_remotectrl = rt_thread_create("remotectrl",      /* 线程名字 */
								          remotectrl_task,  /* 线程入口函数 */
								          RT_NULL,          /* 线程入口函数参数 */
								          512,              /* 线程栈大小 */
								          4,                /* 线程的优先级 */
								          20);              /* 线程时间片 */
	if(thread_remotectrl != RT_NULL)
	{
		rt_thread_startup(thread_remotectrl);
		rt_kprintf("thread_remotectrl startup!\n");
	}
	
	return 0;
}
INIT_APP_EXPORT(Task_RemoteCtrl_create);



















