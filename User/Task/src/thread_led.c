/* Includes ------------------------------------------------------------------*/
#include "thread_led.h"
#include "bsp.h"
#include "separate_led.h"
/* Private macros ------------------------------------------------------------*/
#define DISPLAY_NUM        3 //数码管数目
#define Battery_Max        (25.2f)//25.7
#define Battery_Min        (22.2f)//22.6
#define Battery_Unit       (0.25f) //( (float)( ( Battery_Max - Battery_Min ) / 12.0f ) )
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifndef RT_USING_IDLE_HOOK
/* 定义线程控制块指针 */
rt_thread_t thread_led = RT_NULL;
#endif
//LED_T led1_struct;
//LED_T led2_struct;
//LED_T led3_struct;
uint8_t  Display_Buffer[DISPLAY_NUM] = {0x7F, 0x03, 0x07};
float battery_level;
uint8_t battery_state;
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#ifndef RT_USING_IDLE_HOOK
void led_task(void *pvParameters)
#else
void vApplicationIdleHook( void )
#endif
{
  while(1)
  {
    battery_level = ( BatVoltage_Read() - Battery_Min ) / Battery_Unit;
    if(battery_level > 12)      battery_level = 12;
    else if(battery_level < 0)  battery_level = 0;
    battery_state = battery_level;
    switch(battery_state)
    {
    case 12:
    case 11:
      Display_Buffer[0] = 0x7F;
      Display_Buffer[1] = 0x03;
      Display_Buffer[2] = 0x07;
      break;
    case 10:
      Display_Buffer[0] = 0x7E;
      Display_Buffer[1] = 0x03;
      Display_Buffer[2] = 0x07;
      break;
    case 9:
      Display_Buffer[0] = 0x7C;
      Display_Buffer[1] = 0x03;
      Display_Buffer[2] = 0x07;
      break;
    case 8:
      Display_Buffer[0] = 0x78;
      Display_Buffer[1] = 0x03;
      Display_Buffer[2] = 0x07;
      break;
    case 7:
      Display_Buffer[0] = 0x70;
      Display_Buffer[1] = 0x03;
      Display_Buffer[2] = 0x07;
      break;
    case 6:
      Display_Buffer[0] = 0x60;
      Display_Buffer[1] = 0x03;
      Display_Buffer[2] = 0x07;
      break;
    case 5:
      Display_Buffer[0] = 0x40;
      Display_Buffer[1] = 0x03;
      Display_Buffer[2] = 0x07;
      break;
    case 4:
      Display_Buffer[0] = 0x00;
      Display_Buffer[1] = 0x03;
      Display_Buffer[2] = 0x07;
      break;
    case 3:
      Display_Buffer[0] = 0x00;
      Display_Buffer[1] = 0x02;
      Display_Buffer[2] = 0x07;
      break;
    case 2:
      Display_Buffer[0] = 0x00;
      Display_Buffer[1] = 0x00;
      Display_Buffer[2] = 0x07;
      break;
    case 1:
      Display_Buffer[0] = 0x00;
      Display_Buffer[1] = 0x00;
      Display_Buffer[2] = 0x06;
      break;
    case 0:
      Display_Buffer[0] = 0x00;
      Display_Buffer[1] = 0x00;
      Display_Buffer[2] = 0x04;
      break;
    }

    rt_enter_critical();
    bsp_tm1652_SendData(Display_Buffer, DISPLAY_NUM);
    rt_exit_critical();
    HAL_xmsDelay( 4 );
    rt_enter_critical();
    bsp_tm1652_SendComm();
    rt_exit_critical();
    HAL_xmsDelay( 500 );
  }
}
int Task_LED_create(void)
{
#ifndef RT_USING_IDLE_HOOK
  thread_led = rt_thread_create("led",              /* 线程名字 */
                                led_task,          /* 线程入口函数 */
                                RT_NULL,           /* 线程入口函数参数 */
                                128,               /* 线程栈大小 */
                                30,                /* 线程的优先级 */
                                20);               /* 线程时间片 */
  if(thread_led != RT_NULL)
  {
    rt_thread_startup(thread_led);
    rt_kprintf("thread_led startup!\n");
  }
#else
  if( rt_thread_idle_sethook(vApplicationIdleHook) == RT_EOK )
  {
    rt_kprintf("thread_led startup!\n");
  }
#endif
  return 0;
}
INIT_APP_EXPORT(Task_LED_create);
