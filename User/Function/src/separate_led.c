/* 包含头文件 ----------------------------------------------------------------*/
#include "separate_led.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
static struct LED_T* head_handle = NULL;
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
void SingleLed_handler(LED_T* handle)
{
	if(handle->TIME > 0)
	{
		handle->TIME--;
	}
	if(handle->TIME == 0)
	{
		switch(handle->state)
		{
			case 0:
				if(handle->LightON_Time)
				{
					handle->level_now = handle->lighten_level;
					handle->TIME = handle->LightON_Time;
					handle->state = 1;
				}
				break;
			case 1:
				if(handle->LightOFF_Time)
				{
					if(handle->lighten_level) handle->level_now = 0;
					else handle->level_now = 1;
					handle->TIME = handle->LightOFF_Time;
					handle->state = 2;
				}
				break;
			case 2:
				if(handle->Interval_Time != 0)
				{
					if(handle->Flicker_Num != 0)
					{
						handle->Flicker_NumCount++;
						if(handle->Flicker_Num == handle->Flicker_NumCount)
						{
							handle->TIME = handle->Interval_Time;
							handle->state = 3;
						}
					}
				}
				if(handle->state == 2) 
					handle->state = 0;
				break;
			case 3:
				handle->Flicker_NumCount = 0;
				handle->state = 0;
				break;
		}
	}
    if(handle->level_old != handle->level_now)
    {
        handle->separate_led_Level(handle->level_now);
        handle->level_old = handle->level_now;
    }
}
void SingleLed_init(LED_T* handle, 
                    void(*pin_level)(uint8_t),
                    uint8_t active_level)
{
    memset(handle, 0, sizeof(LED_T));
	handle->separate_led_Level = pin_level;
    handle->lighten_level = active_level;
    handle->level_now = !handle->lighten_level;
    handle->separate_led_Level(handle->level_now);
    handle->level_old = handle->level_now;
}
void SingleLed_time(LED_T* handle, 
                    uint8_t Flicker_Num,
                    uint16_t LightOFF_Time,     //灯灭的时间
					uint16_t LightON_Time,      //灯亮的时间
					uint16_t Interval_Time)     //间隔时间    
{
    handle->Flicker_Num = Flicker_Num;
    handle->LightON_Time = LightON_Time/TICKS_INTERVAL;
    handle->LightOFF_Time = LightOFF_Time/TICKS_INTERVAL;
    handle->Interval_Time = Interval_Time/TICKS_INTERVAL;
	handle->TIME = 0;
	handle->Flicker_NumCount = 0;
}
int SingleLed_start(LED_T* handle)
{
	LED_T* target = head_handle;
    handle->TIME = 0;
	while(target) 
    {
		if(target == handle) 
            return -1;	//already exist.
		target = target->next;
	}
	handle->next = head_handle;
	head_handle = handle;
	return 0;
}
void SingleLed_stop(LED_T* handle)
{
	LED_T** curr;
	LED_T* entry;
	curr = &head_handle;
	while(*curr)
	{
		entry = *curr;
		if(handle == entry)
		{
            handle->separate_led_Level(!handle->lighten_level);
			*curr = entry->next;
			break;
		}
		else
		{
			curr = &entry->next;
		}
	}
}
void SingleLed_ticks(void)
{
	LED_T* target;
	for(target=head_handle; target; target=target->next) 
    {
		SingleLed_handler(target);
	}
}


