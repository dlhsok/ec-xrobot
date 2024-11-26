/* Includes ------------------------------------------------------------------*/
#include "separate_button.h"
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if ( KEY_ExplainPlan == 0 )
static struct KEY_T* head_handle = NULL;
#endif
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void mybtn_handler(KEY_T* handle)
{	
	uint8_t read_gpio_level;
#if KEY_CallBackFunc_ENABLE
	uint8_t i,mask;
#endif
	
	read_gpio_level = handle->separate_button_Level();
	/*------------button debounce handle---------------*/
	if(read_gpio_level != handle->button_level) //not equal to prev one
	{ 
		//continue read 3 times same new level change
		if(++(handle->debounce_cnt) >= DEBOUNCE_TICKS) {
			handle->button_level = read_gpio_level;
			handle->debounce_cnt = 0;
		}
	} 
	else //leved not change ,counter reset.
	{ 
		handle->debounce_cnt = 0;
	}
	/*-----------------State machine-------------------*/
	handle->ReadData = (handle->button_level == handle->active_level);
	handle->Trg = handle->ReadData & (handle->ReadData ^ handle->Cont);
	handle->Cont = handle->ReadData; 
	handle->signal = 0;
	if(handle->ReadData)
	{
		if(handle->Trg)
		{
			/************按下事件*******************/
		    handle->signal |= SIGNAL_PRESS_DOWN;
		#if SIGNAL_LONG_PRESS_HOLD_ENABLE
			/************长按事件*******************/
			handle->LongCount = 0;
		#endif
		#if SINGLE_CLICK_ENABLE | DOUBLE_CLICK_ENABLE | MUCH_CLICK_ENABLE
			/************多按事件*******************/
			handle->RepeatNum += 1;
			handle->RepeatCount = 0;
		#endif
			handle->state_old = 3;
		}
		else
		{
		#if SIGNAL_LONG_PRESS_HOLD_ENABLE
			/************长按事件*******************/
		    handle->LongCount++;
			if(handle->LongTime != 0)
			{
				if(handle->LongCount >= handle->LongTime)
				{
					handle->signal |= SIGNAL_LONG_PRESS_HOLD;
				    handle->LongCount = 0;	
				}
			}
		#endif
			handle->state_old = 2;
		}
	}
	else
	{
		/************松开事件*******************/
		if(handle->state_old > 0)
		{
			handle->signal |= SIGNAL_PRESS_UP; 	
		}
	#if SINGLE_CLICK_ENABLE | DOUBLE_CLICK_ENABLE | MUCH_CLICK_ENABLE
		/************多按事件*******************/
		if(handle->RepeatCount < handle->RepeatSpeed)
		{
			handle->RepeatCount++;
		}
		else if(handle->RepeatNum != 0)
		{
		#if PRESS_UP_DELAY_ENABLE
			handle->signal |= SIGNAL_PRESS_UP_DELAY;
		#endif
		#if SINGLE_CLICK_ENABLE
			if(1 == handle->RepeatNum)
			{
				handle->signal |= SIGNAL_SINGLE_CLICK; 	
			}
			else
		#endif
		#if DOUBLE_CLICK_ENABLE
			if(2 == handle->RepeatNum)
			{
				handle->signal |= SIGNAL_DOUBLE_CLICK;
			}
			else
		#endif
			{
	    #if MUCH_CLICK_ENABLE
				handle->RepeatNumOld = handle->RepeatNum;
				handle->signal |= SIGNAL_MUCH_CLICK;
		#endif
			}
			handle->RepeatNum = 0;
		}
	#endif
		handle->state_old = 0;
	}
#if KEY_CallBackFunc_ENABLE
	if( handle->signal )
	{
		mask = 1;
		for (i=0; i<number_of_event; i++)
		{
			if( handle->signal & mask )
            {
				if( handle->cb[i] != 0 )
				{
                    handle->event_flg = mask;
                    handle->cb[i]( handle );
				}
                handle->signal &= ~mask;
            }
            mask <<= 1;
		}
	}
#endif
}
void mybtn_init(KEY_T* handle, uint8_t(*pin_level)(void),uint8_t active_level,uint16_t RepeatSpeed,uint16_t LongTime)
{
    memset(handle, 0, sizeof(KEY_T));
	handle->separate_button_Level = pin_level;
	handle->button_level = handle->separate_button_Level();
	handle->active_level = active_level;
#if SINGLE_CLICK_ENABLE | DOUBLE_CLICK_ENABLE | MUCH_CLICK_ENABLE
	handle->RepeatSpeed = RepeatSpeed/KEYTICKS_INTERVAL;
#else
	RepeatSpeed = RepeatSpeed;
#endif
#if SIGNAL_LONG_PRESS_HOLD_ENABLE
	handle->LongTime = LongTime/KEYTICKS_INTERVAL;
#else
	LongTime = LongTime;
#endif
}
#if KEY_CallBackFunc_ENABLE
void mybtn_attach(KEY_T* handle, PressEvent event, BtnCallback cb)
{
	handle->cb[event] = cb;
}
#endif
#if MYBTN_TIME_ENABLE
void mybtn_time(KEY_T* handle,uint16_t RepeatSpeed,uint16_t LongTime)
{
	handle->separate_button_Level();
#if SINGLE_CLICK_ENABLE | DOUBLE_CLICK_ENABLE | MUCH_CLICK_ENABLE
    handle->RepeatSpeed = RepeatSpeed/KEYTICKS_INTERVAL;
#else
	RepeatSpeed = RepeatSpeed;
#endif
#if SIGNAL_LONG_PRESS_HOLD_ENABLE
	handle->LongTime = LongTime/KEYTICKS_INTERVAL;
#else
	LongTime = LongTime;
#endif
}
#endif
#if ( KEY_ExplainPlan == 0 )
int mybtn_start(KEY_T* handle)
{
	KEY_T* target = head_handle;
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
void mybtn_stop(KEY_T* handle)
{
	KEY_T** curr;
	KEY_T* entry;
	curr = &head_handle;
	while(*curr)
	{
		entry = *curr;
		if(handle == entry)
		{
			*curr = entry->next;
			break;
		}
		else
		{
			curr = &entry->next;
		}
	}
}
void mybtn_ticks(void)
{
	KEY_T* target;
	for(target=head_handle; target; target=target->next) 
    {
		mybtn_handler(target);
	}
}
#endif







