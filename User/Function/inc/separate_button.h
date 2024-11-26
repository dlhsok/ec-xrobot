#ifndef _Separate_BUTTON_H_
#define _Separate_BUTTON_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#define KEY_ExplainPlan          0
#define KEY_CallBackFunc_ENABLE  1
#define MYBTN_TIME_ENABLE        1
//According to your need to modify the constants.
#define KEYTICKS_INTERVAL        10	//ms
#define DEBOUNCE_TICKS           3	//MAX 7

#define SINGLE_CLICK_ENABLE                1
#define DOUBLE_CLICK_ENABLE                1
#define MUCH_CLICK_ENABLE                  0
#define SIGNAL_LONG_PRESS_HOLD_ENABLE      1
#define PRESS_UP_DELAY_ENABLE              1

#define SIGNAL_PRESS_DOWN		 (1<<PRESS_DOWN)
#define SIGNAL_PRESS_UP			 (1<<PRESS_UP)
#if SINGLE_CLICK_ENABLE
#define SIGNAL_SINGLE_CLICK	     (1<<SINGLE_CLICK)
#endif
#if DOUBLE_CLICK_ENABLE
#define SIGNAL_DOUBLE_CLICK	     (1<<DOUBLE_CLICK)
#endif
#if MUCH_CLICK_ENABLE
#define SIGNAL_MUCH_CLICK        (1<<MUCH_CLICK)
#endif
#if SIGNAL_LONG_PRESS_HOLD_ENABLE
#define SIGNAL_LONG_PRESS_HOLD	 (1<<LONG_PRESS_HOLD)
#endif
#if PRESS_UP_DELAY_ENABLE
#define SIGNAL_PRESS_UP_DELAY	 (1<<PRESS_UP_DELAY)
#endif
/* Exported types ------------------------------------------------------------*/
#if KEY_CallBackFunc_ENABLE
typedef void (*BtnCallback)(void*);
#endif
typedef enum 
{
    PRESS_DOWN = 0,//按下
    PRESS_UP,      //松开
#if SINGLE_CLICK_ENABLE
    SINGLE_CLICK,  //单击
#endif
#if DOUBLE_CLICK_ENABLE
    DOUBLE_CLICK,  //双击
#endif
#if MUCH_CLICK_ENABLE
    MUCH_CLICK,    //多击
#endif
#if SIGNAL_LONG_PRESS_HOLD_ENABLE
    LONG_PRESS_HOLD,//长按保持
#endif
#if PRESS_UP_DELAY_ENABLE
    PRESS_UP_DELAY, //松开延时
#endif
	number_of_event,
	NONE_PRESS
}PressEvent;
typedef struct KEY_T
{
	uint8_t  debounce_cnt :3;//防反跳计数
	uint8_t  button_level :1;//上一次按钮状态
	uint8_t  active_level :1;//按下的状态
	uint8_t  ReadData     :1;
	uint8_t  Trg          :1;
	uint8_t  Cont         :1;
	uint8_t  signal;
#if KEY_CallBackFunc_ENABLE
    uint8_t  event_flg;
#endif
	uint8_t  state_old;
#if SINGLE_CLICK_ENABLE | DOUBLE_CLICK_ENABLE | MUCH_CLICK_ENABLE
	uint8_t  RepeatNum;	
    uint8_t  RepeatNumOld;	
    uint16_t RepeatCount;	/* 连续按键计数器 */
	uint16_t RepeatSpeed;	/* 连续按键周期 */
#endif
#if SIGNAL_LONG_PRESS_HOLD_ENABLE
	uint16_t LongCount;		/* 长按计数器 */
	uint16_t LongTime;		/* 按键按下持续时间, 0表示不检测长按 */
#endif
	uint8_t  (*separate_button_Level)(void); /* 按键按下的判断函数*/
#if KEY_CallBackFunc_ENABLE
	BtnCallback  cb[number_of_event];   //事件函数
#endif
#if ( KEY_ExplainPlan == 0 )
	struct KEY_T* next;
#endif
}KEY_T;
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void mybtn_init(KEY_T* handle, uint8_t(*pin_level)(void),uint8_t active_level,uint16_t RepeatSpeed,uint16_t LongTime);
#if KEY_CallBackFunc_ENABLE
void mybtn_attach(KEY_T* handle, PressEvent event, BtnCallback cb);
#endif
void mybtn_time(KEY_T* handle,uint16_t RepeatSpeed,uint16_t LongTime);
void mybtn_handler(KEY_T* handle);
#if ( KEY_ExplainPlan == 0 )
int  mybtn_start(KEY_T* handle);
void mybtn_stop(KEY_T* handle);
void mybtn_ticks(void);
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
