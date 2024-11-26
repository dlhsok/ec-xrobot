/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_hal_ps2.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
const uint8_t Comd[2]={0x01,0x42};	//开始命令。请求数据
const uint16_t MASK[16] = {
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
};	//按键值与按键明
/* 扩展变量 ------------------------------------------------------------------*/
_u_PS2_Data PS2_Data={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
#if 0 
static void delay_us(int16_t us)
{
	for(int i=0; i<us*10; i++)
	{
			;
	}
}
#else
extern void HAL_usDelay(uint32_t nus);
#define delay_us(us)    HAL_usDelay(us+5)
#endif
//向手柄发送命令
static uint8_t PS2_Cmd(uint8_t CMD)
{
	volatile uint16_t ref=0x01;
	uint8_t Data;

	Data = 0;
	for(ref=0x01; ref<0x0100; ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H();                   //输出以为控制位
		}
		else DO_L();

		CLK_H();                        //时钟拉高
		delay_us(10);
		CLK_L();
		delay_us(10);
		CLK_H();
		if(DI())
		{
			Data = ref|Data;
		}
		delay_us(5);
	}
	
	return Data;
}
//short poll
static void PS2_ShortPoll(void)
{
	CS_L();
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H();
	delay_us(16);	
}

//进入配置
static void PS2_EnterConfing(void)
{
    CS_L();
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H();
	delay_us(16);
}

//发送模式设置
static void PS2_TurnOnAnalogMode(void)
{
	CS_L();
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  软件设置发送模式
	PS2_Cmd(0xEE); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
				   //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H();
	delay_us(16);
}
//振动设置
static void PS2_VibrationMode(void)
{
	CS_L();
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H();
	delay_us(16);	
}
//完成并保存配置
static void PS2_ExitConfing(void)
{
    CS_L();
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H();
	delay_us(16);
}
//清除数据缓冲区
void PS2_ClearData(void)
{
	PS2_Data.DATE.ID = 0;
	PS2_Data.DATE.verify = 0;
	PS2_Data.DATE.key1_U.byte = 0xff;
	PS2_Data.DATE.key2_U.byte = 0xff;
	PS2_Data.DATE.PSS_RX = 0;
	PS2_Data.DATE.PSS_RY = 0;
	PS2_Data.DATE.PSS_LX = 0;
	PS2_Data.DATE.PSS_LY = 0;
}
//判断是否为红灯模式
//返回值；0，红灯模式
//		  其他，其他模式
uint8_t PS2_RedLight(void)
{
	uint8_t Data;
	
	CS_L();
	PS2_Cmd(Comd[0]);  //开始命令
	Data = PS2_Cmd(Comd[1]);  //请求数据
	CS_H();
	PS2_Data.DATE.ID = Data;
	if( Data == 0X73) return 0 ;
	else return 1;
}

//读取手柄数据
void PS2_ReadData(void)
{
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;
	uint8_t data;

	CS_L();
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Data.DATE.ID = PS2_Cmd(Comd[1]);  //请求数据
	for(byte=1;byte<8;byte++)          //开始接受数据
	{
		data = 0;
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H();
			delay_us(10);
			CLK_L();
			delay_us(10);
			CLK_H();
		    if(DI())
			{
		       data = ref|data;
			}
			delay_us(5);
		} 
		PS2_Data.byte[byte] = data;
        delay_us(16);
	}
	CS_H();	
}

//对读出来的PS2的数据进行处理      
//只处理了按键部分         
//默认数据是红灯模式  只有一个按键按下时
//按下为0， 未按下为1
uint8_t PS2_DataKey(void)
{
	uint8_t index;
	uint16_t Handkey;

	PS2_ClearData();
	PS2_ReadData();
    if(PS2_Data.DATE.verify == 0x5a)
	{
		Handkey=(PS2_Data.byte[3]<<8)|PS2_Data.byte[2];     //这是16个按键  按下为0， 未按下为1
		for(index=0;index<16;index++)
		{	    
			if((Handkey&(1<<(MASK[index]-1)))==0)
			{
				return index+1;
			}
		}
	}
	return 0;          //没有任何按键按下
}

//得到一个摇杆的模拟量	 范围0~256
//返回1得取成功，0得取失败
uint8_t PS2_AnologData(_e_RockerSelect button,uint8_t *qdata)
{
	PS2_ClearData();
	PS2_ReadData();
	if(PS2_Data.DATE.verify == 0x5a)
	{
		if(button == PSS_RX) *qdata = PS2_Data.DATE.PSS_RX;
		else if(button == PSS_RY) *qdata = PS2_Data.DATE.PSS_RY;
		else if(button == PSS_LX) *qdata = PS2_Data.DATE.PSS_LX;
		else *qdata = PS2_Data.DATE.PSS_LY;
		
		return 1;
	}
	else
	{
		return 0;
	}
}
/******************************************************
Function:    void PS2_Vibration(uint8_t motor1, uint8_t motor2)
Description: 手柄震动函数，
Calls:		 void PS2_Cmd(uint8_t CMD);
Input: motor1:右侧小震动电机 0x00关，其他开
	   motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
******************************************************/
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	CS_L();
	delay_us(16);
    PS2_Cmd(0x01);  //开始命令
	PS2_Cmd(0x42);  //请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H();
	delay_us(16);  
}
//手柄配置初始化
void PS2_SetInit(void)
{
	CS_H();
	CLK_H();
	DO_H();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//进入配置模式
	PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择是否保存
	PS2_VibrationMode();	//开启震动模式
	PS2_ExitConfing();		//完成并保存配置
	CS_H();
	CLK_H();
	DO_H();
}




