#ifndef __SLAVE_DCMotorMiniwatt_H__
#define __SLAVE_DCMotorMiniwatt_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"
/* 宏定义 --------------------------------------------------------------------*/
#define SLAVE_DCMotorMiniwatt_ENABLED   1
#define MotorPlate_Addr                 2  //电机控制板地址
#define DCMOTOR_NEW   //新驱动，正交编码
/* 类型定义 ------------------------------------------------------------------*/
typedef struct DCMotorMiniwatt_T
{    
    uint8_t  CAN_Number;
#ifdef DCMOTOR_NEW
    union{
		#define PG_RXMOTORDATA_LEN      8
		uint8_t Datum[PG_RXMOTORDATA_LEN];
		struct{
			uint16_t SetSpeed;
			int16_t  SetPwm;
			int16_t  SetCurr;
			int16_t  SetLocation;
		}DATE;
	}Motor_ControlData;
	union{
		#define PG_TXMOTORDATA_LEN   8
	    uint8_t Datum[PG_TXMOTORDATA_LEN];
        struct{
            uint16_t Speed                      ; //Motor Speed
            uint16_t EncoderLineNum             ; //Motor EncoderLineNum
            uint16_t Coded_Value_H              ; //Coded_Value_H
            uint16_t Coded_Value_L              ; //Coded_Value_L
        }DATE;
    }Motor_UploadData;
	union{
		#define PG_RXMOTORDATA1_LEN     8
		uint8_t Datum[PG_RXMOTORDATA1_LEN];
		struct{
			uint8_t retain[6];
			uint8_t id;
			uint8_t order;
		}Data_id;
		struct{
			int16_t pwm;
			uint8_t retain[4];
			uint8_t id;
			uint8_t order;
		}Data_pwm;
		struct{
			uint8_t retain[4];
			int16_t speed;
			uint8_t id;
			uint8_t order;
		}Data_speed;
		struct{
			int32_t location;
			int16_t speed;
			uint8_t id;
			uint8_t order;
		}Data_location;
		struct{
		union
		{
				uint16_t HalfWord;
				struct{
					uint16_t disable        :1;
					uint16_t brake          :1;
					uint16_t clear_stall    :1;
					uint16_t reserve        :13;
				}bit;
			}flag;
			uint8_t retain[4];
			uint8_t id;
			uint8_t order;
		}Data_Flag;
	}Motor_ControlData1;
	union 
	{
		#define PG_DCMOTORCTRL_DATA1_STE_ACK_LEN 8
		uint8_t Datum[PG_DCMOTORCTRL_DATA1_STE_ACK_LEN];
		struct{
			union{
				uint16_t HalfWord;
				struct{
					uint16_t disable        :1;
					uint16_t brake          :1;
					uint16_t stall          :1;
					uint16_t overload       :1;
					uint16_t reserve        :12;
				}bit;
			}state;
			int8_t reserve[PG_DCMOTORCTRL_DATA1_STE_ACK_LEN-2];
		}DATE; 
	}Motor_ControlData1_Ack;//电机数据设置响应
	
	uint32_t Coded_ValueOld;
    uint32_t Coded_ValueAccu;
#else	
    union 
    {
        #define PG_RXMOTOR_SERIAL_NUMBER_LEN 2
        uint8_t Datum[PG_RXMOTOR_SERIAL_NUMBER_LEN];
        struct{
            uint16_t Serial_Number      ; //serial number
        }DATE;
    }Motor_SerialNumber;//序号修改 
    union
    {
        #define PG_TXMOTORDATA_LEN 8
        uint8_t Datum[PG_TXMOTORDATA_LEN];
        struct
        {
            uint16_t Speed;
            uint16_t EncoderNum; //V1.0.3版本上才有此数据		
            uint16_t Coded_Value;
            uint16_t Fault;
        }DATE;
    }Motor_UploadData;//电机反馈数据 
    
    union
    {
        #define PG_RXMOTORDATA_LEN 8
        uint8_t Datum[PG_RXMOTORDATA_LEN];
        struct
        {
            uint16_t SetSpeed                   ; //Motor Set Speed
            int16_t  PwmSet                     ; //Motor Set PWM
            uint16_t Retain2                    ; //Retain2
            uint16_t Retain3                    ; //Retain3
        }DATE; 
    }Motor_ControlData;//电机控制数据
	
	uint16_t Coded_ValueOld;
    uint32_t Coded_ValueAccu;
#endif
    struct DCMotorMiniwatt_T* next;
} DCMotorMiniwattDef_t;

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
uint8_t SLAVE_DCMotorMiniwatt_Init(DCMotorMiniwattDef_t* handle,uint8_t list);     //数据初始化
void SLAVE_DCMotorMiniwatt_NumberSet(DCMotorMiniwattDef_t* handle,uint8_t number); //序列号设置（目前修改后不能直接连接，须修改程序，重新上电）
int16_t SLAVE_DCMotorMiniwatt_SpeedRead(DCMotorMiniwattDef_t* handle);             //速度读取
void SLAVE_DCMotorMiniwatt_SpeedSet(DCMotorMiniwattDef_t* handle,int16_t val);    //速度设置    
void SLAVE_DCMotorMiniwatt_PWMSet(DCMotorMiniwattDef_t* handle,int16_t val);       //PWM设置（速度反馈不能看方向，忽略符号）
#ifdef DCMOTOR_NEW
int32_t SLAVE_DCMotorMiniwatt_PlaceRead(DCMotorMiniwattDef_t* handle);            //编码值读取（目前不是正交编码信号）
void SLAVE_DCMotorMiniwatt_CurrSet(DCMotorMiniwattDef_t* handle,int16_t val);
void SLAVE_DCMotorMiniwatt_LocationSet(DCMotorMiniwattDef_t* handle,int16_t val);
void SLAVE_DCMotorMiniwatt_Digital_OFF(DCMotorMiniwattDef_t* handle);
void SLAVE_DCMotorMiniwatt_Digital_PWMSet(DCMotorMiniwattDef_t* handle,int16_t val);
void SLAVE_DCMotorMiniwatt_Digital_SpeedSet(DCMotorMiniwattDef_t* handle,int16_t val);
void SLAVE_DCMotorMiniwatt_Digital_LocationSet(DCMotorMiniwattDef_t* handle,int32_t val);
void SLAVE_DCMotorMiniwatt_Digital_Disable(DCMotorMiniwattDef_t* handle);
void SLAVE_DCMotorMiniwatt_Digital_Brake(DCMotorMiniwattDef_t* handle);
void SLAVE_DCMotorMiniwatt_Digital_ClrStall(DCMotorMiniwattDef_t* handle);
void SLAVE_DCMotorMiniwatt_Digital_Clr(DCMotorMiniwattDef_t* handle);
void SLAVE_DCMotorMiniwatt_Digital_SpeedPWMSet(DCMotorMiniwattDef_t* handle,int16_t val);
#else
uint16_t SLAVE_DCMotorMiniwatt_PlaceRead(DCMotorMiniwattDef_t* handle);            //编码值读取（目前不是正交编码信号）
#endif
void SLAVE_DCMotorMiniwatt_CANRXDispose(void *head,uint32_t Id);

#endif  // __SLAVE_DCMotorMiniwatt_H__

