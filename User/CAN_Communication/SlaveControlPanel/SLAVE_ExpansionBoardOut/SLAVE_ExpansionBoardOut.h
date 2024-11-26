#ifndef __SLAVE_ExpansionBoardOut_H__
#define __SLAVE_ExpansionBoardOut_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct ExpansionBoardOut_T
{    
    uint8_t  CAN_Number;
    
    union  
    {
        #define PG_RXExpansionBoardOut_SERIAL_NUMBER_LEN 1
        uint8_t Datum[PG_RXExpansionBoardOut_SERIAL_NUMBER_LEN];
        struct{
            uint8_t Serial_Number      ; //serial number
        }DATE;
    }ExpansionBoardOut_SerialNumber;//序号修改 
    
    union 
    {  
        #define PG_RXSTEExpansionOut_LEN 1
        uint8_t Datum[PG_RXSTEExpansionOut_LEN];
        struct{
            
            uint8_t  out1 :1;
            uint8_t  out2 :1;
            uint8_t  out3 :1;
            uint8_t  out4 :1;
			uint8_t  out5 :1;
            uint8_t  out6 :1;
            uint8_t  reserved :1;
            uint8_t  MemoryFlg :1;
        }DATE;
        
    }ExpansionBoardOut_ControlData;
    
    struct ExpansionBoardOut_T* next;
} ExpansionBoardOutDef_t;
/* 宏定义 --------------------------------------------------------------------*/
#define SLAVE_ExpansionBoardOut_ENABLED 1
#define ExpansionBoardOut_Addr     7  //地址

#define ExpansionBoardOut_Out_1  0
#define ExpansionBoardOut_Out_2  1
#define ExpansionBoardOut_Out_3  2
#define ExpansionBoardOut_Out_4  3
#define ExpansionBoardOut_Out_5  4
#define ExpansionBoardOut_Out_6  5

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
uint8_t SLAVE_ExpansionBoardOut_Init(ExpansionBoardOutDef_t* handle,uint8_t list);
void SLAVE_ExpansionBoardOut_NumberSet(ExpansionBoardOutDef_t* handle,uint8_t number);
void SLAVE_ExpansionBoardOut_OutControl(ExpansionBoardOutDef_t* handle,
                                        uint8_t out1,
                                        uint8_t out2,
                                        uint8_t out3,
                                        uint8_t out4,
										uint8_t out5,
                                        uint8_t out6
                                        );
void SLAVE_ExpansionBoardOut_MovementMemory(ExpansionBoardOutDef_t* handle);



#endif  // __SLAVE_SLAVE_ExpansionBoardOut_H__

