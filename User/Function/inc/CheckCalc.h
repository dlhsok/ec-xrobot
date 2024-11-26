#ifndef _CHECKCALC_H_
#define _CHECKCALC_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* 包含头文件 ----------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
/* 宏定义 --------------------------------------------------------------------*/

/* 类型定义 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
uint8_t checksum(uint8_t *qdata,uint8_t len);
uint16_t CRC16Modbus_Compute(uint8_t *puchMsg, uint16_t usDataLen);








#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
