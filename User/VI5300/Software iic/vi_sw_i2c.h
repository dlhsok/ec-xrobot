#ifdef USE_SOFTWARE_IIC

#ifndef _BSP_SW_I2C_H
#define _BSP_SW_I2C_H

#include <inttypes.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_gpio.h"

#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */

void i2c_sw_gpio_config(void);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(uint8_t ack);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);


extern uint8_t vi_sw_writereg(uint8_t slave, uint8_t reg_add,uint8_t reg_dat);
extern uint8_t vi_sw_readreg(uint8_t slave, uint8_t reg_add,uint8_t *buf,uint8_t num);
extern uint8_t vi_iic_readRegs(uint8_t slave, uint8_t *data, uint16_t Length);
extern uint8_t vi_iic_writeRegs(uint8_t slave, uint8_t* data, uint8_t Len);
#endif
#endif
