#ifndef __VI5300_USERPLATFORM_H__
#define __VI5300_USERPLATFORM_H__
#include "VI5300_API.h"
#include "VI5300_Config.h"


VI5300_Status I2C_WriteXBytes(uint8_t startaddr, uint8_t *buf, uint8_t len);
VI5300_Status I2C_ReadXBytes(uint8_t startaddr, uint8_t *buf, uint8_t len);
VI5300_Status WriteCommand(uint8_t cmd);
VI5300_Status I2C_2V1_WriteOneReg(uint8_t addr, uint8_t value);
VI5300_Status I2C_2V1_ReadOneReg(uint8_t addr, uint8_t *value);
VI5300_Status WriteOneReg(uint8_t addr, uint8_t value);
VI5300_Status ReadOneReg(uint8_t addr, uint8_t *value);

#endif





