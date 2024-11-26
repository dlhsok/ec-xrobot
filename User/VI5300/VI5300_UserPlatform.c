
#include "VI5300_UserPlatform.h"
#include "VI5300_API.h"
//#include "i2c.h"
#include "vi_sw_i2c.h"

#define VI5300_ECO_2V1 1

#ifdef VI5300_ECO_2V1

VI5300_Status I2C_2V1_WriteOneReg(uint8_t addr, uint8_t value)
{
		VI5300_Status ret;
		ret = vi_sw_writereg(gSalve,addr,value);
		if(ret == 1)
		ret = VI5300_OK;
		else if(ret == 0)
		ret = VI5300_ERROR;
		
		return ret;
}

VI5300_Status I2C_2V1_ReadOneReg(uint8_t addr, uint8_t *value)
{
	VI5300_Status ret;
	ret = vi_sw_readreg(gSalve,addr,value,1);
	if(ret == 1)
	ret = VI5300_OK;
	else if(ret == 0)
	ret = VI5300_ERROR;

	return ret;
}

VI5300_Status I2C_WriteXByteWraper(uint8_t startaddr, uint8_t *buf, uint8_t len)
{
    VI5300_Status ret = VI5300_OK;
    for(uint8_t i = 0; i < len; i++)
    {
        ret = I2C_2V1_WriteOneReg(startaddr + i, buf[i]);
        VI5300_CHECK_RET(ret);
    }

    return ret;
}

VI5300_Status I2C_ReadXByteWraper(uint8_t startaddr, uint8_t *buf, uint8_t len)
{
    VI5300_Status ret = VI5300_OK;
    for(uint8_t i = 0; i < len; i++)
    {
        ret = I2C_2V1_ReadOneReg(startaddr + i, &buf[i]);  
        VI5300_CHECK_RET(ret);
    }

    return ret;
}

#endif

VI5300_Status WriteOneReg(uint8_t addr, uint8_t value)
{
    return I2C_WriteXBytes(addr, &value, 1);
}

VI5300_Status ReadOneReg(uint8_t addr, uint8_t *value)
{
    return I2C_ReadXBytes(addr, value, 1);
}

VI5300_Status WriteCommand(uint8_t cmd)
{
    return WriteOneReg(VI5300_REG_CMD, cmd);
}

VI5300_Status I2C_WriteXBytes(uint8_t startaddr, uint8_t *buf, uint8_t len)
{
    return I2C_WriteXByteWraper(startaddr, buf, len);
}

VI5300_Status I2C_ReadXBytes(uint8_t startaddr, uint8_t *buf, uint8_t len)
{
    return I2C_ReadXByteWraper(startaddr, buf, len);
}






