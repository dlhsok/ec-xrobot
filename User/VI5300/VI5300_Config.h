#ifndef __VI5300_CONFIG_H__
#define __VI5300_CONFIG_H__

#include "VI5300_Config.h"

typedef unsigned long long uint64_t;

typedef unsigned int uint32_t;

typedef int int32_t;

typedef unsigned short uint16_t;

typedef short int16_t;

typedef unsigned char uint8_t;

typedef signed char int8_t;

typedef uint32_t FixPoint1616_t;


#define NULL 0


#define config_USE_CG_Correction     1
#define config_USE_PileUp_Correction 1
#define config_CAL_COFF              1
#define config_CAL_DMAX              1
#define config_SET_MODE              1


#define VI5300_DEVICE_ADDR          0xD8

#define VI5300_REG_MCU_CFG          0x00
#define VI5300_RET_INT_STATUS       0x03
#define VI5300_REG_SYS_CFG          0x01
#define VI5300_REG_PW_CTRL          0x07
#define VI5300_REG_CMD              0x0a
#define VI5300_REG_SIZE             0x0b
#define VI5300_REG_SCRATCH_PAD_BASE 0x0c 

#define VI5300_WRITEFW_CMD          0x03
#define VI5300_USER_CFG_CMD         0x09
#define VI5300_START_RANG_CMD       0x0E

#define VI5300_CFG_SUBCMD           0x01
#define VI5300_OTPW_SUBCMD          0x02
#define VI5300_OTPR_SUBCMD          0x03

#define VI5300_CHECK_RET(a)   if(a != VI5300_OK) return a



#endif


