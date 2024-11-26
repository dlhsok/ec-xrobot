#ifndef __PID_REGULATOR_H__
#define __PID_REGULATOR_H__
/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
#ifdef _cplusplus
extern "C" {
#endif   
/* 宏定义 --------------------------------------------------------------------*/
/**
  * @brief  Macro to compute logarithm of two
  */
#define LOG2(x) \
  ((x) == 65535 ? 16 : \
   ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 15 : \
    ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 14 : \
     ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2 ? 13 : \
      ((x) == 2*2*2*2*2*2*2*2*2*2*2*2 ? 12 : \
       ((x) == 2*2*2*2*2*2*2*2*2*2*2 ? 11 : \
        ((x) == 2*2*2*2*2*2*2*2*2*2 ? 10 : \
         ((x) == 2*2*2*2*2*2*2*2*2 ? 9 : \
          ((x) == 2*2*2*2*2*2*2*2 ? 8 : \
           ((x) == 2*2*2*2*2*2*2 ? 7 : \
            ((x) == 2*2*2*2*2*2 ? 6 : \
             ((x) == 2*2*2*2*2 ? 5 : \
              ((x) == 2*2*2*2 ? 4 : \
               ((x) == 2*2*2 ? 3 : \
                ((x) == 2*2 ? 2 : \
                 ((x) == 2 ? 1 : \
                  ((x) == 1 ? 0 : -1)))))))))))))))))
/* 类型定义 ------------------------------------------------------------------*/
typedef struct PID_Handle
{
	int16_t   hDefKpGain;           /**< Default Kp gain */
    int16_t   hDefKiGain;           /**< Default Ki gain */
	int16_t   hDefKdGain;           /**< Default Kd gain */
	int16_t   hKpGain;              /**< Kp gain used by PID component */
    int16_t   hKiGain;              /**< Ki gain used by PID component */
	int16_t   hKdGain;              /**< Kd gain used by PID component */
	
	uint16_t  hKpDivisor;           /**< Kp gain divisor, used in conjuction with
                                       Kp gain allows obtaining fractional values.
                                       If FULL_MISRA_C_COMPLIANCY is not defined
                                       the divisor is implemented through
                                       algebrical right shifts to speed up PI
                                       execution. Only in this case this parameter
                                       specifies the number of right shifts to be
                                       executed */
    uint16_t  hKiDivisor;           /**< Ki gain divisor, used in conjuction with
                                       Ki gain allows obtaining fractional values.
                                       If FULL_MISRA_C_COMPLIANCY is not defined
                                       the divisor is implemented through
                                       algebrical right shifts to speed up PI
                                       execution. Only in this case this parameter
                                       specifies the number of right shifts to be
                                       executed */
	uint16_t  hKdDivisor;           /**< Kd gain divisor, used in conjuction with
                                       Kd gain allows obtaining fractional values.
                                       If FULL_MISRA_C_COMPLIANCY is not defined
                                       the divisor is implemented through
                                       algebrical right shifts to speed up PI
                                       execution. Only in this case this parameter
                                       specifies the number of right shifts to be
                                       executed */
	uint16_t  hKpDivisorPOW2;       /**< Kp gain divisor expressed as power of 2.
                                       E.g. if gain divisor is 512 the value
                                       must be 9 as 2^9 = 512 */
    uint16_t  hKiDivisorPOW2;       /**< Ki gain divisor expressed as power of 2.
                                       E.g. if gain divisor is 512 the value
                                       must be 9 as 2^9 = 512 */
	uint16_t  hKdDivisorPOW2;       /*!< Kd gain divisor expressed as power of 2.
                                       E.g. if gain divisor is 512 the value
                                       must be 9 as 2^9 = 512 */
	int32_t   hUpperOutputLimit;    /**< Upper limit used to saturate the PI output */
    int32_t   hLowerOutputLimit;    /**< Lower limit used to saturate the PI output */
    int32_t   wLowerIntegralLimit;	/**< Lower limit used to saturate the integral 
	                                     term given by Ki / Ki divisor * integral of
                                         process variable error */
	int32_t   wPrevProcessVarError; /*!< previous process variable used by the
                                       derivative part of the PID component */
	int32_t   wUpperIntegralLimit;  /**< Upper limit used to saturate the integral 
	                                     term given by Ki / Ki divisor * integral of 
	                                     process variable error */
    //比例项
	int32_t   wProportionalTerm;
	//积分项
	int32_t   wIntegralTerm;        /**< integral term */
	//微分项
	int32_t   wDifferentialTerm;
} PID_Handle_t;
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
//参数配置
void PID_SetKP( PID_Handle_t * pHandle, int16_t hKpGain );
void PID_SetKI( PID_Handle_t * pHandle, int16_t hKpGain );
void PID_SetKD( PID_Handle_t * pHandle, int16_t hKpGain );
void PID_SetIntegralTerm( PID_Handle_t * pHandle, int32_t wIntegralTermValue );
//任务执行
void PID_HandleInit( PID_Handle_t * pHandle );
int32_t PI_Controller( PID_Handle_t * pHandle, int32_t wProcessVarError );
int32_t PID_Controller( PID_Handle_t * pHandle, int32_t wProcessVarError );



#ifdef _cplusplus
}
#endif   

#endif
