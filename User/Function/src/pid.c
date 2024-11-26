/* 包含头文件 ----------------------------------------------------------------*/
#include "pid.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/ 
void abs_limit(float *a, float ABS_MAX)
{
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < -ABS_MAX)
    *a = -ABS_MAX;
}

static void pid_param_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float    kp,
    float    ki,
    float    kd)
{

  pid->integral_limit = intergral_limit;
  pid->max_out        = maxout;
  pid->pid_mode       = mode;

  pid->p = kp;
  pid->i = ki;
  pid->d = kd;

}
/**
  * @brief     modify pid parameter when code running
  * @param[in] pid: control pid struct
  * @param[in] p/i/d: pid parameter
  * @retval    none
  */
static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
  pid->p = kp;
  pid->i = ki;
  pid->d = kd;
  
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out  = 0;
  
}

/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output 
  */
float pid_calc(pid_t *pid, float get, float set)
{
  pid->get = get;
  pid->set = set;
  pid->err[NOW] = set - get;

  if ((pid->input_max_err != 0) && (pid->err[NOW] > pid->input_max_err))
      pid->err[NOW] = pid->input_max_err;
  if ((pid->input_min_err != 0) && (pid->err[NOW] < pid->input_min_err))
      pid->err[NOW] = pid->input_min_err;
  if (pid->pid_mode == POSITION_PID) //position PID
  {
      pid->pout = pid->p * pid->err[NOW];
      pid->iout += pid->i * pid->err[NOW];
      pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
    
      abs_limit(&(pid->iout), pid->integral_limit);
      pid->out = pid->pout + pid->iout + pid->dout;
      abs_limit(&(pid->out), pid->max_out);
  }
  else if (pid->pid_mode == DELTA_PID) //delta PID
  {
      pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
      pid->iout = pid->i * pid->err[NOW];
      pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

      pid->out += pid->pout + pid->iout + pid->dout;
      abs_limit(&(pid->out), pid->max_out);
  }

  pid->err[LLAST] = pid->err[LAST];
  pid->err[LAST]  = pid->err[NOW];
  
  
  if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
    return 0;
  else
    return pid->out;

}
void pid_ClearIntegrals(pid_t*   pid)
{
	pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out  = 0;
}
/**
  * @brief     initialize pid parameter
  * @retval    none
  */
void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd)
{
  pid->f_param_init = pid_param_init;
  pid->f_pid_reset  = pid_reset;
  pid->f_pid_calc = pid_calc;
  pid->f_pid_ClearIntegrals = pid_ClearIntegrals;

  pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
  pid->f_pid_reset(pid, kp, ki, kd);
}


