#ifndef __PID_H__
#define __PID_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"
#include "math.h"
/* 类型定义 ------------------------------------------------------------------*/
enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,
  DELTA_PID,
};
typedef struct pid_t
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;    //input max err;
  float input_min_err;    //input max err;
  float output_deadband;  //output deadband;
  
  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;

  void  (*f_param_init)(struct pid_t *pid, 
                        uint32_t      pid_mode,
                        uint32_t      max_output,
                        uint32_t      inte_limit,
                        float         p,
                        float         i,
                        float         d);
  void  (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
  float (*f_pid_calc)(struct pid_t *pid, float get, float set);
  void  (*f_pid_ClearIntegrals)(struct pid_t*   pid);
} pid_t;
/* 宏定义 --------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd);
#endif  // __PID_H__

