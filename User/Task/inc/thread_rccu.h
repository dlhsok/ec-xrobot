#ifndef __THREAD_RCCU_H_
#define __THREAD_RCCU_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include "mytype.h"
#include "chassis_LineTracker.h"

/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void ChassisCoord_Set(float _x_diff, float _y_diff, float _yaw_diff);
void ChassisCoord_WaitStop(void);
void ChassisCoord_WaitYawStop(void);
void ServoSetPluseAndTime(uint8_t mode, uint16_t pwmval,uint16_t time);
void ChassisSpeed_Set(float _x_spd, float _y_spd);
int32_t Read_Position_x_mm(void);
int32_t Read_Position_y_mm(void);
void rccu_setmode_to_tracking( void );
void ChassisStop(void);
void ChassisSpeed_WaitCarToStop(void);
void ChassisSpeed_Set1(float _x_spd, float _y_spd ,float _yaw);
#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* __THREAD_RCCU_H_ */


