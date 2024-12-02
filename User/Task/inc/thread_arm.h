#ifndef __THREAD_ARM_H_
#define __THREAD_ARM_H_

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
void ArmCoord_SetAbsolute(float _x, float _y);
void ArmCoord_WaitStop(void);
void ArmCoord_WaitYawStop(void);
void ServoSetPluseAndTime(uint8_t mode, uint16_t pwmval,uint16_t time);
void ArmSpeed_Set(float _x_spd, float _y_spd);
int32_t Read_Position_x_mm(void);
int32_t Read_Position_y_mm(void);
int32_t Read_Angle_deg(void);
void arm_setmode_to_tracking( void );
void arm_setmode_to_coord( void );
void arm_setmode_to_normal( void );
void ArmStop(void);
void ArmSpeed_WaitCarToStop(void);
void ArmSpeed_Set1(float _x_spd, float _y_spd ,float _yaw);
#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* __THREAD_RCCU_H_ */


