/* Includes ------------------------------------------------------------------*/
#include "chassis_function.h"
#ifdef ARM_MATH_CM4
#include "arm_math.h"
#else
#include <math.h>
#endif
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/****************************************  Park 变换  ****************************************/
/****************************************  Park 变换  ****************************************/
static void Chassis_MoveTransform(ChassisHandle_TypeDef* chassis_handle, fp32* chassis_vx, fp32* chassis_vy)
{
#ifdef ARM_MATH_CM4
  chassis_handle->sin_yaw = arm_sin_f32(chassis_handle->gimbal_yaw_ecd_angle / RAD_TO_ANGLE);
  chassis_handle->cos_yaw = arm_cos_f32(chassis_handle->gimbal_yaw_ecd_angle / RAD_TO_ANGLE);
#else
  chassis_handle->sin_yaw = sin(chassis_handle->gimbal_yaw_ecd_angle / RAD_TO_ANGLE);
  chassis_handle->cos_yaw = cos(chassis_handle->gimbal_yaw_ecd_angle / RAD_TO_ANGLE);
#endif
  *chassis_vy = chassis_handle->cos_yaw * chassis_handle->vy + chassis_handle->sin_yaw * chassis_handle->vx;
  *chassis_vx = -chassis_handle->sin_yaw * chassis_handle->vy + chassis_handle->cos_yaw * chassis_handle->vx;
}
/****************************************  麦克纳姆轮解算  ****************************************/
/****************************************  麦克纳姆轮解算  ****************************************/
static void Mecanum_Calculate_InverseTransform(ChassisHandle_TypeDef* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw)
{
  //麦克纳姆轮逆运算函数
  fp32 wheel_rpm[4];
  fp32 max = 0;
  fp32 rate;
  uint8_t i;

  VAL_LIMIT(chassis_vx, -chassis_handle->MechanicalStruct_Para.MAX_Chassis_VX_Speed, chassis_handle->MechanicalStruct_Para.MAX_Chassis_VX_Speed);  //mm/s
  VAL_LIMIT(chassis_vy, -chassis_handle->MechanicalStruct_Para.MAX_Chassis_VY_Speed, chassis_handle->MechanicalStruct_Para.MAX_Chassis_VY_Speed);  //mm/s
  VAL_LIMIT(chassis_vw, -chassis_handle->MechanicalStruct_Para.MAX_Chassis_VW_Speed, chassis_handle->MechanicalStruct_Para.MAX_Chassis_VW_Speed);  //deg/s

  wheel_rpm[0] = ( chassis_vy - chassis_vx + (chassis_vw * chassis_handle->rotate_ratio_fr)) * chassis_handle->wheel_rpm_ratio;
  wheel_rpm[1] = ( chassis_vy + chassis_vx - (chassis_vw * chassis_handle->rotate_ratio_fl)) * chassis_handle->wheel_rpm_ratio;
  wheel_rpm[2] = ( chassis_vy - chassis_vx - (chassis_vw * chassis_handle->rotate_ratio_bl)) * chassis_handle->wheel_rpm_ratio;
  wheel_rpm[3] = ( chassis_vy + chassis_vx + (chassis_vw * chassis_handle->rotate_ratio_br)) * chassis_handle->wheel_rpm_ratio;

  //find max item
  for (i = 0; i < 4; i++)
    if (fabs(wheel_rpm[i]) > max)
      max = fabs(wheel_rpm[i]);

  //equal proportion
  if (max > chassis_handle->MechanicalStruct_Para.MAX_Wheel_rpm)
  {
    rate = chassis_handle->MechanicalStruct_Para.MAX_Wheel_rpm / max;
    for (i = 0; i < 4; i++)
    {
      wheel_rpm[i] *= rate;
    }
  }

  memcpy(chassis_handle->wheel_rpm, wheel_rpm, 4 * sizeof(fp32));
}


static void Mecanum_Calculate_ForwardTransform( ChassisHandle_TypeDef* chassis_handle,
    int16_t Wheel1Speed,
    int16_t Wheel2Speed,
    int16_t Wheel3Speed,
    int16_t Wheel4Speed,
    int32_t Wheel1Distance,
    int32_t Wheel2Distance,
    int32_t Wheel3Distance,
    int32_t Wheel4Distance )

{
  //麦克纳姆轮正运算函数
  static double last_d_x, last_d_y, last_d_w, d_x, d_y, d_w, diff_d_x, diff_d_y, diff_d_w;
  static double mecanum_angle, last_mecanum_angle, diff_mecanum_angle;
  static double position_x, position_y, angle_w;
  static double v_x, v_y, v_w;

  chassis_handle->position.speed_rpm[0] = Wheel1Speed;
  chassis_handle->position.speed_rpm[1] = Wheel2Speed;
  chassis_handle->position.speed_rpm[2] = Wheel3Speed;
  chassis_handle->position.speed_rpm[3] = Wheel4Speed;

  chassis_handle->position.total_ecd[0] = Wheel1Distance;
  chassis_handle->position.total_ecd[1] = Wheel2Distance;
  chassis_handle->position.total_ecd[2] = Wheel3Distance;
  chassis_handle->position.total_ecd[3] = Wheel4Distance;

  last_d_x = d_x;
  last_d_y = d_y;
  last_d_w = d_w;
	last_mecanum_angle = mecanum_angle;

  d_x = chassis_handle->ecd_ratio * (-chassis_handle->position.total_ecd[0] + chassis_handle->position.total_ecd[1] - chassis_handle->position.total_ecd[2] + chassis_handle->position.total_ecd[3]);
  d_y = chassis_handle->ecd_ratio * (chassis_handle->position.total_ecd[0] + chassis_handle->position.total_ecd[1] + chassis_handle->position.total_ecd[2] + chassis_handle->position.total_ecd[3]);
  d_w = chassis_handle->ecd_ratio * ( (chassis_handle->position.total_ecd[0] / chassis_handle->rotate_ratio_fr) - \
                                      (chassis_handle->position.total_ecd[1] / chassis_handle->rotate_ratio_fl) - \
                                      (chassis_handle->position.total_ecd[2] / chassis_handle->rotate_ratio_bl) + \
                                      (chassis_handle->position.total_ecd[3] / chassis_handle->rotate_ratio_br) );
  diff_d_x = d_x - last_d_x;
  diff_d_y = d_y - last_d_y;
  diff_d_w = d_w - last_d_w;
	diff_mecanum_angle = mecanum_angle - last_mecanum_angle;
	if(diff_mecanum_angle > 180)
	{
		diff_mecanum_angle -= 360;
	}
	else if(diff_mecanum_angle < -180)
	{
		diff_mecanum_angle += 360;
	}

  /* use glb_chassis gyro angle data */
  mecanum_angle = chassis_handle->yaw_gyro_angle / RAD_TO_ANGLE;

  position_x += diff_d_x * cos(mecanum_angle) - diff_d_y * sin(mecanum_angle);
  position_y += diff_d_x * sin(mecanum_angle) + diff_d_y * cos(mecanum_angle);
//  angle_w += diff_d_w;
	angle_w += diff_mecanum_angle;

  chassis_handle->position.position_x_mm = position_x;        //mm
  chassis_handle->position.position_y_mm = position_y;        //mm
  chassis_handle->position.angle = angle_w;
  chassis_handle->position.angle_deg = angle_w * RAD_TO_ANGLE; //degree

  v_x = ( -chassis_handle->position.speed_rpm[0] + chassis_handle->position.speed_rpm[1] - chassis_handle->position.speed_rpm[2] + chassis_handle->position.speed_rpm[3] ) / ((double)chassis_handle->wheel_rpm_ratio * 4.0);
  v_y = ( chassis_handle->position.speed_rpm[0] + chassis_handle->position.speed_rpm[1] + chassis_handle->position.speed_rpm[2] + chassis_handle->position.speed_rpm[3] ) / ((double)chassis_handle->wheel_rpm_ratio * 4.0);
  v_w = ( (chassis_handle->position.speed_rpm[0] / chassis_handle->rotate_ratio_fr) - \
          (chassis_handle->position.speed_rpm[1] / chassis_handle->rotate_ratio_fl) - \
          (chassis_handle->position.speed_rpm[2] / chassis_handle->rotate_ratio_bl) + \
          (chassis_handle->position.speed_rpm[3] / chassis_handle->rotate_ratio_br) ) / ((double)chassis_handle->wheel_rpm_ratio * 4.0);

  chassis_handle->position.v_x_mm = v_x;                 //mm/s
  chassis_handle->position.v_y_mm = v_y;                 //mm/s
  chassis_handle->position.rate_deg = v_w * RAD_TO_ANGLE; //degree/s

}
static void Mecanum_Chassis_ControlCalc(ChassisHandle_TypeDef* chassis_handle)
{
  Chassis_MoveTransform(chassis_handle, &chassis_handle->chassis_vx, &chassis_handle->chassis_vy);
  Mecanum_Calculate_InverseTransform(chassis_handle, chassis_handle->chassis_vx, chassis_handle->chassis_vy, chassis_handle->vw);
}
static void Mecanum_CalcRotateRatio( ChassisHandle_TypeDef* chassis_handle,
                                     fp32 wheelbase,
                                     fp32 wheeltrack,
                                     fp32 rotate_x_offset,
                                     fp32 rotate_y_offset,
                                     fp32 wheel_perimeter,
                                     fp32 motor_encoder_accuracy,
                                     fp32 MAX_Chassis_VX_Speed,
                                     fp32 MAX_Chassis_VY_Speed,
                                     fp32 MAX_Chassis_VW_Speed,
                                     fp32 MAX_Wheel_rpm )
{
  fp32 temporary;
  temporary = ( wheelbase + wheeltrack ) / 2.0f;
  chassis_handle->MechanicalStruct_Para.wheelbase = wheelbase;
  chassis_handle->MechanicalStruct_Para.wheeltrack = wheeltrack;
  chassis_handle->MechanicalStruct_Para.rotate_x_offset = rotate_x_offset;
  chassis_handle->MechanicalStruct_Para.rotate_y_offset = rotate_y_offset;
  chassis_handle->MechanicalStruct_Para.wheel_perimeter = wheel_perimeter;

  chassis_handle->MechanicalStruct_Para.motor_encoder_accuracy = motor_encoder_accuracy;
  chassis_handle->ecd_ratio = wheel_perimeter * MOTOR_REDUCTION_RATIO / (4 * motor_encoder_accuracy);

  chassis_handle->MechanicalStruct_Para.MAX_Chassis_VX_Speed = MAX_Chassis_VX_Speed;
  chassis_handle->MechanicalStruct_Para.MAX_Chassis_VY_Speed = MAX_Chassis_VY_Speed;
  chassis_handle->MechanicalStruct_Para.MAX_Chassis_VW_Speed = MAX_Chassis_VW_Speed;
  chassis_handle->MechanicalStruct_Para.MAX_Wheel_rpm = MAX_Wheel_rpm;

  chassis_handle->rotate_ratio_fl = (temporary - rotate_x_offset - rotate_y_offset) / RAD_TO_ANGLE;
  chassis_handle->rotate_ratio_fr = (temporary - rotate_x_offset + rotate_y_offset) / RAD_TO_ANGLE;
  chassis_handle->rotate_ratio_bl = (temporary + rotate_x_offset - rotate_y_offset) / RAD_TO_ANGLE;
  chassis_handle->rotate_ratio_br = (temporary + rotate_x_offset + rotate_y_offset) / RAD_TO_ANGLE;
  chassis_handle->wheel_rpm_ratio = 60.0f / (wheel_perimeter * MOTOR_REDUCTION_RATIO);
}
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/****************************************  底盘控制  ****************************************/
/****************************************  底盘控制  ****************************************/
void Chassis_Init( ChassisHandle_TypeDef* chassis_handle,
                   ChassisType_TypeDef ChassisType,
                   fp32 MAX_Chassis_VX_Speed,   //最大X轴速度
                   fp32 MAX_Chassis_VY_Speed,   //最大Y轴速度
                   fp32 MAX_Chassis_VW_Speed,   //最大中心旋转轴速度
                   fp32 MAX_Wheel_rpm )         //轮子最大速度
{
  memset(chassis_handle, 0, sizeof(ChassisHandle_TypeDef));
  chassis_handle->ChassisType = ChassisType;
  switch(chassis_handle->ChassisType)
  {
  case DEBUG_COMMONWHEEL:                   //调试模式
  {
  } break;
  case FOUR_DRIVE_McNamara:                 //麦克纳姆轮
  {
    Mecanum_CalcRotateRatio( chassis_handle,
                             WHEELBASE,
                             WHEELTRACK,
                             GIMBAL_X_OFFSET,
                             GIMBAL_Y_OFFSET,
                             WHEEL_PERIMETER,
                             MOTOR_ENCODER_ACCURACY,
                             MAX_Chassis_VX_Speed,
                             MAX_Chassis_VY_Speed,
                             MAX_Chassis_VW_Speed,
                             MAX_Wheel_rpm );
    chassis_handle->Chassis_CtrlFunc = Mecanum_Chassis_ControlCalc;
    chassis_handle->Chassis_Calc_ForwardTransform = Mecanum_Calculate_ForwardTransform;
  }
  break;
  }
}





