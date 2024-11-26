/* 包含头文件 ----------------------------------------------------------------*/
#include "SCARA.h"
#include "math.h"
#include "data.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
/***********机械臂*************/
_s_SCARA SCARA_S = {
	LINKAGE1,        //主臂长度
	LINKAGE2,        //副臂长度
	CENCER_OFFSET,   //旋转轴偏移量
	HEAD_OFFSET,     //头部偏移量
	{1.0f,1.0f,1.0f},//缩放比例
    {SCARA_OFFSET_X,SCARA_OFFSET_Y,SCARA_OFFSET_Z},//机械臂笛卡尔坐标系偏移量
	0.0f,0.0f,0.0f,//计算角度（主臂），计算角度（副臂），计算角度（旋转轴）
	{243.87f,0.0f,275.0f},//笛卡尔坐标系（设置值）
	{0.0f,0.0f,0.0f},    //机械臂正解得出的笛卡尔坐标
	{90.0f,0.0f,0.0f}     //机械臂反解得出的角度
};//机械臂数据
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void calculate_SCARA_forward_Transform(float Angle_BIGARM,    //计算角度（主臂）
									   float Angle_FOREARM,   //计算角度（副臂）
									   float Angle_ROTAT)     //计算角度（旋转轴）
{//机械臂正解函数
	float x_sin;
	float x_cos;
	float y_sin;
	float y_cos;
	float arm_xy;
	
	x_sin = sin(Angle_BIGARM/SCARA_RAD2DEG) * SCARA_S.Linkage_1;
    x_cos = cos(Angle_BIGARM/SCARA_RAD2DEG) * SCARA_S.Linkage_1;
    y_sin = sin(Angle_FOREARM/SCARA_RAD2DEG) * SCARA_S.Linkage_2;
    y_cos = cos(Angle_FOREARM/SCARA_RAD2DEG) * SCARA_S.Linkage_2;
	
	arm_xy = x_cos + y_cos + SCARA_S.CencerOffset + SCARA_S.HeadOffset;
	
	SCARA_S.axis[X_AXIS] = arm_xy * cos(Angle_ROTAT/SCARA_RAD2DEG) - SCARA_S.SCARA_offset[X_AXIS];
	SCARA_S.axis[Y_AXIS] = arm_xy * sin(Angle_ROTAT/SCARA_RAD2DEG) - SCARA_S.SCARA_offset[Y_AXIS];
	SCARA_S.axis[Z_AXIS] = x_sin + y_sin - SCARA_S.SCARA_offset[Z_AXIS];
}
void calculate_delta(float cartesian[3]) 
{//机械臂反解函数
	float SCARA_pos[3];
	float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi; 
	float ARM_XYZ, ARM_XY;

	SCARA_pos[X_AXIS] = cartesian[X_AXIS] * SCARA_S.axis_scaling[X_AXIS] + SCARA_S.SCARA_offset[X_AXIS];  
	SCARA_pos[Y_AXIS] = cartesian[Y_AXIS] * SCARA_S.axis_scaling[Y_AXIS] + SCARA_S.SCARA_offset[Y_AXIS];  
	SCARA_pos[Z_AXIS] = cartesian[Z_AXIS] * SCARA_S.axis_scaling[Z_AXIS] + SCARA_S.SCARA_offset[Z_AXIS];

	ARM_XY = sqrt(pow(SCARA_pos[X_AXIS],2) + pow(SCARA_pos[Y_AXIS],2)) - SCARA_S.CencerOffset - SCARA_S.HeadOffset;
	ARM_XYZ = sqrt(pow(ARM_XY,2) + pow(SCARA_pos[Z_AXIS],2));

	SCARA_C2 = (pow(ARM_XYZ,2) - pow(SCARA_S.Linkage_1,2) - pow(SCARA_S.Linkage_2,2))/(2 * SCARA_S.Linkage_1 * SCARA_S.Linkage_2);

	SCARA_S2 = sqrt( 1 - pow(SCARA_C2,2) );

	SCARA_K1 = SCARA_S.Linkage_1 + SCARA_S.Linkage_2 * SCARA_C2;
	SCARA_K2 = SCARA_S.Linkage_2 * SCARA_S2;

	SCARA_theta = (atan2(SCARA_pos[Z_AXIS],ARM_XY)+atan2(SCARA_K2, SCARA_K1));
	SCARA_psi   =  atan2(SCARA_S2, SCARA_C2);

	SCARA_S.Angle[X_AXIS] = SCARA_theta * SCARA_RAD2DEG;  
	SCARA_S.Angle[Y_AXIS] = (SCARA_theta - SCARA_psi) * SCARA_RAD2DEG; 
	SCARA_S.Angle[Z_AXIS] = atan2(SCARA_pos[Y_AXIS],SCARA_pos[X_AXIS]) * SCARA_RAD2DEG;
}

void Servo_AngleToPWM(float Angle[3],
	                 uint16_t *qAngleValue_ARM1,
					 uint16_t *qAngleValue_ARM2,
					 uint16_t *qAngleValue_YAW)
{//舵机角度转换位PWM
	float BigArm_Angle;  //大臂角度
	float ForeArm_Angle; //小臂角度
	float Rotat_Angle; //地盘角度
	float Rotat_AngleBuffer;
	
	BigArm_Angle = Angle[X_AXIS] - BIGARM_INIT_ANGLE;
	if(BigArm_Angle > 360.0f)  BigArm_Angle -= 360.0f;
	if(BigArm_Angle < -360.0f) BigArm_Angle += 360.0f;
	if(BigArm_Angle > 180.0f)  BigArm_Angle = BigArm_Angle - 360.0f;
	if(BigArm_Angle < -180.0f)  BigArm_Angle = BigArm_Angle + 360.0f;
	BigArm_Angle = SERVO_BIGARM_ZERO + ((BigArm_Angle*1000.0f/SERVO_ANGLE) + 0.5f);
	
	ForeArm_Angle = Angle[Y_AXIS] - FOREARM_INIT_ANGLE;
	if(ForeArm_Angle > 360.0f)  ForeArm_Angle -= 360.0f;
	if(ForeArm_Angle < -360.0f) ForeArm_Angle += 360.0f;
	if(ForeArm_Angle > 180.0f)  ForeArm_Angle = ForeArm_Angle - 360.0f;
	if(ForeArm_Angle < -180.0f)  ForeArm_Angle = ForeArm_Angle + 360.0f;
	ForeArm_Angle = SERVO_FOREARM_ZERO - ((ForeArm_Angle*1000.0f/SERVO_ANGLE) + 0.5f);
	
	Rotat_Angle = Angle[Z_AXIS] - ROTAT_INIT_ANGLE;
	if(Rotat_Angle > 360.0f)  Rotat_Angle -= 360.0f;
	if(Rotat_Angle < -360.0f) Rotat_Angle += 360.0f;
	if(Rotat_Angle > 180.0f)  Rotat_Angle = Rotat_Angle - 360.0f;
	if(Rotat_Angle < -180.0f)  Rotat_Angle = Rotat_Angle + 360.0f;
	
	Rotat_AngleBuffer = ( Rotat_Angle * 1000.0f / SERVO_ANGLE ) + 0.5f + SERVO_YAW_ZERO;
	if( Rotat_AngleBuffer > 2500 )
	{
		Rotat_Angle  = Rotat_Angle-360;
		Rotat_AngleBuffer = ( Rotat_Angle * 1000.0f / SERVO_ANGLE ) + 0.5f + SERVO_YAW_ZERO;
	}
	Rotat_Angle = Rotat_AngleBuffer;
	
	*qAngleValue_ARM1 = BigArm_Angle;
	*qAngleValue_ARM2 = ForeArm_Angle;
	*qAngleValue_YAW = Rotat_Angle;
}
void Servo_PWMToAngle(uint16_t AngleValue_ARM1,
					  uint16_t AngleValue_ARM2,
					  uint16_t AngleValue_YAW,
	                  float Angle[3])
{//舵机PWM转换位角度
	float BigArm_Angle;  //大臂角度
	float ForeArm_Angle; //小臂角度
	float Rotat_Angle; //地盘角度
	
	BigArm_Angle = ((float)((AngleValue_ARM1-SERVO_BIGARM_ZERO)*SERVO_ANGLE/1000.0f)) + BIGARM_INIT_ANGLE;
	if(BigArm_Angle > 360.0f)  BigArm_Angle -= 360.0f;
	if(BigArm_Angle < -360.0f) BigArm_Angle += 360.0f;
	if(BigArm_Angle > 180.0f)  BigArm_Angle = BigArm_Angle - 360.0f;
	
	ForeArm_Angle = ((float)((SERVO_FOREARM_ZERO-AngleValue_ARM2)*SERVO_ANGLE/1000.0f)) + FOREARM_INIT_ANGLE;
	if(ForeArm_Angle > 360.0f)  ForeArm_Angle -= 360.0f;
	if(ForeArm_Angle < -360.0f) ForeArm_Angle += 360.0f;
	if(ForeArm_Angle > 180.0f)  ForeArm_Angle = ForeArm_Angle - 360.0f;
	
	Rotat_Angle = ((float)((AngleValue_YAW-SERVO_YAW_ZERO)*SERVO_ANGLE/1000.0f)) + ROTAT_INIT_ANGLE;
	if(Rotat_Angle > 360.0f)  Rotat_Angle -= 360.0f;
	if(Rotat_Angle < -360.0f) Rotat_Angle += 360.0f;
	if(Rotat_Angle > 180.0f)  Rotat_Angle = Rotat_Angle - 360.0f;
	
	Angle[X_AXIS] = BigArm_Angle;
	Angle[Y_AXIS] = ForeArm_Angle;
	Angle[Z_AXIS] = Rotat_Angle;
}



