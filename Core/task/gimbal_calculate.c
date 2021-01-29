#include "gimbal.h"
#include "robot.h"
#include "ramp.h"

struct ramp_t PitchRamp = RAMP_DEFAULT;     //机器人pitch轴斜坡
struct ramp_t YawRamp = RAMP_DEFAULT;       //机器人yaw轴斜坡
//
extern struct Robot_t infantry;

#include "FreeRTOS.h"
#include "task.h"

/**
	* @brief 底盘电机数据计算
	* @param none
	* @retval none
	*/
void GimbalParamChange(void)
{
	if(infantry.WorkState == STOP)
	{
		CanTransmit_9_12(&hcan1, 0,0,0,0);
		CanTransmit_9_12(&hcan2, 0,0,0,0);
		return;
	}
	
	//Offset_过YAWMotor零点处理
	infantry.gimbal.Yaw_offset = GIMBAL_YAW_MOTOR.fdbPosition - infantry.gimbal.YawBiasAngle;
	if(GIMBAL_YAW_MOTOR.fdbPosition - GIMBAL_YAW_MOTOR.last_fdbPosition > 4096)
		infantry.gimbal.Yaw_offset -= 8192;
	else if(GIMBAL_YAW_MOTOR.fdbPosition - GIMBAL_YAW_MOTOR.last_fdbPosition < -4096)
		infantry.gimbal.Yaw_offset += 8192;
	
	//Position limited
	if(infantry.gimbal.PitchAngle < infantry.gimbal.PitchAngle_lowest)infantry.gimbal.PitchAngle = infantry.gimbal.PitchAngle_lowest;
	if(infantry.gimbal.PitchAngle > infantry.gimbal.PitchAngle_highest)infantry.gimbal.PitchAngle = infantry.gimbal.PitchAngle_highest;
	
	GIMBAL_PITCH_MOTOR.position_pid.ref = infantry.gimbal.PitchAngle;
	GIMBAL_YAW_MOTOR.position_pid.ref = infantry.gimbal.YawAngle;
	
	GimbalParamCalculate();
	CanTransmit_9_12(&hcan1, GIMBAL_YAW_MOTOR.speed_pid.output, 0, 0, 0);
	CanTransmit_9_12(&hcan2, GIMBAL_PITCH_MOTOR.speed_pid.output, 0, 0, 0);
}

/**
	* @brief 云台Yaw轴与Pitch轴电机输出数据计算
	* @param None
	* @retval None
	*/
void GimbalParamCalculate(void)
{
	//Pitch
	GIMBAL_PITCH_MOTOR.position_pid.fdb = (float)GIMBAL_PITCH_MOTOR.real_position;
	PID_Calc(&GIMBAL_PITCH_MOTOR.position_pid);
	GIMBAL_PITCH_MOTOR.speed_pid.ref = GIMBAL_PITCH_MOTOR.position_pid.output;
	GIMBAL_PITCH_MOTOR.speed_pid.fdb = bmi088.gyro.x * 57.29578f;//(float)GIMBAL_PITCH_MOTOR.fdbSpeed;
	PID_Calc(&GIMBAL_PITCH_MOTOR.speed_pid);
	//Yaw
	if(infantry.GimbalMode == Follow_Gyro_Mode)
		GIMBAL_YAW_MOTOR.position_pid.fdb = bmi088.angle.encoder_yaw;
	else if(infantry.GimbalMode == Follow_Encoder_Mode)
		GIMBAL_YAW_MOTOR.position_pid.fdb = (float)GIMBAL_YAW_MOTOR.fdbPosition;
	
	PID_Calc(&GIMBAL_YAW_MOTOR.position_pid);
	GIMBAL_YAW_MOTOR.speed_pid.ref = GIMBAL_YAW_MOTOR.position_pid.output;
	GIMBAL_YAW_MOTOR.speed_pid.fdb = bmi088.gyro.z * 57.29578f;	//单位:deg/s
	PID_Calc(&GIMBAL_YAW_MOTOR.speed_pid);
}
