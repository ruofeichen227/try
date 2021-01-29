#include "gimbal.h"
#include "robot.h"
#include "datatypes.h"
#include "pc_communicate.h"
//
extern struct Robot_t infantry;
extern SendData data_rx;
/**
	* @brief 云台控制-Remote
	* @param None
	* @retval None
	*/
void GimbalCtrl_Remote(void)
{
	if(Remote.rc.ch3 > CHx_BIAS)
		infantry.gimbal.PitchAngle = infantry.gimbal.PitchBiasAngle + 1.0f*(float)(Remote.rc.ch3 - CHx_BIAS);
	else
		infantry.gimbal.PitchAngle = infantry.gimbal.PitchBiasAngle + 0.8f*(float)(Remote.rc.ch3 - CHx_BIAS);
	
	if(infantry.GimbalMode == Follow_Gyro_Mode)
		infantry.gimbal.YawAngle -= 0.05f*(float)(Remote.rc.ch2 - CHx_BIAS);
	else if(infantry.GimbalMode == Follow_Encoder_Mode)
	{
		static float RotateAngle;
		if(infantry.ChassisMode == Run)
		{
			infantry.gimbal.YawAngle = infantry.gimbal.YawBiasAngle - 1.5f*(float)(Remote.rc.ch2 - CHx_BIAS);
			RotateAngle = infantry.gimbal.YawAngle;
		}
		else if(infantry.ChassisMode == RotateRun)
		{
			RotateAngle += (0.00134f*infantry.velocity.rot_speed - 0.01f*(float)(Remote.rc.ch2 - CHx_BIAS));
			while(RotateAngle > 8191)
				RotateAngle -= 8192;
			infantry.gimbal.YawAngle = RotateAngle;
		}
	}
}

/**
	* @brief 云台控制-MouseKey
	* @param None
	* @retval None
	*/
void GimbalCtrl_MouseKey(void)
{
	if(infantry.GameMode == AutoAim || data_rx.z!=0)
	{
//		int delta_yaw, delta_pitch;
//		delta_yaw = GIMBAL_YAW_MOTOR.fdbPosition - infantry.gimbal.YawBiasAngle;
//		delta_pitch = GIMBAL_PITCH_MOTOR.fdbPosition - infantry.gimbal.PitchBiasAngle;
//		if((delta_pitch<50 || delta_pitch>-50) && (delta_yaw<100 || delta_yaw>-100))
//			auto_aim_ready = 1;
//		
//		if(auto_aim_ready == 0) 
//		{
//			infantry.gimbal.PitchAngle = infantry.gimbal.PitchBiasAngle;
//			infantry.gimbal.YawAngle = infantry.gimbal.YawBiasAngle;
//		}
		//电脑瞄准状态且已找到目标
		infantry.gimbal.PitchAngle = GIMBAL_PITCH_MOTOR.fdbPosition + data_rx.y * 22.756f*0.5;	//8192 / 360 / 100
		infantry.gimbal.YawAngle = GIMBAL_YAW_MOTOR.fdbPosition - data_rx.x * 22.756f;
//		infantry.gimbal.PitchAngle = GIMBAL_PITCH_MOTOR.fdbPosition + data_rx.y * 22.756f*0.5;	//8192 / 360 装甲板
//		infantry.gimbal.YawAngle = GIMBAL_YAW_MOTOR.fdbPosition - data_rx.x * 22.756f;
	}
	else //(Remote.key.z == 0 || pc_data_rx.dist == 0)
	{
		if(Remote.mouse.y > 0)
			infantry.gimbal.PitchAngle -= 0.25f*(float)Remote.mouse.y;
		else
			infantry.gimbal.PitchAngle -= 0.32f*(float)Remote.mouse.y;
		infantry.gimbal.YawAngle -= 0.28f*(float)Remote.mouse.x;
	}
}
