/**
  ******************************************************************************
  * 文件名          : robot.c
  * 文件描述        : 步兵机器人参数设置
  * 创建时间        : 2020.7.31
  * 作者            : 邓紫龙
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.1.20
  * 修改人          : 邓紫龙
  ******************************************************************************
  * 1.本代码基于STMF427IIT6开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
  */

#include "robot.h"
#include "supervise.h"
#include <string.h>
#include "datatypes.h"

//ctrl msg
struct Robot_t infantry;
extern SendData data_rx;
/**
	* @brief 机器人数据初始化
	* @param none
	* @retval none
	*/
void RobotParamInit(void)
{
	infantry.gimbal.PitchBiasAngle = 1360;	//此处为pitch电机fdbposition数据
	infantry.gimbal.YawBiasAngle = 2710;		//此处为Yaw电机fdbposition数据
	infantry.gimbal.PitchAngle_highest = infantry.gimbal.PitchBiasAngle + 800;
	infantry.gimbal.PitchAngle_lowest = infantry.gimbal.PitchBiasAngle - 430;
	
	infantry.ShootWay = SingleShoot;
	
	memset(&CHASSIS_MOTOR1, 0, sizeof(CHASSIS_MOTOR1));
	memset(&CHASSIS_MOTOR2, 0, sizeof(CHASSIS_MOTOR2));
	memset(&CHASSIS_MOTOR3, 0, sizeof(CHASSIS_MOTOR3));
	memset(&CHASSIS_MOTOR4, 0, sizeof(CHASSIS_MOTOR4));
	memset(&GIMBAL_PITCH_MOTOR, 0, sizeof(GIMBAL_PITCH_MOTOR));
	memset(&GIMBAL_YAW_MOTOR, 0, sizeof(GIMBAL_YAW_MOTOR));
	memset(&SHOOT_PLUCK_MOTOR, 0, sizeof(SHOOT_PLUCK_MOTOR));
	memset(&SHOOT_FRICTION_MOTOR1, 0, sizeof(SHOOT_FRICTION_MOTOR1));
	memset(&SHOOT_FRICTION_MOTOR2, 0, sizeof(SHOOT_FRICTION_MOTOR2));
	
	MotorParamInit(&CHASSIS_MOTOR1,	20,0,0,0,12000,	0,0,0,0,0);
	MotorParamInit(&CHASSIS_MOTOR2,	20,0,0,0,12000, 0,0,0,0,0);
	MotorParamInit(&CHASSIS_MOTOR3,	20,0,0,0,12000, 0,0,0,0,0);
	MotorParamInit(&CHASSIS_MOTOR4,	20,0,0,0,12000, 0,0,0,0,0);
	MotorParamInit(&GIMBAL_PITCH_MOTOR,	100,0.6,0,2048,10000,	0.6,0,0,5000,4000);
	MotorParamInit(&GIMBAL_YAW_MOTOR,		100,0,0,2048,12000, 	0.5,0,0,5000,5000);
	MotorParamInit(&SHOOT_PLUCK_MOTOR,			20,0.6,0,2048,8000, 	0.5,0,0,5000,5000);
	MotorParamInit(&SHOOT_FRICTION_MOTOR1,	20,0,0,0,8000, 	0,0,0,0,0);
	MotorParamInit(&SHOOT_FRICTION_MOTOR2,	20,0,0,0,8000, 	0,0,0,0,0);
}

void RobotStateChange(void)
{
	infantry.Last_WorkState = infantry.WorkState;
	if(infantry.Last_WorkState == STOP)
	{
		infantry.GimbalMode = Follow_Encoder_Mode;
		//使进入工作状态后按当前位置校准底盘
		if(infantry.GimbalMode == Follow_Gyro_Mode)
			infantry.gimbal.YawAngle = bmi088.angle.encoder_yaw;
		else if(infantry.GimbalMode == Follow_Encoder_Mode)
			infantry.gimbal.YawAngle = GIMBAL_YAW_MOTOR.fdbPosition;
		
		//infantry.gimbal.PitchAngle = infantry.gimbal.PitchBiasAngle;
	}
	
	if(bmi088.accBiasFound == 0 || Remote.inputmode == RC_Stop || Is_Error(1<<GYRO_INDEX) || Is_Error(1<<RC_INDEX))
	{
		infantry.WorkState = STOP;
		infantry.ShootMode = shoot_disabled;
		//chassis
		CHASSIS_MOTOR1.speed_pid.output = 0;
		CHASSIS_MOTOR2.speed_pid.output = 0;
		CHASSIS_MOTOR3.speed_pid.output = 0;
		CHASSIS_MOTOR4.speed_pid.output = 0;
		//gimbal
		GIMBAL_YAW_MOTOR.speed_pid.output = 0;
		GIMBAL_PITCH_MOTOR.speed_pid.output = 0;
		//shoot
		SHOOT_PLUCK_MOTOR.speed_pid.output = 0;
		SHOOT_FRICTION_MOTOR1.speed_pid.output = 0;
		SHOOT_FRICTION_MOTOR2.speed_pid.output = 0;
		return;
	}
	else if(Remote.inputmode == RC_Remote)
		infantry.WorkState = RemoteControl;
	else if(Remote.inputmode == RC_MouseKey)
	{
		infantry.WorkState = MouseKeyControl;
		if(Remote.key.z == 1 || data_rx.z!=0)
			infantry.GameMode = AutoAim;
		else
			infantry.GameMode = Normal;
	}
	
	//修改工作状态-remote拨码开关
	if(Remote.rc.s1 == 1)
		infantry.ShootMode = shoot_disabled;
	else
		infantry.ShootMode = shoot_enabled;
	
	if(Remote.rc.s1 == 2)
		infantry.ChassisMode = RotateRun;
	else
		infantry.ChassisMode = Run;
	
	//根据控制方式修改参数
	if(infantry.WorkState == RemoteControl)
	{
		ChassisCtrl_Remote();
		GimbalCtrl_Remote();
		ShootCtrl_Remote();
		//=====================Shoot=======================
	}
	else if(Remote.inputmode == RC_MouseKey)
	{
		ChassisCtrl_MouseKey();
		GimbalCtrl_MouseKey();
		ShootCtrl_MouseKey();
		//=====================Shoot=======================
		
		
//		if(Remote.mouse.press_l == 1)
//			infantry.shoot.FireRate = 4500;
//		else
//			infantry.shoot.FireRate = 0;
	}
}

