#include "chassis.h"
#include "robot.h"
#include "ramp.h"
#include <math.h>

struct ramp_t ChassisFBRamp = RAMP_DEFAULT;
struct ramp_t ChassisLRRamp = RAMP_DEFAULT;
//
extern struct Robot_t infantry;

/**
	* @brief 底盘数据计算
	* @param none
	* @retval none
	*/
void ChassisParamChange(void)
{	
	if(infantry.WorkState == STOP)
	{
		CanTransmit_1234(&hcan1, 0,0,0,0);
		return;
	}
	
	float angle = infantry.gimbal.Yaw_offset;
	//angle = 2.5f*(float)(Remote.rc.ch2 - CHx_BIAS);
	//底盘运动模式
	if(infantry.ChassisMode == Run)
	{
		infantry.chassis.FBSpeed = infantry.velocity.forward_back_speed;
		infantry.chassis.LRSpeed = infantry.velocity.left_right_speed;
		infantry.chassis.RotateSpeed = -0.0001f* fabsf(angle)* angle* infantry.velocity.turn_speed;
	}
	else if(infantry.ChassisMode == RotateRun)
	{
		if(angle < 0)angle += 8192;	//sin()参数为正,将负的角度值变回正值
		angle = angle * 0.00076f;	//0.00076 = 2*pi / 8192, 将8192度欧拉角变回弧度角
		
		infantry.chassis.FBSpeed = (float)infantry.velocity.forward_back_speed * cos(angle) + (float)infantry.velocity.left_right_speed * sin(angle);
		infantry.chassis.LRSpeed = (float)infantry.velocity.forward_back_speed * -sin(angle) + (float)infantry.velocity.left_right_speed * cos(angle);
		infantry.chassis.RotateSpeed = infantry.velocity.rot_speed;
	}
	
	CHASSIS_MOTOR1.speed_pid.ref = infantry.chassis.FBSpeed + infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
	CHASSIS_MOTOR2.speed_pid.ref = -infantry.chassis.FBSpeed + infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
	CHASSIS_MOTOR3.speed_pid.ref = -infantry.chassis.FBSpeed - infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
	CHASSIS_MOTOR4.speed_pid.ref = infantry.chassis.FBSpeed - infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
	//PID calculate & send msg
	ChassisParamCalculate();
//	CanTransmit_1234(&hcan1, CHASSIS_MOTOR1.speed_pid.output, CHASSIS_MOTOR2.speed_pid.output,
//														 CHASSIS_MOTOR3.speed_pid.output, CHASSIS_MOTOR4.speed_pid.output);
}

/**
	* @brief 底盘电机输出数据计算
	* @param None
	* @retval None
	*/
void ChassisParamCalculate(void)
{
	CHASSIS_MOTOR1.speed_pid.fdb = CHASSIS_MOTOR1.fdbSpeed;
	CHASSIS_MOTOR2.speed_pid.fdb = CHASSIS_MOTOR2.fdbSpeed;
	CHASSIS_MOTOR3.speed_pid.fdb = CHASSIS_MOTOR3.fdbSpeed;
	CHASSIS_MOTOR4.speed_pid.fdb = CHASSIS_MOTOR4.fdbSpeed;
	
	PID_Calc(&CHASSIS_MOTOR1.speed_pid);
	PID_Calc(&CHASSIS_MOTOR2.speed_pid);
	PID_Calc(&CHASSIS_MOTOR3.speed_pid);
	PID_Calc(&CHASSIS_MOTOR4.speed_pid);
}
