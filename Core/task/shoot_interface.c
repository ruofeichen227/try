#include "shoot.h"
#include "robot.h"
#include "datatypes.h"

extern struct Robot_t infantry;
extern SendData data_rx;

int curr_position = 0, last_curr_position = 0, expect_position = 0;

/**
	* @brief 射击控制-Remote
	* @param None
	* @retval None
	*/
void ShootCtrl_Remote(void)
{
//	if(infantry.ShootWay == SingleShoot)                                         //位置环
//	{
//		curr_position = SHOOT_PLUCK_MOTOR.fdbPosition;
//		if(Remote.mouse.press_l == 1)   //触发方式待更改
//		expect_position = SHOOT_PLUCK_MOTOR.fdbPosition + 300;    //数值待更改
//		
//		if(expect_position > curr_position)
//		SHOOT_PLUCK_MOTOR.position_pid.ref = expect_position; 
//		else
//		SHOOT_PLUCK_MOTOR.position_pid.ref = curr_position;
//	}	
//	else if(infantry.ShootWay == DoubleShoot)
//	{
//		curr_position = SHOOT_PLUCK_MOTOR.fdbPosition;
//		if(Remote.mouse.press_l == 1)   //触发方式待更改
//		expect_position = SHOOT_PLUCK_MOTOR.fdbPosition + 600;    //数值待更改
//		
//		if(expect_position > curr_position)
//		SHOOT_PLUCK_MOTOR.position_pid.ref = expect_position; 
//		else
//		SHOOT_PLUCK_MOTOR.position_pid.ref = curr_position;
//	}
//	else if(infantry.ShootWay == ContinuousShoot)                                //速度环
    infantry.shoot.ShootSpeed = 4940;
		infantry.shoot.FireRate = 2.0f*(float)(Remote.rc.ch4 - CHx_BIAS);
}

/**
	* @brief 射击控制-MouseKey
	* @param None
	* @retval None
	*/
void ShootCtrl_MouseKey(void)
{
	infantry.shoot.ShootSpeed = 4940;
	infantry.shoot.ShootSpeed = 5500;
	if(Remote.mouse.press_r == 1 && Remote.last_mouse.press_r != 1)   //右键切换射击模式
	{
		if(infantry.ShootWay == SingleShoot)
			infantry.ShootWay = DoubleShoot;
		else if(infantry.ShootWay == DoubleShoot)
			infantry.ShootWay = ContinuousShoot;
		else if(infantry.ShootWay == ContinuousShoot)
			infantry.ShootWay = SingleShoot;
	}
	
 	if(infantry.ShootWay == SingleShoot)                                //位置环
	{
		curr_position = SHOOT_PLUCK_MOTOR.real_position;
		if(Remote.rc.ch4 != 1024 && Remote.last_rc.ch4 == 1024) //if(Remote.mouse.press_l == 1 && Remote.last_mouse.press_l != 1)  //左键射击
		expect_position = SHOOT_PLUCK_MOTOR.real_position + 50000;           //数值待更改
	}	
	else if(infantry.ShootWay == DoubleShoot)
	{
		curr_position = SHOOT_PLUCK_MOTOR.real_position;
		if(Remote.rc.ch4 != 1024 && Remote.last_rc.ch4 == 1024)//if(Remote.mouse.press_l == 1 && Remote.last_mouse.press_l != 1) 
		expect_position = SHOOT_PLUCK_MOTOR.real_position + 100000;    //数值待更改
	}
	else if(infantry.ShootWay == ContinuousShoot) 		         //速度环
	{
		if(Remote.mouse.press_l == 1)
	    infantry.shoot.FireRate = 1000;
		else
			infantry.shoot.FireRate = 0;
	}
}