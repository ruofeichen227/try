#include "chassis.h"
#include "robot.h"

//
extern struct Robot_t infantry;

/**
	* @brief 底盘控制-Remote
	* @param None
	* @retval None
	*/
void ChassisCtrl_Remote(void)
{
	infantry.velocity.forward_back_speed = 16.0f*(float)(Remote.rc.ch1 - CHx_BIAS);
	infantry.velocity.left_right_speed = 16.0f*(float)(Remote.rc.ch0 - CHx_BIAS);
	infantry.velocity.turn_speed = 100;
	infantry.velocity.rot_speed = 2400;
	if(infantry.ChassisMode == RotateRun)
	{
		//减慢速度
		infantry.velocity.forward_back_speed *= 0.25f;
		infantry.velocity.left_right_speed *= 0.25f;
	}
	
}

/**
	* @brief 底盘控制-MouseKey
	* @param None
	* @retval None
	*/
void ChassisCtrl_MouseKey(void)
{
	if(Remote.keyboard & Key_Shift) //key : Shift  Speed up
	{
		infantry.velocity.forward_back_speed =  2700;//700
		infantry.velocity.left_right_speed = 2500;//750
		infantry.velocity.turn_speed = 60;
		infantry.velocity.rot_speed = 3200;
	}
	if(!(Remote.keyboard & Key_Shift))
	{
		infantry.velocity.forward_back_speed = 1800;//400
		infantry.velocity.left_right_speed = 1750;//450
		infantry.velocity.turn_speed = 40;
		infantry.velocity.rot_speed = 2000;
	}
	
	if(Remote.keyboard & Key_W)	 //key : W  forward
		infantry.velocity.forward_back_speed *= 1;
	else if(Remote.keyboard & Key_S)	//key : S  backward
		infantry.velocity.forward_back_speed *= -1;
	else
		infantry.velocity.forward_back_speed = 0;
	
	if(Remote.keyboard & Key_D)	//key : D  right
		infantry.velocity.left_right_speed *= 1;
	else if(Remote.keyboard & Key_A)	//key : A  left
		infantry.velocity.left_right_speed *= -1;
	else
		infantry.velocity.left_right_speed = 0;
}
