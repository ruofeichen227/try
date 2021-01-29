#include "shoot.h"
#include "robot.h"

#define FIRE_RATE 1000
#define SHOOT_SPEED 4940

//
extern struct Robot_t infantry;
extern int expect_position, curr_position;
void ShootParamChange(void)
{
	if(infantry.ShootMode == shoot_disabled)
	{
		SHOOT_PLUCK_MOTOR.speed_pid.output = 0;
		SHOOT_FRICTION_MOTOR1.speed_pid.output = 0;
		SHOOT_FRICTION_MOTOR2.speed_pid.output = 0;
		CanTransmit_1234(&hcan2, 0,0,0,0);
		return;
	}
	else if(infantry.ShootMode == shoot_enabled)
	{
		
		SHOOT_FRICTION_MOTOR1.speed_pid.ref = -infantry.shoot.ShootSpeed;
		SHOOT_FRICTION_MOTOR2.speed_pid.ref = infantry.shoot.ShootSpeed;
		
		if(infantry.WorkState == MouseKeyControl)
		{
		  if(infantry.ShootWay == SingleShoot || infantry.ShootWay == DoubleShoot)
		  {
			  if(expect_position > curr_position)
		      SHOOT_PLUCK_MOTOR.position_pid.ref = expect_position; 
		    else
	        SHOOT_PLUCK_MOTOR.position_pid.ref = curr_position;       //消除位置环抖动
		  }
		}
		else if(infantry.WorkState == RemoteControl || infantry.ShootWay == ContinuousShoot)	
		    SHOOT_PLUCK_MOTOR.speed_pid.ref = infantry.shoot.FireRate;

		ShootParamCalculate();
		CanTransmit_1234(&hcan2, SHOOT_FRICTION_MOTOR1.speed_pid.output, SHOOT_FRICTION_MOTOR2.speed_pid.output, SHOOT_PLUCK_MOTOR.speed_pid.output, 0);
	}
}

/**
	* @brief 云台发射系统电机输出数据计算
	* @param None
	* @retval None
	*/
void ShootParamCalculate(void)
{
	if(infantry.WorkState == MouseKeyControl)
	{
    if(infantry.ShootWay == SingleShoot || infantry.ShootWay == DoubleShoot)
	  {
	    SHOOT_PLUCK_MOTOR.position_pid.fdb = (float)SHOOT_PLUCK_MOTOR.real_position;
      PID_Calc(&SHOOT_PLUCK_MOTOR.position_pid);
      SHOOT_PLUCK_MOTOR.speed_pid.ref = SHOOT_PLUCK_MOTOR.position_pid.output;
      SHOOT_PLUCK_MOTOR.speed_pid.fdb = (float)SHOOT_PLUCK_MOTOR.fdbSpeed;
    }
	}
	else if(infantry.WorkState == RemoteControl || infantry.ShootWay == ContinuousShoot)
		SHOOT_PLUCK_MOTOR.speed_pid.fdb = SHOOT_PLUCK_MOTOR.fdbSpeed;
	
	SHOOT_FRICTION_MOTOR1.speed_pid.fdb = SHOOT_FRICTION_MOTOR1.fdbSpeed;
	SHOOT_FRICTION_MOTOR2.speed_pid.fdb = SHOOT_FRICTION_MOTOR2.fdbSpeed;
	PID_Calc(&SHOOT_PLUCK_MOTOR.speed_pid);
	PID_Calc(&SHOOT_FRICTION_MOTOR1.speed_pid);
	PID_Calc(&SHOOT_FRICTION_MOTOR2.speed_pid);
}
