#ifndef ROBOT_H
#define ROBOT_H

#include "stm32f4xx.h"
#include "main.h"
#include "bsp_can.h"
#include "motor.h"
#include "remote.h"
#include "AttitudeResolve.h"
#include "pc_communicate.h"
#include "referee.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"

//底盘
extern struct CAN_Motor can1_motor_1;
extern struct CAN_Motor can1_motor_2;
extern struct CAN_Motor can1_motor_3;
extern struct CAN_Motor can1_motor_4;
//发射系统
extern struct CAN_Motor can2_motor_1;
extern struct CAN_Motor can2_motor_2;
extern struct CAN_Motor can2_motor_3;
//云台双轴
extern struct CAN_Motor can1_motor_9;
extern struct CAN_Motor can2_motor_9;
//遥控器
extern struct DT7Remote_t Remote;
//陀螺仪
extern struct IMU_t bmi088;
//小电脑
extern struct PC_Data_TX_t pc_data_tx;
extern struct PC_Data_RX_t pc_data_rx;
//裁判系统

//电机具体功能宏定义
#define CHASSIS_MOTOR1 can1_motor_1
#define CHASSIS_MOTOR2 can1_motor_2
#define CHASSIS_MOTOR3 can1_motor_3
#define CHASSIS_MOTOR4 can1_motor_4
#define GIMBAL_YAW_MOTOR can1_motor_9
#define GIMBAL_PITCH_MOTOR can2_motor_9
#define SHOOT_PLUCK_MOTOR can2_motor_3
#define SHOOT_FRICTION_MOTOR1 can2_motor_1
#define SHOOT_FRICTION_MOTOR2 can2_motor_2

//控制模式
enum WorkState_e
{
	STOP,
	RemoteControl,
	MouseKeyControl
};
enum Chassis_Mode_e
{
	Run,
	RotateRun
};
enum Gimbal_Mode_e
{
	//底盘跟随云台
	Follow_Encoder_Mode,
	Follow_Gyro_Mode,
	//云台跟随底盘
	Follow_Chassis		
};
enum Shoot_Mode_e
{
	shoot_enabled,
	shoot_disabled
};
enum Game_Mode_e
{
	Normal,
	AutoAim,
	AtkBuffActivate
};

enum Shoot_Way_e
{
	SingleShoot,
	DoubleShoot,
	ContinuousShoot,
	RemoteShoot
};

//各部分运动参数
struct Chassis_t
{
	int FBSpeed;
	int LRSpeed;
	int RotateSpeed;
};
struct Gimbal_t
{
	float PitchAngle;
	float PitchBiasAngle;
	float PitchAngle_lowest;
	float PitchAngle_highest;
	float YawAngle;
	float YawBiasAngle;
	float Yaw_offset;
};
struct Shoot_t
{
	float ShootSpeed;
	float FireRate;
};

//整体运动速度
struct Robot_Vel
{
	float forward_back_speed;	//前后速度
	float left_right_speed;		//左右速度
	float turn_speed;					//转向速度
	float rot_speed;					//小陀螺速度
};

//机器人参数结构体
struct Robot_t
{
	//mode
	enum WorkState_e WorkState;
	enum WorkState_e Last_WorkState;
	enum Chassis_Mode_e ChassisMode;
	enum Gimbal_Mode_e GimbalMode;
	enum Shoot_Mode_e ShootMode;
	enum Game_Mode_e GameMode;
	enum Shoot_Way_e ShootWay;
	//param
	struct Gimbal_t gimbal;
	struct Shoot_t shoot;
	struct Chassis_t chassis;
	//speed - for the whole robot
	struct Robot_Vel velocity;
};

extern void RobotParamInit(void);
extern void RobotStateChange(void);

#endif
