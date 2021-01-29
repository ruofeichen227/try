#ifndef ATTITUDERESOLVE_H
#define ATTITUDERESOLVE_H

#include "stm32f4xx.h"

#define BMI088

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024
#define GYRO_VARIANCE_BASE				4000
#define SENSORS_ACC_SCALE_SAMPLES  		400

struct Axis_f
{
	float x;
	float y;
	float z;
};

struct Axis_i
{
	int x;
	int y;
	int z;
};

struct Angle_t
{
	int yaw;
	int last_yaw;
	int encoder_yaw;
//	int last_encoder_yaw;
	int yaw_count;
	int pitch;
	int last_pitch;
	int encoder_pitch;
	int pitch_count;
	int roll;
	int last_roll;
	int encoder_roll;
	int roll_count;
};

struct IMU_t
{
	struct Axis_f acc;
	struct Axis_f gyro;
	float temp;
	struct Axis_f attitude; 	//姿态角数据（0~360）
	struct Angle_t angle;     //姿态角数据（0~8192）
	struct Axis_f gyroBias;
	int accBiasFound;
};
// 陀螺仪校准-采样数据结构体
typedef struct
{
	struct Axis_f bias;
	int isBiasValueFound;
	int isBufferFilled;
	struct Axis_f buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

static void sensorsCalculateVarianceAndMean(BiasObj* bias, struct Axis_f* varOut, struct Axis_f* meanOut);
static int sensorsFindBiasValue(BiasObj* bias);
int processGyroBias(float gx, float gy, float gz, struct Axis_f *gyroBiasOut);
int processAccScale(float ax, float ay, float az, int* accBiasFound);
void imuUpdate(struct Axis_f acc, struct Axis_f gyro, struct IMU_t *imu);

extern void IMU_init(struct IMU_t *imu);
extern void imuDataHandle(struct IMU_t* imu);

#endif
