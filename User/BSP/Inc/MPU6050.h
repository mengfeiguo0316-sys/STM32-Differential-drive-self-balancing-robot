#ifndef __MPU6050_H
#define __MPU6050_H
#include "stm32f1xx_hal.h"

typedef struct
{
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;//MPU6050原始数据
	
	int16_t Gyro_Offset_X;
	int16_t Gyro_Offset_Y;
	int16_t Gyro_Offset_Z;// 零偏全局变量（供后续姿态解算减去偏置）

	float yaw;
	float pitch;
	float row;//互补滤波融合欧拉角
	
	float yaw_a;
	float pitch_a;
	float row_a;//加速度解算欧拉角

    float yaw_g;
	float pitch_g;
	float row_g;//陀螺仪解算欧拉角

}MPU6050_Data_t;

extern MPU6050_Data_t mpu6050_data;

uint8_t MPU6050_Init(void);
void MPU6050_GetPhysical(void);
float * MPU6050_GetDebugData(void);
#endif




