/************************************************
 * Developer : Gonk
 * GitHub    : https://github.com/mengfeiguo0316-sys
 * Email     : mengfeiguo0316@gmail.com or 2250513147@qq.com
 * Phone     : 199 5063 9665
 *
 * Version   : V1.0.0
 * Updated   : 2026-11-12
 *
 * Description:MPU6050 传感器驱动，负责初始化及原始/物理量读取
 *   
 * Copyright (c) 2026 Gonk
 * All rights reserved.
 ************************************************/

#include "MPU6050.h"
#include "SI2C.h"

// 实际情况决定
#define MPU6050_ADDR        0x68
// 核心控制与配置寄存器
#define WHO_AM_I            0x75    // 器件身份识别寄存器（默认读出值为 0x68）
#define PWR_MGMT_1          0x6B    // 电源管理 1（解除休眠、唤醒芯片、设置时钟源）
#define SMPLRT_DIV          0x19    // 采样频率分频器（设置数据输出速率）
#define CONFIG              0x1A    // 低通滤波配置 (DLPF)
#define GYRO_CONFIG         0x1B    // 陀螺仪量程配置
#define ACCEL_CONFIG        0x1C    // 加速度计量程配置

MPU6050_DebugData_t mpu6050_data;

//函数声明
static void MPU6050_Calibrate(void);
static void MPU6050_ReadRawData(int16_t *accel, int16_t *gyro);


/**
 * @brief  初始化 MPU6050，配置采样率，滤波器，陀螺仪，加速度计量程
 * @param  无
 */
uint8_t MPU6050_Init(void)
{
    uint8_t data;

    if (SI2C_Read(MPU6050_ADDR, WHO_AM_I, &data))
        return 0;
    
    // WHO_AM_I 寄存器出厂默认固定值就是 0x68
    if (data != 0x68)
        return 0;

    if (SI2C_Write(MPU6050_ADDR, PWR_MGMT_1, 0x01))
        return 0;

    if (SI2C_Write(MPU6050_ADDR, SMPLRT_DIV, 0x00))
        return 0;

    if (SI2C_Write(MPU6050_ADDR, CONFIG, 0x00))
        return 0;

    if (SI2C_Write(MPU6050_ADDR, GYRO_CONFIG, 0x18))
        return 0;

    if (SI2C_Write(MPU6050_ADDR, ACCEL_CONFIG, 0x00))
        return 0;
        
    HAL_Delay(50);

    MPU6050_Calibrate(); // 校准陀螺仪零偏

    return 1;
}

/**
 * @brief  陀螺仪零漂校准（在小车开机、初始化 MPU6050 完成后，小车必须处于绝对静止状态调用）
 * @note   连续采样多次求平均值，作为零位偏置保存
 */
static void MPU6050_Calibrate(void)
{
    int32_t sum_gx = 0;
    int32_t sum_gy = 0;
    int32_t sum_gz = 0;
    const uint16_t samples = 200; // 采集 200 次求平均

    int16_t temp_accel[3];
    int16_t temp_gyro[3];

    for (uint16_t i = 0; i < samples; i++)
    {
        MPU6050_ReadRawData(temp_accel, temp_gyro);
        sum_gx += temp_gyro[0];
        sum_gy += temp_gyro[1];
        sum_gz += temp_gyro[2];
        HAL_Delay(5); // 稍微间隔 5ms 采样一次，确保数据刷新
    }

    // 计算静态零偏
    mpu6050_data.Gyro_Offset_X = (int16_t)(sum_gx / samples);
    mpu6050_data.Gyro_Offset_Y = (int16_t)(sum_gy / samples);
    mpu6050_data.Gyro_Offset_Z = (int16_t)(sum_gz / samples);
}


/**
 * @brief  读取 MPU6050 的 6 轴原始 16 位 ADC 数据
 * @param  accel: 存储 3 轴原始加速度的 int16_t 数组
 * @param  gyro:  存储 3 轴原始角速度的 int16_t 数组
 */
static void MPU6050_ReadRawData(int16_t *accel, int16_t *gyro)
{
    uint8_t buf[14];

    if (SI2C_ReadBytes(MPU6050_ADDR, 0x3B, buf, 14) != 0)
    {
        return;
    }

    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);

    gyro[0] = (int16_t)((buf[8] << 8) | buf[9]);
    gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);
}

/**
 * @brief  获取 MPU6050 转换后的真实物理量（单位：g 和 °/s）
 */
void MPU6050_GetPhysical(void)
{
    uint8_t buf[14];
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;

    // 更新数据
    if (SI2C_ReadBytes(MPU6050_ADDR, 0x3B, buf, 14) != 0)
        return;

    // 1. 拼接 16 位有符号原始值（大端模式，高位在前）
    raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    raw_az = (int16_t)((buf[4] << 8) | buf[5]);
    // buf[6] 和 buf[7] 为温度数据，跳过
    raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
    raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    // 2. 转换为加速度物理量（±2g 量程 -> 16384.0f LSB/g）
    mpu6050_data.ax = (float)raw_ax / 16384.0f;
    mpu6050_data.ay = (float)raw_ay / 16384.0f;
    mpu6050_data.az = (float)raw_az / 16384.0f;

    // 3. 转换为角速度物理量（±2000°/s 量程 -> 16.4f LSB/(°/s)），并扣除静态零偏
    mpu6050_data.gx = ((float)raw_gx - mpu6050_data.Gyro_Offset_X) / 16.4f;
    mpu6050_data.gy = ((float)raw_gy - mpu6050_data.Gyro_Offset_Y) / 16.4f;
    mpu6050_data.gz = ((float)raw_gz - mpu6050_data.Gyro_Offset_Z) / 16.4f;
}

MPU6050_DebugData_t MPU6050_GetDebugData(void)
{
	MPU6050_DebugData_t data;
	
	data.pitch=mpu6050_data.pitch;
	data.row=mpu6050_data.row;
	data.yaw=mpu6050_data.yaw;

	return data;
}
