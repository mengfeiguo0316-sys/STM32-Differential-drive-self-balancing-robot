/************************************************
 * Developer : Gonk
 * GitHub    : https://github.com/mengfeiguo0316-sys
 * Email     : mengfeiguo0316@gmail.com or 2250513147@qq.com
 * Phone     : 199 5063 9665
 *
 * Version   : V1.0.0
 * Updated   : 2026-11-12
 *
 * Description:应用任务管理模块，负责系统主循环与业务任务调度主要管理软实时任务
 * 			   硬实时任务在中断完成保证系统稳定性。
 *   
 * Copyright (c) 2026 Gonk
 * All rights reserved.
 ************************************************/
#include"app_tasks.h"
#include "MPU6050.h"
#include "Filter.h"
#include "fast_math.h"
#include"Delay.h"
#include"VOFA+.h"
#include"Tasks.h"
#include "key.h"
#include "led.h"
#include "stm32f1xx_hal.h"

//函数声明
static void MPU6050_GetAngle(void);

/************************************************ 应用任务初始化 ************************************************/
void app_Tasks_Init(void)
{
	Delay_Init();
	MPU6050_Init();
	VOFA_Plus_Init();
}

/************************************************软实时任务 主循环轮询************************************************/
void app_task_VOFA_Plus(void)
{
	PERIODIC(MPU6050_Debug, 20);
	VOFA_Plus_SendMPU6050Data();
}

void app_task_LED1_TOGGLE(void)
{
	PERIODIC(LED1_TOGGLE, 500);
	BSP_LED_Toggle(LED1);
}


/************************************************硬实时任务 中断实时处理************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
    if (GPIO_Pin == GPIO_PIN_9)
    {
        MPU6050_GetAngle(); // 严格每 5ms 更新一次姿态角与角速度
    }
}


/**
 * @brief  更新MPU6050 融合后的姿态角（单位：°）和角速度（单位：°/s）
 */
/**
 * @brief  更新 MPU6050 融合后的三轴姿态角（单位：°）和角速度（单位：°/s）
 */
void MPU6050_GetAngle(void)
{
    MPU6050_GetPhysical();

    //加速度计低通预处理
    float clean_ax = LPF_Update(&Filter_AccelX, mpu6050_data.ax);
    float clean_ay = LPF_Update(&Filter_AccelY, mpu6050_data.ay);
    float clean_az = LPF_Update(&Filter_AccelZ, mpu6050_data.az);

    
    mpu6050_data.pitch   = CF_Update(&pitch, -clean_ax, clean_az, mpu6050_data.gy);
    mpu6050_data.pitch_g = pitch.gyro_rate; // 俯仰角速度，供直立环 PID 的 D 项使用

    mpu6050_data.row     = CF_Update(&row,   clean_ay, clean_az, mpu6050_data.gx);
    mpu6050_data.row_g   = row.gyro_rate;  // 横滚角速度

    //Yaw 航向角积分（加入 ±0.15 °/s 死区，抑制静止温漂）
    if (mpu6050_data.gz > 0.15f || mpu6050_data.gz < -0.15f)
    {
        mpu6050_data.yaw += mpu6050_data.gz * 0.005f;
    }
}
