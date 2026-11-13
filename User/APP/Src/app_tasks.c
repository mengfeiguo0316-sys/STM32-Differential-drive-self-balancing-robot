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

static void Get_Angle(void);

/************************************************软实时任务 主循环轮询************************************************/

void app_Tasks_Init(void)
{
	Delay_Init();
	MPU6050_Init();
	VOFA_Plus_Init();
	Balance_Car_Filter_Init();
}

void app_task_VOFA_Plus(void)
{
	PERIODIC(MPU6050_Debug, 5);
	float *channels=MPU6050_GetDebugData();
	VOFA_Plus_SendData(channels, 3);
}



/************************************************硬实时任务 中断处理************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
  if(GPIO_Pin==GPIO_PIN_9)
  {
	Get_Angle();//更新角度
	
  }
}

static void Get_Angle(void)
{
    MPU6050_GetPhysical();  // 更新加速度与角速度物理量

    // 加速度计解算角度
    mpu6050_data.pitch_a = qatan2(-mpu6050_data.ax, mpu6050_data.az) * 57.29578f;
    mpu6050_data.row_a   = qatan2( mpu6050_data.ay, mpu6050_data.az) * 57.29578f;
    mpu6050_data.yaw_a   = 0.0f;
    
	//互补滤波闭环融合
	//mpu6050_data.pitch = F_Update(&Filter_AccelX, mpu6050_data.pitch_a);
	//mpu6050_data.row   = F_Update(&Filter_AccelX, mpu6050_data.row_a);
    mpu6050_data.yaw  += mpu6050_data.gz * 0.005f;// Yaw 绕垂直地面的 Z 轴旋转 -> 累加 gz
}
