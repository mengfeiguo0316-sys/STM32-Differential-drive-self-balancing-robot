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

static void Get_Angle(void);

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
	PERIODIC(MPU6050_Debug, 10);
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
        Get_Angle(); // 严格每 5ms 更新一次姿态角与角速度

    }
}

static void Get_Angle(void)
{	
	MPU6050_GetPhysical();

}
