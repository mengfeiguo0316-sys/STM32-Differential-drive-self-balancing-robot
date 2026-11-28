/************************************************
 * Developer : Gonk
 * GitHub    : https://github.com/mengfeiguo0316-sys
 * Email     : mengfeiguo0316@gmail.com or 2250513147@qq.com
 * Phone     : 199 5063 9665
 *
 * Version   : V1.0.0
 * Updated   : 2026-11-28
 *
 * Description: AT8236驱动模块，负责电机驱动芯片AT8236的初始化与PWM控制
 *   
 * Copyright (c) 2026 Gonk
 * All rights reserved.
 ************************************************/
#include "AT8236.h"
#include "tim.h"
#include "stm32f1xx_hal_tim.h"

#define PWM_MAX 3599
#define PWM_FULL (PWM_MAX + 1) // 3600，在向上计数模式下输出严格 100% 恒高电平

void Motor_Brake(void);
/**
 * @brief  AT8236初始化函数，配置PWM输出通道
 * @param  无
 */
void AT8236_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // 左 IN1
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // 左 IN2
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // 右 IN1
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // 右 IN2

    Motor_Brake();//进入刹车待命状态 
}

/**
 * @brief  AT8236限幅函数，限制PWM输出范围在[-PWM_MAX, PWM_MAX]
 * @param  无
 */
static int Limit(int v)
{
    if (v >  PWM_MAX) v =  PWM_MAX;
    if (v < -PWM_MAX) v = -PWM_MAX;
    return v;
}

/**
 * @brief  慢衰减：pwm>0 正转，<0 反转，=0 主动刹车（永不触发 AT8236 休眠）
 */
void Motor_Set(int left, int right)
{
    left  = Limit(left);
    right = Limit(right);

    if (left >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_FULL);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_FULL - left);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_FULL + left); // left为负，等价于 PWM_FULL - |left|
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_FULL);
    }

    if (right >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_FULL);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_FULL - right);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_FULL + right);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_FULL);
    }
}

/**
 * @brief   电机刹车函数，将两路PWM输出置为最大值，实现快速刹车
 */
void Motor_Brake(void)  
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_MAX + 1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MAX + 1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_MAX + 1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_MAX + 1);
}

