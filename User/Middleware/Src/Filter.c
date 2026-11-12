/************************************************
 * Developer : Gonk
 * GitHub    : https://github.com/mengfeiguo0316-sys
 * Email     : mengfeiguo0316@gmail.com or 2250513147@qq.com
 * Phone     : 199 5063 9665
 *
 * Version   : V1.0.0
 * Updated   : 2026-11-12
 *
 * Description:一阶低通滤波器实现，包含限幅与时间常数配置
 *   
 * Copyright (c) 2026 Gonk
 * All rights reserved.
 ************************************************/

#include "Filter.h"

LPF_Filter_t Filter_LeftMotor;   // 左轮编码器滤波器
LPF_Filter_t Filter_RightMotor;  // 右轮编码器滤波器
LPF_Filter_t Filter_GyroY;       // MPU6050 直立环角速度滤波器
LPF_Filter_t Filter_AccelX;      // MPU6050 加速度计滤波器

/**
  * @brief  直接系数 Alpha 初始化滤波器
  * @param  filter: 滤波器实例指针
  * @param  alpha: 滤波系数 (0.0 ~ 1.0)，越小越平滑但延迟越大
  * @param  max_step: 限幅阈值 (若不需要限幅，传 0.0f 即可)
  */
void F_Init_Alpha(LPF_Filter_t *filter, float alpha, float max_step)
{
    // 安全边界限制
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    filter->alpha = alpha;
    filter->max_step = (max_step > 0.0f) ? max_step : 0.0f;
    filter->last_raw = 0.0f;
    filter->output = 0.0f;
    filter->is_first = 1;
}

/**
  * @brief  物理截止频率初始化（高阶玩法：适合各种采样率）
  * @param  filter: 滤波器实例指针
  * @param  cutoff_freq_hz: 截止频率 (Hz)，高于此频率的震动将被大幅衰减
  * @param  sample_time_s: 采样周期 (单位：秒，例如 10ms 传入 0.01f)
  * @param  max_step: 限幅阈值 (不需要则传 0.0f)
  */
void F_Init_Freq(LPF_Filter_t *filter, float cutoff_freq_hz, float sample_time_s, float max_step)
{
    if (cutoff_freq_hz <= 0.0f || sample_time_s <= 0.0f) 
    {
        F_Init_Alpha(filter, 1.0f, max_step); // 参数非法则默认不滤波
        return;
    }

    // RC 时间常数 τ = 1 / (2 * π * fc)
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq_hz);
    // 离散化计算 alpha = dt / (dt + rc)
    float alpha = sample_time_s / (sample_time_s + rc);

    F_Init_Alpha(filter, alpha, max_step);
}

/**
  * @brief  执行一次限幅+一阶低通滤波计算
  * @param  filter: 滤波器实例指针
  * @param  raw_input: 传感器本次读取的原始值
  * @retval 滤波后的干净数据
  */
float F_Update(LPF_Filter_t *filter, float raw_input)
{
    // 1. 首次上电对齐：直接信任第一次读数，防止开机数据从 0 缓慢爬升导致小车抽搐
    if (filter->is_first) 
    {
        filter->last_raw = raw_input;
        filter->output = raw_input;
        filter->is_first = 0;
        return filter->output;
    }

    // 2. 限幅滤波（防大错）：拦截电磁干扰导致的瞬间极值毛刺
    if (filter->max_step > 0.0f) 
    {
        float diff = raw_input - filter->last_raw;
        if (diff < 0.0f) diff = -diff; // 快速取绝对值
        
        if (diff > filter->max_step) 
        {
            raw_input = filter->last_raw; // 突变超标，丢弃本次异常值
        }
    }
    filter->last_raw = raw_input;

    // 3. 一阶低通滤波（抹小错）：Y(n) = α * X(n) + (1 - α) * Y(n-1)
    // 变形为 1 次乘法 + 2 次加减法，减轻无 FPU 单片机(如STM32F103)的运算负担
    filter->output += filter->alpha * (raw_input - filter->output);

    return filter->output;
}

/**
  * @brief  复位滤波器状态
  */
void F_Reset(LPF_Filter_t *filter)
{
    filter->last_raw = 0.0f;
    filter->output = 0.0f;
    filter->is_first = 1;
}

void Balance_Car_Filter_Init(void)
{
    // 左右编码器：沿用你调好的参数，alpha = 0.3，限幅阈值 = 200 RPM
    F_Init_Alpha(&Filter_LeftMotor,  0.3f, 200.0f);
    F_Init_Alpha(&Filter_RightMotor, 0.3f, 200.0f);

    // MPU6050 陀螺仪 Y 轴（负责直立环 D 项阻尼）：
    // 对延迟极其敏感！假设 5ms(0.005s) 采样一次，设置截止频率为 30Hz，不设限幅(0.0f)
    F_Init_Freq(&Filter_GyroY, 30.0f, 0.005f, 0.0f);

    // MPU6050 加速度计 X 轴（车体震动噪声极大）：
    // 需要重度滤波，5ms 采样下设置截止频率 5Hz，压制车轮齿轮震动传上来的高频毛刺
    F_Init_Freq(&Filter_AccelX, 5.0f, 0.005f, 0.0f);
}