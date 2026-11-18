/************************************************
 * Developer : Gonk
 * Version   : V1.3.0
 * Updated   : 2026-11-14
 * Description: 编码器低通限幅滤波 + MPU6050 互补滤波融合
 ************************************************/

#include "Filter.h"
#include "fast_math.h"

/* 编码器滤波器 */
LPF_Filter_t Filter_LeftMotor;
LPF_Filter_t Filter_RightMotor;

/* MPU6050 六轴低通滤波器 */
LPF_Filter_t Filter_AccelX;
LPF_Filter_t Filter_AccelY;
LPF_Filter_t Filter_AccelZ;
LPF_Filter_t Filter_GyroX;
LPF_Filter_t Filter_GyroY;
LPF_Filter_t Filter_GyroZ;

/* 姿态互补滤波器 */
CompFilter_t Filter_Attitude;   // Pitch 俯仰角
CompFilter_t Filter_Roll;       // Roll  横滚角

void F_Init_Alpha(LPF_Filter_t *filter, float alpha, float max_step)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    filter->alpha    = alpha;
    filter->max_step = (max_step > 0.0f) ? max_step : 0.0f;
    filter->last_raw = 0.0f;
    filter->output   = 0.0f;
    filter->is_first = 1;
    filter->err_cnt  = 0;
}

void F_Init_Freq(LPF_Filter_t *filter, float cutoff_freq_hz, float sample_time_s, float max_step)
{
    if (cutoff_freq_hz <= 0.0f || sample_time_s <= 0.0f) 
    {
        F_Init_Alpha(filter, 1.0f, max_step);
        return;
    }

    float rc    = 1.0f / (2.0f * M_PI * cutoff_freq_hz);
    float alpha = sample_time_s / (sample_time_s + rc);

    F_Init_Alpha(filter, alpha, max_step);
}

float F_Update(LPF_Filter_t *filter, float raw_input)
{
    if (filter->is_first) 
    {
        filter->last_raw = raw_input;
        filter->output   = raw_input;
        filter->is_first = 0;
        filter->err_cnt  = 0;
        return filter->output;
    }

    if (filter->max_step > 0.0f) 
    {
        float diff = raw_input - filter->last_raw;
        if (diff < 0.0f) diff = -diff;
        
        if (diff > filter->max_step) 
        {
            filter->err_cnt++;
            if (filter->err_cnt <= 3) // 连续3次内判为干扰毛刺，丢弃
            {
                raw_input = filter->last_raw;
            }
            else                      // 连续超过3次判为真实阶跃，放行防死锁
            {
                filter->err_cnt = 0;
            }
        }
        else
        {
            filter->err_cnt = 0;
        }
    }
    filter->last_raw = raw_input;

    filter->output += filter->alpha * (raw_input - filter->output);
    return filter->output;
}

void F_Reset(LPF_Filter_t *filter)
{
    filter->last_raw = 0.0f;
    filter->output   = 0.0f;
    filter->is_first = 1;
    filter->err_cnt  = 0;
}

void CF_Init_Alpha(CompFilter_t *cf, float alpha, float dt)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    cf->alpha     = alpha;
    cf->dt        = (dt > 0.0f) ? dt : 0.005f;
    cf->angle     = 0.0f;
    cf->gyro_rate = 0.0f;
    cf->is_first  = 1;
}

void CF_Init_Tau(CompFilter_t *cf, float tau_s, float dt)
{
    if (tau_s <= 0.0f || dt <= 0.0f)
    {
        CF_Init_Alpha(cf, 0.98f, 0.005f);
        return;
    }
    float alpha = tau_s / (tau_s + dt);
    CF_Init_Alpha(cf, alpha, dt);
}

/**
  * @brief  互补滤波单轴融合 (传入已扣除零偏的 gyro_rate)
  * @param  acc_horizontal: 水平轴加速度 (Pitch 传入 -ax，Roll 传入 ay)
  * @param  acc_z: 垂直 Z 轴加速度
  * @param  gyro_rate: 对应轴角速度 (deg/s)
  */
float CF_Update(CompFilter_t *cf, float acc_horizontal, float acc_z, float gyro_rate)
{
    if (acc_horizontal == 0.0f && acc_z == 0.0f)
    {
        return cf->angle;
    }

    // 调用 fast_math 的 qatan2 快速反正切，大幅缩短 STM32F103 中断耗时
    float accel_angle = qatan2(acc_horizontal, acc_z) * 57.29578f;
    cf->gyro_rate     = gyro_rate;

    // 首次上电：直接对齐加速度计静态倾角，消除开机收敛抽搐
    if (cf->is_first)
    {
        cf->angle    = accel_angle;
        cf->is_first = 0;
        return cf->angle;
    }

    // 一阶互补融合
    cf->angle = cf->alpha * (cf->angle + cf->gyro_rate * cf->dt)
              + (1.0f - cf->alpha) * accel_angle;

    return cf->angle;
}

void CF_Reset(CompFilter_t *cf)
{
    cf->angle     = 0.0f;
    cf->gyro_rate = 0.0f;
    cf->is_first  = 1;
}

void Balance_Car_Filter_Init(void)
{
    // 1. 左右编码器
    F_Init_Alpha(&Filter_LeftMotor,  0.3f, 200.0f);
    F_Init_Alpha(&Filter_RightMotor, 0.3f, 200.0f);

    // 2. 加速度计三轴：5ms 采样下截止频率 8Hz，强力抑制电机/齿轮高频震动
    F_Init_Freq(&Filter_AccelX, 8.0f, 0.005f, 0.0f);
    F_Init_Freq(&Filter_AccelY, 8.0f, 0.005f, 0.0f);
    F_Init_Freq(&Filter_AccelZ, 8.0f, 0.005f, 0.0f);

    // 3. 陀螺仪三轴：5ms 采样下截止频率 35Hz，保留直立环 D 项快速响应
    F_Init_Freq(&Filter_GyroX, 35.0f, 0.005f, 0.0f);
    F_Init_Freq(&Filter_GyroY, 35.0f, 0.005f, 0.0f);
    F_Init_Freq(&Filter_GyroZ, 35.0f, 0.005f, 0.0f);

    // 4. 互补滤波器：时间常数 tau = 0.5s (5ms 下等效 alpha ≈ 0.9901)
    //    若习惯你原来的 0.98f，可改为 CF_Init_Alpha(&Filter_Attitude, 0.98f, 0.005f);
    CF_Init_Tau(&Filter_Attitude, 0.5f, 0.005f);
    CF_Init_Tau(&Filter_Roll,     0.5f, 0.005f);
}