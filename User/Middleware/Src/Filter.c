/************************************************
 * Developer : Gonk
 * Version   : V1.3.0
 * Updated   : 2026-11-18
 * Description: 滤波器模块，负责对传感器数据进行滤波处理，提供平滑的输出数据
 ************************************************/

#include "Filter.h"
#include "fast_math.h"

// 全局滤波器实例
LPF_Filter_t lpf_ax;
LPF_Filter_t lpf_ay;
LPF_Filter_t lpf_az;

CompFilter_t Pitch;
CompFilter_t Roll;


/**
 * @brief  一阶低通滤波器初始化函数
 * @param filter 滤波器指针
 * @param cutoff_freq_hz 截止频率 (Hz)
 * @param sample_time_s 采样周期 (s)
 */
void F_Init_Freq(LPF_Filter_t *filter, float cutoff_freq_hz, float sample_time_s)
{
    //根据物理公式计算时间常数 RC 和滤波系数 alpha
    float rc      = 1.0f / (2.0f * M_PI * cutoff_freq_hz);
    filter->alpha = sample_time_s / (sample_time_s + rc);

    filter->output   = 0.0f;
    filter->is_first = 1;
    filter->max_step = 0.0f;
    filter->last_raw = 0.0f;
    filter->err_cnt  = 0;
}

/**
 * @brief  一阶低通滤波器更新函数
 * @param filter 滤波器指针
 * @param raw_input 原始输入值
 * @return 滤波后的输出值
 */
float F_Update(LPF_Filter_t *filter, float raw_input)
{
    // 上电第一帧，直接让输出等于当前输入，避免从 0 开始缓慢爬升
    if (filter->is_first)
    {
        filter->output   = raw_input;
        filter->is_first = 0;
        return filter->output;
    }

    //一阶低通核心公式：新输出 = 旧输出 + alpha * (新输入 - 旧输出)
    filter->output += filter->alpha * (raw_input - filter->output);

    return filter->output;
}

/**
 * @brief  互补滤波器初始化函数
 * @param filter 滤波器指针
 * @param alpha 互补滤波器参数
 * @param dt 采样周期
 */
void CF_Init(CompFilter_t *cf, float alpha, float dt)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    cf->alpha     = alpha;
    cf->dt        = (dt > 0.0f) ? dt : 0.005f;
    cf->angle     = 0.0f;
    cf->gyro_rate = 0.0f;
    cf->is_first  = 1;
}


/**
 * @brief  一阶互补滤波单步融合
 * @param  cf        : 互补滤波器实例指针
 * @param  acc_hor   : 水平方向加速度 (算 Pitch 传 -ax，算 Roll 传 ay)
 * @param  acc_z     : 垂直方向加速度 (az)
 * @param  gyro_rate : 对应轴角速度 (deg/s，已扣除零偏)
 * @retval 融合后的最优倾角 (deg)
 */
float CF_Update(CompFilter_t *cf, float acc_hor, float acc_z, float gyro_rate)
{
    // 1. 防止除零异常
    if (acc_hor == 0.0f && acc_z == 0.0f)
    {
        return cf->angle;
    }

    // 2. 用加速度计解算静态倾角 (弧度转角度: 180 / PI ≈ 57.29578)
    float accel_angle = qatan2(acc_hor, acc_z) * 57.29578f;

    // 3. 保存当前角速度，供外部直立环 D 项直接读取
    cf->gyro_rate = gyro_rate;

    // 4. 上电第一帧：直接信任加速度计倾角，避免从 0° 缓慢爬升导致电机猛抽
    if (cf->is_first)
    {
        cf->angle    = accel_angle;
        cf->is_first = 0;
        return cf->angle;
    }

    // 5. 核心互补融合公式：短期动态信陀螺仪积分，长期基准靠加速度计拉回
    cf->angle = cf->alpha * (cf->angle + cf->gyro_rate * cf->dt)
              + (1.0f - cf->alpha) * accel_angle;

    return cf->angle;
}