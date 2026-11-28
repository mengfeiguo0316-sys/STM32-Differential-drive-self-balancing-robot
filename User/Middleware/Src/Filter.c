/************************************************
 * Developer : Gonk
 * Version   : V1.3.0
 * Updated   : 2026-11-18
 * Description: 滤波器模块，负责对传感器数据进行滤波处理，提供平滑的输出数据
 ************************************************/

#include "Filter.h"
#include "fast_math.h"

LPF_Filter_t Filter_AccelX;
LPF_Filter_t Filter_AccelY;
LPF_Filter_t Filter_AccelZ;
CompFilter_t pitch;
CompFilter_t row;


/**
 * @brief  一阶互补滤波融合器初始化函数
 * @param  cf : 一阶互补滤波器结构体指针
 * @param  alpha : 陀螺仪权重系数 (5ms下推荐 0.985f ~ 0.992f)
 * @param  dt : 采样周期 
 */
static void C_Init(CompFilter_t *cf, float alpha, float dt)
{
    cf->alpha = alpha;
    cf->dt = dt;
    cf->angle = 0.0f;
    cf->is_first = 1; 
}

/**
 * @brief  一阶互补滤波融合器
 * @param  acc_m : 加速度计解算的角度 (deg)
 * @param  gyro_m : 陀螺仪解算的角度 (deg)
 * @retval 融合后的最优倾角 (deg)
 */
float CF_Update(CompFilter_t *cf, float acc_hor, float acc_z, float gyro_rate)
{
    if (acc_hor == 0.0f && acc_z == 0.0f)
        return cf->angle;

    float accel_angle = qatan2(acc_hor, acc_z) * 57.29578f;
    cf->gyro_rate     = gyro_rate;

    if (cf->is_first)
    {
        cf->angle    = accel_angle;
        cf->is_first = 0;
        return cf->angle;
    }

    cf->angle = cf->alpha * (cf->angle + cf->gyro_rate * cf->dt)
              + (1.0f - cf->alpha) * accel_angle;

    return cf->angle;
}


/* 一阶低通初始化 */
static void LPF_Init_Freq(LPF_Filter_t *f, float cutoff_hz, float dt)
{
    float rc    = 1.0f / (2.0f * M_PI * cutoff_hz);
    f->alpha    = dt / (dt + rc);
    f->output   = 0.0f;
    f->is_first = 1;
}

/**
 * @brief  一阶低通滤波器更新函数
 * @param raw_input 原始输入值
 * @return 滤波后的输出值
 */
float LPF_Update(LPF_Filter_t *f, float raw_input)
{
    if (f->is_first)
    {
        f->output   = raw_input;
        f->is_first = 0;
        return f->output;
    }
    f->output += f->alpha * (raw_input - f->output);
    return f->output;
}


/**
 * @brief  一键初始化所有滤波器（务必在 MPU6050_Init 中调用一次！）
 */
void Filter_Init(void)
{
    LPF_Init_Freq(&Filter_AccelX, 8.0f, 0.005f);
    LPF_Init_Freq(&Filter_AccelY, 8.0f, 0.005f);
    LPF_Init_Freq(&Filter_AccelZ, 8.0f, 0.005f);

    C_Init(&pitch, 0.98f, 0.005f);
    C_Init(&row,  0.98f, 0.005f);
}
