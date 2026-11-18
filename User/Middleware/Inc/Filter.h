#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
  * @brief 一阶低通 + 防死锁限幅滤波器结构体
  */
typedef struct {
    float alpha;          // 一阶低通滤波系数 (0.0f ~ 1.0f)
    float max_step;       // 限幅滤波阈值 (0.0f 表示关闭限幅)
    float last_raw;       // 上一次合法的原始值
    float output;         // 当前滤波后的平滑输出值
    uint8_t is_first;     // 首次运行标志位
    uint8_t err_cnt;      // 连续超限计数器 (防止真实阶跃时滤波器死锁)
} LPF_Filter_t;

/**
  * @brief MPU6050 一阶互补滤波器结构体
  */
typedef struct {
    float alpha;          // 陀螺仪权重系数 (5ms下推荐 0.985f ~ 0.992f)
    float dt;             // 采样周期 (0.005f)
    float angle;          // 融合后的最优倾角 (deg)
    float gyro_rate;      // 滤波后的干净角速度 (deg/s，直接供直立环 D 项使用)
    uint8_t is_first;     // 首帧对齐标志位
} CompFilter_t;

/* ---------------- 全局滤波器实例声明 ---------------- */
extern LPF_Filter_t Filter_LeftMotor;
extern LPF_Filter_t Filter_RightMotor;

extern LPF_Filter_t Filter_AccelX;
extern LPF_Filter_t Filter_AccelY;
extern LPF_Filter_t Filter_AccelZ;

extern LPF_Filter_t Filter_GyroX;
extern LPF_Filter_t Filter_GyroY;
extern LPF_Filter_t Filter_GyroZ;

extern CompFilter_t Filter_Attitude;   // Pitch 俯仰角互补滤波器 (直立环核心)
extern CompFilter_t Filter_Roll;       // Roll  横滚角互补滤波器

/* ---------------- 一阶低通滤波 API ---------------- */
void  F_Init_Alpha(LPF_Filter_t *filter, float alpha, float max_step);
void  F_Init_Freq(LPF_Filter_t *filter, float cutoff_freq_hz, float sample_time_s, float max_step);
float F_Update(LPF_Filter_t *filter, float raw_input);
void  F_Reset(LPF_Filter_t *filter);

/* ---------------- 互补滤波融合 API ---------------- */
void  CF_Init_Alpha(CompFilter_t *cf, float alpha, float dt);
void  CF_Init_Tau(CompFilter_t *cf, float tau_s, float dt);
float CF_Update(CompFilter_t *cf, float acc_horizontal, float acc_z, float gyro_rate);
void  CF_Reset(CompFilter_t *cf);

/* ---------------- 平衡小车一键初始化 ---------------- */
void  Balance_Car_Filter_Init(void);

#endif /* __FILTER_H */