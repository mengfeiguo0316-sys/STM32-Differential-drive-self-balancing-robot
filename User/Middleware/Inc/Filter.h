#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
  * @brief 工业级通用一阶低通+限幅滤波器结构体 (面向对象封装)
  */
typedef struct {
    float alpha;          // 一阶低通滤波系数 (0.0f ~ 1.0f)
    float max_step;       // 限幅滤波阈值 (0.0f 表示关闭限幅，>0 表示允许的最大单次跳变)
    float last_raw;       // 上一次合法的原始值 (用于限幅对比)
    float output;         // 当前滤波后的平滑输出值
    uint8_t is_first;     // 首次运行标志位 (消除开机从0缓慢爬升的滞后)
} LPF_Filter_t;

/* ---------------- 核心 API 接口 ---------------- */

// 方式1：直接用 alpha 系数初始化（适合快速盲调）
void F_Init_Alpha(LPF_Filter_t *filter, float alpha, float max_step);

// 方式2：按物理截止频率(Hz)和采样周期(s)初始化（平衡小车MPU6050强烈推荐！）
void F_Init_Freq(LPF_Filter_t *filter, float cutoff_freq_hz, float sample_time_s, float max_step);

// 核心更新函数：传入原始数据，返回滤波后数据
float F_Update(LPF_Filter_t *filter, float raw_input);

// 状态复位函数：小车倒地保护停机再扶起时，清空历史记忆
void F_Reset(LPF_Filter_t *filter);

#endif /* __LPF_FILTER_H */