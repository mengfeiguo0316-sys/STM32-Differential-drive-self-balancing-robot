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
    uint8_t err_cnt;      // 连续超限计数器 (防止真实阶跃时滤波器死锁)
    float max_step;       // 限幅滤波阈值 (0.0f 表示关闭限幅)
    float last_raw;       // 上一次合法的原始值
    
    float alpha;          // 一阶低通滤波系数 (0.0f ~ 1.0f)
    float output;         // 当前滤波后的平滑输出值
    uint8_t is_first;     // 首次运行标志位
    
} LPF_Filter_t;

/**
  * @brief一阶互补滤波器结构体
  */
typedef struct {
    float alpha;          // 陀螺仪权重系数 (5ms下推荐 0.985f ~ 0.992f)
    float dt;             // 采样周期 (0.005f)
    float angle;          // 融合后的最优倾角 (deg)
    float gyro_rate;      // 对应轴角速度 (deg/s，直接供直立环 D 项使用)
    uint8_t is_first;     // 首帧对齐标志位
} CompFilter_t;

extern CompFilter_t pitch;
extern CompFilter_t row;
extern LPF_Filter_t Filter_AccelX;
extern LPF_Filter_t Filter_AccelY;
extern LPF_Filter_t Filter_AccelZ;


void Filter_Init(void);
float LPF_Update(LPF_Filter_t *f, float raw_input);
float CF_Update(CompFilter_t *cf, float acc_hor, float acc_z, float gyro_rate);

#endif /* __FILTER_H */