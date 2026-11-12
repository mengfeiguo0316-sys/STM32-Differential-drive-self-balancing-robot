#ifndef __FAST_MATH_H__
#define __FAST_MATH_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * STM32F103 高性能快速三角函数库 (专为姿态解算设计)
 * 特性：脱离 <math.h>，耗时为标准库的 1/10 左右，误差 < 0.03 度
 */

float qsin(float x);
float qcos(float x);
float qtan(float x);
float qasin(float x);
float qacos(float x);
float qatan(float x);
float qatan2(float y, float x);

#ifdef __cplusplus
}
#endif

#endif /* __FAST_TRIG_H__ */