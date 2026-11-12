#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>

/* ==================== 核心宏开关 ==================== */
// 开启/关闭 VOFA+ 调试功能：调试时置 1，量产时置 0
#define VOFA_PLUS_ENABLE        1

// 协议选择：1 = FireWater (字符串格式，直观易读)
//           2 = JustFloat (二进制浮点流 + 帧尾，高频发送效率最高，推荐！)
#define VOFA_PROTOCOL           1

// 支持的最大浮点通道数与发送缓冲区大小
#define VOFA_MAX_CHANNELS       16
#define VOFA_SEND_BUF_MAX_LEN   128

#if VOFA_PLUS_ENABLE

/* ==================== 对外通用 API ==================== */
void VOFA_Plus_Init(void);

/**
 * @brief 通用多通道浮点数据打包函数
 * @param channels: 浮点数据数组指针（不论是欧拉角还是电机转速，拼成数组传进来即可）
 * @param ch_num:   通道数量（float 元素个数）
 * @param send_buf: 输出字节缓冲区
 * @param buf_len:  缓冲区最大长度
 * @retval 实际打包的字节长度，失败返回 0
 */
uint16_t VOFA_Plus_PackData(const float *channels, uint8_t ch_num, uint8_t *send_buf, uint16_t buf_len);

/**
 * @brief 通用多通道数据发送函数
 * @param channels: 浮点数据数组指针
 * @param ch_num:   通道数量
 */
void VOFA_Plus_SendData(const float *channels, uint8_t ch_num);

#else
/* ==================== 空实现（关闭宏时零开销） ==================== */
#define VOFA_Plus_Init()                                           ((void)0)
#define VOFA_Plus_PackData(channels, ch_num, send_buf, buf_len)    ((uint16_t)0)
#define VOFA_Plus_SendData(channels, ch_num)                       ((void)0)

#endif /* VOFA_PLUS_ENABLE */

#endif /* __VOFA_H */