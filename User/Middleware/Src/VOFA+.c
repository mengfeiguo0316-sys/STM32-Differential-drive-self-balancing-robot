/************************************************
 * Developer : Gonk
 * GitHub    : https://github.com/mengfeiguo0316-sys
 * Email     : mengfeiguo0316@gmail.com or 2250513147@qq.com
 * Phone     : 199 5063 9665
 *
 * Version   : V1.0.0
 * Updated   : 2026-11-12
 *
 * Description:VOFA+可视化数据发送模块，支持 FireWater 与 JustFloat 格式
 *   
 * Copyright (c) 2026 Gonk
 * All rights reserved.
 ************************************************/

#include "VOFA+.h"

#if VOFA_PLUS_ENABLE

#include "usart.h"
#include <stdio.h>
#include <string.h>

/* 绑定底层串口句柄（也可后续替换为 bsp_uart 接口） */
#define VOFA_UART_HANDLE  huart1

/* VOFA+ 内部发送缓冲区 */
static uint8_t vofa_send_buf[VOFA_SEND_BUF_MAX_LEN] = {0};

void VOFA_Plus_Init(void)
{
    memset(vofa_send_buf, 0, sizeof(vofa_send_buf));
}

/**
 * @brief 通用多通道浮点数据打包函数
 * @param channels: 浮点数据数组指针（不论是欧拉角还是电机转速，拼成数组传进来即可）
 * @param ch_num:   通道数量（float 元素个数）
 * @param send_buf: 输出字节缓冲区
 * @param buf_len:  缓冲区最大长度
 * @retval 实际打包的字节长度，失败返回 0
 */
uint16_t VOFA_Plus_PackData(const float *channels, uint8_t ch_num, uint8_t *send_buf, uint16_t buf_len)
{
    if (channels == NULL || send_buf == NULL || ch_num == 0 || ch_num > VOFA_MAX_CHANNELS)
    {
        return 0;
    }

#if (VOFA_PROTOCOL == 1) /* FireWater 模式：格式化为 "ch0,ch1,ch2\r\n" */
    uint16_t offset = 0;

    for (uint8_t i = 0; i < ch_num; i++)
    {
        // 最后一个数据后面加 "\r\n"，其余数据后面加 ","
        const char *fmt = (i == (ch_num - 1)) ? "%.2f\r\n" : "%.2f,";
        int ret = snprintf((char *)(send_buf + offset), buf_len - offset, fmt, channels[i]);

        if (ret < 0 || (offset + ret) >= buf_len)
        {
            return 0; // 缓冲区空间不足，防溢出保护
        }
        offset += (uint16_t)ret;
    }
    return offset;

#elif (VOFA_PROTOCOL == 2) /* JustFloat 模式：N 个 float 二进制流 + 4字节帧尾 */
    uint16_t data_bytes = ch_num * sizeof(float);
    if (buf_len < (data_bytes + 4))
    {
        return 0; // 缓冲区过小
    }

    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

    // 1. 直接内存拷贝 N 个 float
    memcpy(send_buf, channels, data_bytes);
    // 2. 追加 JustFloat 标准帧尾
    memcpy(send_buf + data_bytes, tail, 4);

    return (data_bytes + 4);

#else
    return 0;
#endif
}

/**
 * @brief 通用多通道数据发送函数
 * @param channels: 浮点数据数组指针
 * @param ch_num:   通道数量
 */
void VOFA_Plus_SendData(const float *channels, uint8_t ch_num)
{
    uint16_t data_len = VOFA_Plus_PackData(channels, ch_num, vofa_send_buf, VOFA_SEND_BUF_MAX_LEN);
    if (data_len == 0)
    {
        return;
    }

    HAL_UART_Transmit_DMA(&VOFA_UART_HANDLE, vofa_send_buf, data_len);
}

#endif /* VOFA_PLUS_ENABLE */