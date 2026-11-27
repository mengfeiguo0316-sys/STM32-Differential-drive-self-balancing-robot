#include "VOFA+.h"

#if VOFA_PLUS_ENABLE

#include "usart.h"
#include"stdio.h"
#include"string.h"
#define VOFA_UART_HANDLE        huart1  
// VOFA+ 数据发送缓冲区
static uint8_t vofa_send_buf[VOFA_SEND_BUF_MAX_LEN] = {0};

/**
 * @brief  VOFA+ 初始化
 * @note   清空发送缓冲区，通常在 main.c 中调用
 */
void VOFA_Plus_Init(void)
{
    memset(vofa_send_buf, 0, sizeof(vofa_send_buf));
}

/**
 * @brief VOFA+ 打包数据（支持 FireWater 或 RawData 模式）
 * @param vofa_data: 待发送的数据结构体指针
 * @param send_buf: 输出缓冲区
 * @param buf_len: 缓冲区长度
 * @retval 打包后的数据长度，失败返回 0
 */
uint16_t VOFA_Plus_PackData(VOFA_Data_t *vofa_data, uint8_t *send_buf, uint16_t buf_len)
{
    if (buf_len < sizeof(VOFA_Data_t))
    {
        return 0; // 缓冲区过小
    }

#if VOFA_PROTOCOL == 1 // FireWater 模式
    // 检查缓冲区是否足够容纳格式化字符串（至少 32 字节）
    if (buf_len < 32)
    {
        return 0; // 缓冲区过小
    }

    // FireWater 模式：按指定格式输出文本，例如 "x:100.0,y:95.5,z:20.0\r\n"
    uint16_t len = snprintf((char *)send_buf, buf_len, "%.1f,%.1f,%.1f\r\n",
                            vofa_data->pitch,
                            vofa_data->row,
                            vofa_data->yaw);
    return len; // 返回实际写入长度

#elif VOFA_PROTOCOL == 2 // RawData 模式

    // RawData 模式：直接发送二进制数据，需确保缓冲区足够存放 4 个 float
    if (buf_len < 16)
    {             // 4 个 float 共 16 字节
        return 0; // 缓冲区过小
    }
    // 依次拷贝各个 float 数据到发送缓冲区
    memcpy(send_buf, &vofa_data->target_rpm, sizeof(float));                       // 目标转速
    memcpy(send_buf + sizeof(float), &vofa_data->actual_rpm, sizeof(float));       // 实际转速
    memcpy(send_buf + sizeof(float) * 2, &vofa_data->pid_output, sizeof(float));   // PID 输出
    memcpy(send_buf + sizeof(float) * 3, &vofa_data->final_output, sizeof(float)); // 最终输出（含补偿）
    return sizeof(VOFA_Data_t);                                                    // 返回总数据长度
#else                    // 默认使用 RawData 模式
    return 0; // 不支持的协议
#endif
}

/**
 * @brief  VOFA+ 发送数据（通过串口）
 * @note   此函数会调用打包函数，然后通过 HAL_UART_Transmit 发送，超时 10ms
 *         通常在 speed_loop.c 等控制循环中调用
 */
void VOFA_Plus_SendData(VOFA_Data_t *data)
{
    if (data == NULL)
    {
        return; // 空指针检查
    }

    // 调用打包函数获取数据长度
    uint16_t data_len = VOFA_Plus_PackData(data, vofa_send_buf, VOFA_SEND_BUF_MAX_LEN);
    if (data_len == 0) return; // 打包失败则退出

    // 通过串口发送数据
    HAL_UART_Transmit(&VOFA_UART_HANDLE, vofa_send_buf, data_len, 10); // 超时 10ms
}

/**
 * @brief  从 MPU6050 获取调试数据并通过 VOFA+ 发送
 * @note   此函数在 MPU6050.c 中调用，用于发送姿态数据（俯仰、横滚、偏航）
 */
void VOFA_Plus_SendMPU6050Data(void)
{
    VOFA_Data_t vofa_data = MPU6050_GetDebugData();
    VOFA_Plus_SendData(&vofa_data);
}
#endif