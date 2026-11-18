#ifndef __VOFA_H
#define __VOFA_H

#include "MPU6050.h"
#include <stdint.h>


/* ==================== 核心宏开关 ==================== */
// 开启/关闭VOFA+调试功能，调试时置1，量产时置0
#define VOFA_PLUS_ENABLE        1
#if VOFA_PLUS_ENABLE

/* ==================== 硬件配置（根据工程实际修改） ==================== */
//  VOFA协议选择：1=FireWater（字符串格式，易调试），2=RawData
#define VOFA_PROTOCOL           1  // 默认选FireWater，设为2可切换为RawData（若未实现）
// 3. 发送缓冲区最大长度，单包数据量大时建议>=32字节
#define VOFA_SEND_BUF_MAX_LEN   128

// 重命名MPU6050_DebugData_t为VOFA_Data_t，避免重复定义
typedef MPU6050_Data_t VOFA_Data_t;

/* ==================== 函数声明 ==================== */
void VOFA_Plus_Init(void);
void VOFA_Plus_SendData(VOFA_Data_t *data);
void VOFA_Plus_SendMPU6050Data(void);
#else
/* ==================== 空实现（当VOFA+功能关闭时防止编译报错） ==================== */
#define VOFA_Plus_Init()                                  ((void)0)
#define VOFA_Plus_PackData(vofa_data, send_buf, buf_len)  ((uint16_t)0)
#define VOFA_Plus_SendData(data)                          ((void)0)
#define VOFA_Plus_SendMPU6050Data()                       ((void)0)

#endif /* VOFA_PLUS_ENABLE */ 

#endif /* __VOFA_H */