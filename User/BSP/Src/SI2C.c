/************************************************
 * Developer : Gonk
 * GitHub    : https://github.com/mengfeiguo0316-sys
 * Email     : mengfeiguo0316@gmail.com or 2250513147@qq.com
 * Phone     : 199 5063 9665
 *
 * Version   : V1.0.0
 * Updated   : 2026-11-12
 *
 * Description:软件 I2C 总线驱动，支持单字节与多字节读写
 *   
 * Copyright (c) 2026 Gonk
 * All rights reserved.
 ************************************************/

#include "SI2C.h"
#include"Delay.h"

/* 软件 I2C 实现核心机制：SCL 低电平期间切换 SDA，SCL 高电平期间保持并采样 SDA */
// 封装引脚，这里为开漏输出高电平靠外部上拉电阻
#define SDA_RELEASE()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define SDA_L()        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define SDA_READ()     HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)

#define SCL_RELEASE()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define SCL_L()        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

/**
 * @brief  产生 I2C 起始信号 / 重复起始信号
 * @retval 0：成功；1：总线忙
 */
static uint8_t SI2C_Start(void)
{
    SDA_RELEASE();          // 先释放 SDA
    Delay_us(2);
    SCL_RELEASE();          // 释放 SCL，拉高
    Delay_us(5);            // 等待总线稳定

    if (!SDA_READ())  return 1;  // 检测 SDA 是否被持续拉低，若低说明总线被占死

    SDA_L();                // SCL 为高时拉低 SDA，产生 Start 条件
    Delay_us(5);

    SCL_L();                // 钳住 SCL，准备传输数据
    Delay_us(5);
    return 0;
}

/**
 * @brief  产生 I2C 停止信号
 */
static void SI2C_Stop(void)
{
    SDA_L();                // 确保 SDA 为低
    Delay_us(2);
    SCL_RELEASE();          // 释放 SCL（拉高）
    Delay_us(5);

    SDA_RELEASE();          // SCL 高电平期间拉高 SDA，产生 Stop 条件
    Delay_us(5);
}

/**
 * @brief  发送一个字节（高位在前）
 * @param  byte：要发送的 8 位数据
 */
static void SI2C_SendByte(uint8_t byte)
{
    for (int i = 0; i < 8; i++)
    {
        if (byte & (0x80 >> i))
            SDA_RELEASE();  // 输出 1（释放引脚由外部上拉）
        else
            SDA_L();        // 输出 0（强制拉低）
        Delay_us(5);

        SCL_RELEASE();      // 拉高 SCL，通知从机读取
        Delay_us(5);

        SCL_L();            // 拉低 SCL，准备下一位
        Delay_us(5);
    }
}

/**
 * @brief  发送 ACK 应答信号
 */
static void SI2C_SendAck(void)
{
    SDA_L();                // 主机拉低 SDA 表示 ACK
    Delay_us(2);

    SCL_RELEASE();          // 拉高 SCL 使应答有效
    Delay_us(5);

    SCL_L();                // 拉低 SCL 结束应答
    Delay_us(5);

    SDA_RELEASE();          // 释放 SDA
    Delay_us(2);
}

/**
 * @brief  发送 NACK 非应答信号
 */
static void SI2C_SendNack(void)
{
    SDA_RELEASE();          // 主机释放 SDA（高电平）表示 NACK
    Delay_us(2);

    SCL_RELEASE();          // 拉高 SCL
    Delay_us(5);

    SCL_L();                // 拉低 SCL
    Delay_us(5);
}

/**
 * @brief  等待从机应答
 * @retval 0：收到应答；1：超时未应答
 */
static uint8_t SI2C_WaitAck(void)
{
    uint16_t time_out = 1000;

    SDA_RELEASE();          // 主机释放 SDA，交由从机控制
    Delay_us(2);

    SCL_RELEASE();          // 拉高 SCL 准备采样
    Delay_us(5);

    while (SDA_READ())      // 从机若应答会主动拉低 SDA
    {
        if (--time_out == 0)
        {
            SCL_L();
            SI2C_Stop();    // 超时释放总线
            return 1;
        }
        Delay_us(1);
    }

    SCL_L();                // 采样完毕，拉低 SCL
    Delay_us(5);
    return 0;
}

/**
 * @brief  接收一个字节（高位在前）
 * @retval 读取到的字节
 */
static uint8_t SI2C_ReceiveByte(void)
{
    uint8_t data = 0x00;

    SDA_RELEASE();          // 释放 SDA，等待从机发送数据

    for (int i = 0; i < 8; i++)
    {
        SCL_RELEASE();      // 拉高 SCL 进行采样
        Delay_us(5);

        if (SDA_READ())     // 读取引脚状态
        {
            data |= (0x80 >> i);
        }

        SCL_L();            // 采样完毕立刻拉低 SCL，触发从机发送下一位
        Delay_us(5);
    }

    return data;
}

/**
 * @brief  向设备指定寄存器写入单字节
 * @param  addr：7 位设备地址（如 0x68）
 * @param  reg ：寄存器地址
 * @param  data：写入的数据
 * @retval 0：成功；1：失败
 */
uint8_t SI2C_Write(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (SI2C_Start() != 0) return 1;

    SI2C_SendByte(addr << 1);           // 7 位地址 + 写标志 (0)
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    SI2C_SendByte(reg);                 // 发送寄存器地址
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    SI2C_SendByte(data);                // 发送数据
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    SI2C_Stop();
    return 0;
}

/**
 * @brief  从设备指定寄存器读取单字节
 * @param  addr：7 位设备地址（如 0x68）
 * @param  reg ：寄存器地址
 * @param  data：存储数据的指针
 * @retval 0：成功；1：失败
 */
uint8_t SI2C_Read(uint8_t addr, uint8_t reg, uint8_t *data)
{
    if (SI2C_Start() != 0) return 1;

    SI2C_SendByte(addr << 1);           // 7 位地址 + 写标志
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    SI2C_SendByte(reg);                 // 发送寄存器地址
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    // 重复起始条件，切换到读模式
    if (SI2C_Start() != 0) return 1;

    SI2C_SendByte((addr << 1) | 0x01);  // 7 位地址 + 读标志 (1)
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    *data = SI2C_ReceiveByte();
    SI2C_SendNack();                    // 单字节读取完毕发送 NACK
    SI2C_Stop();
    return 0;
}

/**
 * @brief  从设备连续读取多个字节
 * @param  addr  ：7 位设备地址（如 0x68）
 * @param  reg   ：起始寄存器地址
 * @param  buffer：接收缓冲区
 * @param  length：读取长度
 * @retval 0：成功；1：失败
 */
uint8_t SI2C_ReadBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length)
{
    if (buffer == 0 || length == 0) return 1;

    if (SI2C_Start() != 0) return 1;

    SI2C_SendByte(addr << 1);           // 7 位地址 + 写标志
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    SI2C_SendByte(reg);                 // 寄存器地址
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    if (SI2C_Start() != 0) return 1;

    SI2C_SendByte((addr << 1) | 0x01);  // 7 位地址 + 读标志
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    for (int i = 0; i < length; i++)
    {
        buffer[i] = SI2C_ReceiveByte();
        if (i < length - 1)
            SI2C_SendAck();             // 非末尾字节回复 ACK 继续读
        else
            SI2C_SendNack();            // 末尾字节回复 NACK 结束
    }

    SI2C_Stop();
    return 0;
}

/**
 * @brief  向设备连续写入多个字节
 * @param  addr  ：7 位设备地址（如 0x68）
 * @param  reg   ：起始寄存器地址
 * @param  data  ：待写入数据缓冲区
 * @param  length：写入长度
 * @retval 0：成功；1：失败
 */
uint8_t SI2C_WriteBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (data == 0 || length == 0) return 1;

    if (SI2C_Start() != 0) return 1;

    SI2C_SendByte(addr << 1);
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    SI2C_SendByte(reg);
    if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }

    for (int i = 0; i < length; i++)
    {
        SI2C_SendByte(data[i]);
        if (SI2C_WaitAck() != 0) { SI2C_Stop(); return 1; }
    }

    SI2C_Stop();
    return 0;
}

/**
 * @brief  修改指定寄存器的单个位
 */
uint8_t SI2C_writeBit(uint8_t addr, uint8_t reg, uint8_t location, uint8_t level)
{
    uint8_t val = 0;
    if (location > 7) return 1;
    if (SI2C_Read(addr, reg, &val) != 0) return 1;

    if (level)
        val |= (1 << location);
    else
        val &= ~(1 << location);

    return SI2C_Write(addr, reg, val);
}

/**
 * @brief  修改指定寄存器的连续多个位
 */
uint8_t SI2C_writeBits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t regValue = 0;
    uint8_t mask;

    if (length == 0 || length > 8 || bitStart > 7 || (bitStart + length) > 8)
        return 1;

    if (SI2C_Read(addr, reg, &regValue) != 0)
        return 1;

    mask = (length == 8) ? 0xFF : (((1 << length) - 1) << bitStart);
    regValue &= ~mask;
    regValue |= (data & ((1 << length) - 1)) << bitStart;

    return SI2C_Write(addr, reg, regValue);
}