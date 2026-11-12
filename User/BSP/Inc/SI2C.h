#ifndef __SI2C_H
#define __SI2C_H
#include "stm32f1xx_hal.h"

uint8_t SI2C_Write(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t SI2C_CheckAddr(uint8_t addr);
uint8_t SI2C_Read(uint8_t addr, uint8_t reg, uint8_t *data);
uint8_t SI2C_ReadBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length);
uint8_t SI2C_WriteBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length);
uint8_t SI2C_writeBit(uint8_t addr, uint8_t reg, uint8_t location, uint8_t level);
uint8_t SI2C_writeBits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);

#endif