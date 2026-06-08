/**
 * @file    sht30.h
 * @brief   SHT30 温湿度传感器驱动 (I2C)
 *
 * 通信协议: I2C, 7-bit 地址 0x44
 * 测量范围: -40~125°C, 0~100%RH
 * 精度: ±0.3°C, ±2%RH
 */

#ifndef SHT30_H
#define SHT30_H

#include "main.h"

/* SHT30 I2C 地址 (ADDR 引脚接 GND) */
#define SHT30_I2C_ADDR          0x44

/* 测量模式命令 */
#define SHT30_CMD_SINGLE_HIGH   0x2400  /* 单次测量，高重复性 (12.5ms) */
#define SHT30_CMD_SINGLE_MED    0x240B  /* 单次测量，中重复性 (6ms) */
#define SHT30_CMD_SINGLE_LOW    0x2416  /* 单次测量，低重复性 (3ms) */
#define SHT30_CMD_SOFT_RESET    0x30A2  /* 软件复位 */

/* 状态寄存器 */
#define SHT30_CMD_READ_STATUS   0xF32D
#define SHT30_CMD_CLEAR_STATUS  0x3041

/* 初始化 SHT30 */
int  sht30_init(I2C_TypeDef *i2c);

/* 单次测量，结果存入参数 */
int  sht30_measure(I2C_TypeDef *i2c, float *temperature, float *humidity);

/* 读取状态寄存器 */
int  sht30_read_status(I2C_TypeDef *i2c, uint16_t *status);

/* 软件复位 */
void sht30_reset(I2C_TypeDef *i2c);

/* CRC-8 校验 (多项式: 0x31, 初始值: 0xFF) */
uint8_t sht30_crc8(const uint8_t *data, int len);

#endif /* SHT30_H */
