/**
 * @file    sht30.c
 * @brief   SHT30 温湿度传感器驱动实现 (I2C)
 *
 * 数据转换公式（参考 SHT30 数据手册）：
 *   Temperature(°C) = -45 + 175 * (S_T / (2^16 - 1))
 *   Humidity(%)     = 100 * (S_RH / (2^16 - 1))
 */

#include "sht30.h"
#include "uart_dma.h"

/* ========== CRC-8 校验表 (多项式: 0x31, 初始值: 0xFF) ========== */
uint8_t sht30_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ========== I2C 发送命令 ========== */
static int sht30_send_command(I2C_TypeDef *i2c, uint16_t cmd)
{
    uint8_t buf[2];
    buf[0] = (cmd >> 8) & 0xFF;
    buf[1] = cmd & 0xFF;

    /* 生成起始条件 */
    I2C_GenerateSTART(i2c, ENABLE);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT));

    /* 发送器件地址 (写) */
    I2C_Send7bitAddress(i2c, SHT30_I2C_ADDR << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* 发送命令 */
    for (int i = 0; i < 2; i++) {
        I2C_SendData(i2c, buf[i]);
        while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }

    /* 停止 */
    I2C_GenerateSTOP(i2c, ENABLE);
    return 0;
}

/* ========== 初始化 SHT30 ========== */
int sht30_init(I2C_TypeDef *i2c)
{
    I2C_InitTypeDef i2c_init;

    /* I2C 配置: 100kHz, 7-bit 地址 */
    i2c_init.I2C_ClockSpeed = 100000;
    i2c_init.I2C_Mode = I2C_Mode_I2C;
    i2c_init.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c_init.I2C_OwnAddress1 = 0x00;
    i2c_init.I2C_Ack = I2C_Ack_Enable;
    i2c_init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(i2c, &i2c_init);
    I2C_Cmd(i2c, ENABLE);

    /* 等待传感器上电（最大 1ms） */
    for (volatile int i = 0; i < 72000; i++);

    /* 软件复位 */
    sht30_reset(i2c);

    /* 复位后等待 2ms */
    for (volatile int i = 0; i < 144000; i++);

    uart_printf("[SHT30] Initialized (I2C addr: 0x%02X)\r\n", SHT30_I2C_ADDR);
    return 0;
}

/* ========== 单次测量 ========== */
int sht30_measure(I2C_TypeDef *i2c, float *temperature, float *humidity)
{
    uint8_t data[6];
    uint16_t raw_temp, raw_humi;
    int timeout;

    /* 发送高重复性测量命令 */
    if (sht30_send_command(i2c, SHT30_CMD_SINGLE_HIGH) < 0) {
        return -1;
    }

    /* 等待测量完成（高重复性模式：最大 15ms） */
    for (volatile int i = 0; i < 15000; i++) {
        __NOP();
    }

    /* 读取 6 字节：T_MSB, T_LSB, T_CRC, H_MSB, H_LSB, H_CRC */
    timeout = 100000;
    I2C_GenerateSTART(i2c, ENABLE);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT)) {
        if (--timeout < 0) return -2;
    }

    I2C_Send7bitAddress(i2c, SHT30_I2C_ADDR << 1, I2C_Direction_Receiver);
    timeout = 100000;
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        if (--timeout < 0) return -2;
    }

    for (int i = 0; i < 6; i++) {
        if (i == 5) {
            I2C_AcknowledgeConfig(i2c, DISABLE); /* 最后一字节 NACK */
        }
        timeout = 100000;
        while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
            if (--timeout < 0) {
                I2C_AcknowledgeConfig(i2c, ENABLE);
                I2C_GenerateSTOP(i2c, ENABLE);
                return -2;
            }
        }
        data[i] = I2C_ReceiveData(i2c);
    }

    I2C_AcknowledgeConfig(i2c, ENABLE);
    I2C_GenerateSTOP(i2c, ENABLE);

    /* CRC 校验 */
    if (sht30_crc8(data, 2) != data[2]) {
        uart_printf("[SHT30] Temperature CRC error\r\n");
        return -3;
    }
    if (sht30_crc8(data + 3, 2) != data[5]) {
        uart_printf("[SHT30] Humidity CRC error\r\n");
        return -3;
    }

    /* 数据转换 */
    raw_temp = ((uint16_t)data[0] << 8) | data[1];
    raw_humi = ((uint16_t)data[3] << 8) | data[4];

    *temperature = -45.0f + 175.0f * (float)raw_temp / 65535.0f;
    *humidity    = 100.0f * (float)raw_humi / 65535.0f;

    return 0;
}

/* ========== 读取状态寄存器 ========== */
int sht30_read_status(I2C_TypeDef *i2c, uint16_t *status)
{
    uint8_t data[3];

    sht30_send_command(i2c, SHT30_CMD_READ_STATUS);
    for (volatile int i = 0; i < 10000; i++);

    /* 简化读取流程 */
    int timeout = 100000;
    I2C_GenerateSTART(i2c, ENABLE);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT)) {
        if (--timeout < 0) return -1;
    }

    I2C_Send7bitAddress(i2c, SHT30_I2C_ADDR << 1, I2C_Direction_Receiver);
    timeout = 100000;
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        if (--timeout < 0) return -1;
    }

    for (int i = 0; i < 3; i++) {
        if (i == 2) I2C_AcknowledgeConfig(i2c, DISABLE);
        timeout = 100000;
        while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
            if (--timeout < 0) return -1;
        }
        data[i] = I2C_ReceiveData(i2c);
    }

    I2C_AcknowledgeConfig(i2c, ENABLE);
    I2C_GenerateSTOP(i2c, ENABLE);

    if (sht30_crc8(data, 2) == data[2]) {
        *status = ((uint16_t)data[0] << 8) | data[1];
        return 0;
    }
    return -3;
}

/* ========== 软件复位 ========== */
void sht30_reset(I2C_TypeDef *i2c)
{
    sht30_send_command(i2c, SHT30_CMD_SOFT_RESET);
    for (volatile int i = 0; i < 20000; i++); /* 等待 2ms */
}
