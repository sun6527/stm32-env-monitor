/**
 * @file    uart_dma.h
 * @brief   UART + DMA 通信驱动
 *
 * USART1 (PA9/PA10): 调试串口，波特率 115200, 8N1
 * 使用 DMA1 进行数据传输，降低 CPU 占用
 */

#ifndef UART_DMA_H
#define UART_DMA_H

#include "main.h"

/* 缓冲区大小 */
#define UART_RX_BUF_SIZE    256
#define UART_TX_BUF_SIZE    256

/* UART 初始化 (USART1, 115200-8-N-1, DMA TX+RX) */
void uart_dma_init(void);

/* DMA 发送数据 */
int  uart_dma_send(const uint8_t *data, uint16_t len);

/* 获取接收缓冲区中可用数据量 */
uint16_t uart_dma_rx_available(void);

/* 从接收缓冲区读取数据 */
uint16_t uart_dma_rx_read(uint8_t *buf, uint16_t max_len);

/* 格式化发送 (printf 风格, 最大 256 字节) */
void uart_printf(const char *fmt, ...);

/* 发送传感器数据 JSON 格式 */
void uart_send_sensor_json(float temp, float humi, uint8_t fan, uint8_t light);

/* DMA 传输完成回调 */
void uart_dma_tx_complete_callback(void);
void uart_dma_rx_complete_callback(uint16_t len);

#endif /* UART_DMA_H */
