/**
 * @file    uart_dma.c
 * @brief   UART + DMA 驱动实现
 *
 * USART1: 调试串口 (PA9 TX / PA10 RX), 115200-8-N-1
 * DMA1_CH4: USART1_TX
 * DMA1_CH5: USART1_RX
 *
 * 使用 DMA 环形接收缓冲区，降低 CPU 占用率。
 */

#include "uart_dma.h"
#include <stdarg.h>
#include <stdio.h>

/* DMA 缓冲区 */
static uint8_t g_uart_tx_buf[UART_TX_BUF_SIZE];
static uint8_t g_uart_rx_buf[UART_RX_BUF_SIZE];

/* 环形接收缓冲区读写索引 */
static volatile uint16_t g_rx_read_idx = 0;

/* DMA 发送状态 */
static volatile uint8_t g_tx_busy = 0;

/* ========== USART1 + DMA 初始化 ========== */
void uart_dma_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    DMA_InitTypeDef dma;

    /* ---------- GPIO ---------- */
    /* PA9  - USART1_TX (复用推挽) */
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA10 - USART1_RX (浮空输入) */
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* ---------- USART1 ---------- */
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    /* 使能 DMA 发送/接收 */
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    USART_Cmd(USART1, ENABLE);

    /* ---------- DMA1_CH4: USART1_TX ---------- */
    DMA_DeInit(DMA1_Channel4);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)g_uart_tx_buf;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;        /* 内存→外设 */
    dma.DMA_BufferSize = 0;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;             /* 单次传输 */
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &dma);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    /* ---------- DMA1_CH5: USART1_RX (环形模式) ---------- */
    DMA_DeInit(DMA1_Channel5);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)g_uart_rx_buf;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;        /* 外设→内存 */
    dma.DMA_BufferSize = UART_RX_BUF_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;           /* 环形模式 */
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &dma);

    /* 启动 DMA 接收 */
    DMA_Cmd(DMA1_Channel5, ENABLE);
}

/* ========== DMA 发送 ========== */
int uart_dma_send(const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > UART_TX_BUF_SIZE) return -1;

    /* 等待上次发送完成 */
    uint32_t timeout = 100000;
    while (g_tx_busy) {
        if (--timeout == 0) return -2;
    }

    g_tx_busy = 1;

    /* 复制数据到发送缓冲区 */
    memcpy(g_uart_tx_buf, data, len);

    /* 配置 DMA */
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CNDTR = len;
    DMA1_Channel4->CMAR = (uint32_t)g_uart_tx_buf;
    DMA_Cmd(DMA1_Channel4, ENABLE);

    return len;
}

/* ========== 接收缓冲区查询 ========== */
uint16_t uart_dma_rx_available(void)
{
    uint16_t write_idx = UART_RX_BUF_SIZE - DMA1_Channel5->CNDTR;
    if (write_idx >= g_rx_read_idx) {
        return write_idx - g_rx_read_idx;
    } else {
        return UART_RX_BUF_SIZE - g_rx_read_idx + write_idx;
    }
}

uint16_t uart_dma_rx_read(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    while (count < max_len && uart_dma_rx_available() > 0) {
        buf[count++] = g_uart_rx_buf[g_rx_read_idx];
        g_rx_read_idx = (g_rx_read_idx + 1) % UART_RX_BUF_SIZE;
    }
    return count;
}

/* ========== 格式化发送 ========== */
void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        uart_dma_send((uint8_t*)buf, (uint16_t)len);
    }
}

/* ========== 发送传感器 JSON ========== */
void uart_send_sensor_json(float temp, float humi, uint8_t fan, uint8_t light)
{
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "{\"type\":\"status\",\"temp\":%.1f,\"humi\":%.1f,\"fan\":%d,\"light\":%d}\r\n",
        temp, humi, fan, light);
    if (len > 0) {
        uart_dma_send((uint8_t*)buf, (uint16_t)len);
    }
}

/* ========== DMA 完成回调 ========== */
void uart_dma_tx_complete_callback(void)
{
    g_tx_busy = 0;
}

void uart_dma_rx_complete_callback(uint16_t len)
{
    /* 环形缓冲区自动覆盖，无需特殊处理 */
    (void)len;
}

/* ========== DMA1 中断处理 ========== */
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC4)) {
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        DMA_Cmd(DMA1_Channel4, DISABLE);
        uart_dma_tx_complete_callback();
    }
}

void DMA1_Channel5_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC5)) {
        DMA_ClearITPendingBit(DMA1_IT_TC5);
        uart_dma_rx_complete_callback(UART_RX_BUF_SIZE);
    }
    if (DMA_GetITStatus(DMA1_IT_HT5)) {
        DMA_ClearITPendingBit(DMA1_IT_HT5);
        uart_dma_rx_complete_callback(UART_RX_BUF_SIZE / 2);
    }
}
