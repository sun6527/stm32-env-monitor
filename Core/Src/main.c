/**
 * @file    main.c
 * @brief   嵌入式环境监测与控制系统 - 主入口
 *
 * 基于 STM32F103RBT6 + FreeRTOS 的温室环境监测终端
 *
 * 功能：
 *   - SHT30 温湿度采集 (I2C)
 *   - PWM 补光灯/风扇控制
 *   - UART + DMA 数据上报
 *   - 闭环阈值联动自动控制
 *   - 看门狗 + 任务监控
 *
 * 硬件平台: STM32F103RBT6, 72MHz
 * RTOS:     FreeRTOS v10.4.3
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "sht30.h"
#include "gpio_pwm.h"
#include "uart_dma.h"

#include "temp_humi_task.h"
#include "pwm_control_task.h"
#include "comm_task.h"
#include "task_monitor.h"

/* ========== 全局队列定义 ========== */
QueueHandle_t g_sensor_data_queue = NULL;
QueueHandle_t g_ctrl_cmd_queue = NULL;
QueueHandle_t g_log_queue = NULL;

/* 全局任务句柄 */
TaskHandle_t g_temp_humi_task_handle = NULL;
TaskHandle_t g_pwm_ctrl_task_handle = NULL;
TaskHandle_t g_comm_task_handle = NULL;
TaskHandle_t g_monitor_task_handle = NULL;

/* ========== 系统时钟配置 ========== */
static void system_clock_config(void)
{
    ErrorStatus HSEStartUpStatus;

    /* 复位 RCC 配置 */
    RCC_DeInit();

    /* 使能 HSE (8MHz 外部晶振) */
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS) {
        /* 闪存预取缓冲 + 2 等待周期 (72MHz) */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        FLASH_SetLatency(FLASH_Latency_2);

        /* AHB = 72MHz, APB1 = 36MHz, APB2 = 72MHz */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);

        /* PLL: 8MHz * 9 = 72MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (RCC_GetSYSCLKSource() != 0x08);
    } else {
        /* HSE 失败，使用 HSI */
        while (1);
    }
}

/* ========== 外设时钟使能 ========== */
static void peripheral_clock_init(void)
{
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC | RCC_APB2Periph_USART1 |
        RCC_APB2Periph_AFIO,
        ENABLE);
    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART2 |
        RCC_APB1Periph_I2C1,
        ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

/* ========== NVIC 中断配置 ========== */
static void nvic_config(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* USART1 DMA 中断 */
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

/* ========== 独立看门狗初始化 ========== */
static void iwdg_init(void)
{
    /* 使能写访问 */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    /* LSI = 40kHz, 预分频 64 → 625Hz, 重装载 3125 → 5s 超时 */
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(3125);
    /* 重装载并启动 */
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/* ========== 主函数 ========== */
int main(void)
{
    /* --- 硬件初始化 --- */
    system_clock_config();
    peripheral_clock_init();
    nvic_config();

    /* 外设初始化 */
    uart_dma_init();
    gpio_light_ctrl_init();
    gpio_key_init();
    pwm_init();
    sht30_init(I2C1);
    iwdg_init();

    uart_printf("\r\n========================================\r\n");
    uart_printf("  STM32 Environmental Monitor v1.0\r\n");
    uart_printf("  MCU: STM32F103RBT6 @ 72MHz\r\n");
    uart_printf("  RTOS: FreeRTOS\r\n");
    uart_printf("========================================\r\n\r\n");

    /* --- 创建内核对象 --- */
    g_sensor_data_queue = xQueueCreate(QUEUE_SENSOR_DATA_LEN, sizeof(sensor_data_t));
    g_ctrl_cmd_queue     = xQueueCreate(QUEUE_CTRL_CMD_LEN, sizeof(control_cmd_t));
    g_log_queue          = xQueueCreate(QUEUE_LOG_LEN, 128); /* 每条日志最大 128 字节 */

    if (!g_sensor_data_queue || !g_ctrl_cmd_queue || !g_log_queue) {
        uart_printf("[FATAL] Queue creation failed!\r\n");
        while (1);
    }

    /* --- 创建任务 --- */
    temp_humi_task_create();
    pwm_ctrl_task_create();
    comm_task_create();
    monitor_task_create();

    uart_printf("[INIT] All tasks created, starting scheduler...\r\n");

    /* --- 启动调度器 --- */
    vTaskStartScheduler();

    /* 正常情况不会到这里 */
    uart_printf("[FATAL] Scheduler failed to start!\r\n");
    while (1);
}
