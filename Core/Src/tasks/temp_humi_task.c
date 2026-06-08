/**
 * @file    temp_humi_task.c
 * @brief   温湿度采集任务实现
 *
 * 周期: 500ms
 * 功能:
 *   - I2C 读取 SHT30 传感器数据
 *   - 数据合法性校验
 *   - 阈值越限检测与告警
 *   - 通过队列发送给控制任务
 */

#include "temp_humi_task.h"
#include "sht30.h"
#include "uart_dma.h"
#include "control_algorithm.h"
#include "task_monitor.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* 连续错误计数（用于故障检测） */
#define MAX_CONSECUTIVE_ERRORS  5

/* ========== 数据校验 ========== */
static int sensor_data_is_valid(float temp, float humi)
{
    /* SHT30 有效范围 */
    if (temp < -40.0f || temp > 125.0f) return 0;
    if (humi < 0.0f || humi > 100.0f) return 0;

    /* 温室场景合理范围（超出视为异常） */
    if (temp < -10.0f || temp > 60.0f) return 0;
    if (humi < 5.0f || humi > 99.0f) return 0;

    return 1;
}

/* ========== 阈值越限检测 ========== */
static void check_threshold_alarm(float temp, float humi)
{
    char log_buf[128];

    if (temp > TEMP_HIGH_THRESHOLD) {
        snprintf(log_buf, sizeof(log_buf),
                 "[ALARM] Temp HIGH: %.1f°C > %.1f°C", temp, TEMP_HIGH_THRESHOLD);
        xQueueSend(g_log_queue, log_buf, 0);
    } else if (temp < TEMP_LOW_THRESHOLD) {
        snprintf(log_buf, sizeof(log_buf),
                 "[ALARM] Temp LOW: %.1f°C < %.1f°C", temp, TEMP_LOW_THRESHOLD);
        xQueueSend(g_log_queue, log_buf, 0);
    }

    if (humi > HUMI_HIGH_THRESHOLD) {
        snprintf(log_buf, sizeof(log_buf),
                 "[ALARM] Humidity HIGH: %.1f%% > %.1f%%", humi, HUMI_HIGH_THRESHOLD);
        xQueueSend(g_log_queue, log_buf, 0);
    } else if (humi < HUMI_LOW_THRESHOLD) {
        snprintf(log_buf, sizeof(log_buf),
                 "[ALARM] Humidity LOW: %.1f%% < %.1f%%", humi, HUMI_LOW_THRESHOLD);
        xQueueSend(g_log_queue, log_buf, 0);
    }
}

/* ========== 任务创建 ========== */
void temp_humi_task_create(void)
{
    xTaskCreate(
        temp_humi_task_func,
        "TempHumi",
        STACK_SIZE_TEMP_HUMI,
        NULL,
        PRIO_TEMP_HUMI,
        &g_temp_humi_task_handle
    );
}

/* ========== 任务主函数 ========== */
void temp_humi_task_func(void *pvParameters)
{
    (void)pvParameters;
    sensor_data_t sensor_data;
    float temp, humi;
    uint32_t last_wake_time;
    int consecutive_errors = 0;

    /* 初始延时，等待外设稳定 */
    vTaskDelay(pdMS_TO_TICKS(1000));

    last_wake_time = xTaskGetTickCount();

    uart_printf("[TempHumi] Task started, period=500ms\r\n");

    while (1) {
        /* 读取 SHT30 */
        int ret = sht30_measure(I2C1, &temp, &humi);

        if (ret == 0 && sensor_data_is_valid(temp, humi)) {
            consecutive_errors = 0;

            /* 填充数据 */
            sensor_data.temperature = temp;
            sensor_data.humidity = humi;
            sensor_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            sensor_data.valid = 1;

            /* 发送到控制任务 */
            if (xQueueSend(g_sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) != pdPASS) {
                uart_printf("[TempHumi] Queue full, data dropped\r\n");
            }

            /* 阈值越限检测 */
            check_threshold_alarm(temp, humi);

            uart_printf("[TempHumi] T=%.1f°C H=%.1f%%\r\n", temp, humi);

        } else {
            consecutive_errors++;
            uart_printf("[TempHumi] Read error (ret=%d, cnt=%d)\r\n", ret, consecutive_errors);

            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                uart_printf("[TempHumi] SHT30 fatal error, sensor offline!\r\n");

                /* 发送无效数据通知控制任务 */
                sensor_data.valid = 0;
                sensor_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                xQueueSend(g_sensor_data_queue, &sensor_data, 0);
            }
        }

        /* 精确周期延时 */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(500));
    }
}
