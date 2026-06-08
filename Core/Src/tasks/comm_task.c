/**
 * @file    comm_task.c
 * @brief   通信任务实现
 *
 * 功能:
 *   - UART+DMA 数据上报（JSON 格式）
 *   - 上位机指令解析
 *   - 调试日志输出
 */

#include "comm_task.h"
#include "uart_dma.h"
#include "gpio_pwm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* 上报周期 */
#define REPORT_INTERVAL_MS  2000  /* 每 2s 上报一次 */

/* 指令缓冲区 */
static uint8_t g_cmd_buf[128];
static uint8_t g_cmd_pos = 0;

/* ========== 解析上位机指令 ========== */
static void parse_command(const char *cmd)
{
    control_cmd_t ctrl_cmd;
    memset(&ctrl_cmd, 0, sizeof(ctrl_cmd));

    if (strncmp(cmd, "AUTO", 4) == 0) {
        ctrl_cmd.type = CTRL_AUTO_MODE;
        xQueueSend(g_ctrl_cmd_queue, &ctrl_cmd, 0);
        uart_printf("[Comm] CMD: AUTO mode\r\n");
    }
    else if (strncmp(cmd, "MANUAL", 6) == 0) {
        ctrl_cmd.type = CTRL_MANUAL_MODE;
        xQueueSend(g_ctrl_cmd_queue, &ctrl_cmd, 0);
        uart_printf("[Comm] CMD: MANUAL mode\r\n");
    }
    else if (strncmp(cmd, "FAN:", 4) == 0) {
        int duty = atoi(cmd + 4);
        if (duty >= 0 && duty <= 100) {
            ctrl_cmd.type = CTRL_FAN_SET_DUTY;
            ctrl_cmd.value = (uint8_t)duty;
            xQueueSend(g_ctrl_cmd_queue, &ctrl_cmd, 0);
            uart_printf("[Comm] CMD: FAN=%d%%\r\n", duty);
        }
    }
    else if (strncmp(cmd, "LIGHT:", 6) == 0) {
        int duty = atoi(cmd + 6);
        if (duty >= 0 && duty <= 100) {
            ctrl_cmd.type = CTRL_LIGHT_SET_DUTY;
            ctrl_cmd.value = (uint8_t)duty;
            xQueueSend(g_ctrl_cmd_queue, &ctrl_cmd, 0);
            uart_printf("[Comm] CMD: LIGHT=%d%%\r\n", duty);
        }
    }
    else if (strncmp(cmd, "STATUS", 6) == 0) {
        uart_send_sensor_json(0, 0, pwm_fan_get_duty(), pwm_light_get_duty());
    }
    else if (strncmp(cmd, "HELP", 4) == 0) {
        uart_printf("[Comm] Commands:\r\n");
        uart_printf("  AUTO / MANUAL - mode switch\r\n");
        uart_printf("  FAN:<0-100>   - set fan duty\r\n");
        uart_printf("  LIGHT:<0-100> - set light duty\r\n");
        uart_printf("  STATUS        - query status\r\n");
    }
    else {
        uart_printf("[Comm] Unknown CMD: %s\r\n", cmd);
    }
}

/* ========== 检查接收指令 ========== */
static void check_rx_commands(void)
{
    uint8_t byte;
    while (uart_dma_rx_available() > 0) {
        uart_dma_rx_read(&byte, 1);

        if (byte == '\r' || byte == '\n') {
            if (g_cmd_pos > 0) {
                g_cmd_buf[g_cmd_pos] = '\0';
                parse_command((const char*)g_cmd_buf);
                g_cmd_pos = 0;
            }
        } else if (g_cmd_pos < sizeof(g_cmd_buf) - 1) {
            g_cmd_buf[g_cmd_pos++] = byte;
        }
    }
}

/* ========== 发送日志 ========== */
static void flush_logs(void)
{
    char log_buf[128];
    while (xQueueReceive(g_log_queue, log_buf, 0) == pdPASS) {
        uart_printf("%s\r\n", log_buf);
    }
}

/* ========== 任务创建 ========== */
void comm_task_create(void)
{
    xTaskCreate(
        comm_task_func,
        "Comm",
        STACK_SIZE_COMM,
        NULL,
        PRIO_COMM,
        &g_comm_task_handle
    );
}

/* ========== 任务主函数 ========== */
void comm_task_func(void *pvParameters)
{
    (void)pvParameters;
    uint32_t last_wake_time;

    vTaskDelay(pdMS_TO_TICKS(500));
    last_wake_time = xTaskGetTickCount();

    uart_printf("[Comm] Task started, report interval=%dms\r\n", REPORT_INTERVAL_MS);
    uart_printf("[Comm] Type HELP for command list\r\n");

    while (1) {
        /* 检查并处理接收指令 */
        check_rx_commands();

        /* 发送日志 */
        flush_logs();

        /* 定期状态上报 */
        static int report_counter = 0;
        if (++report_counter >= (REPORT_INTERVAL_MS / 200)) {
            report_counter = 0;
            /* 通过串口上报当前状态 */
            uart_send_sensor_json(0, 0, pwm_fan_get_duty(), pwm_light_get_duty());
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(200));
    }
}
