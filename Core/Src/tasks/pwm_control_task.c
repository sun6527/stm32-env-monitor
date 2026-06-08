/**
 * @file    pwm_control_task.c
 * @brief   PWM 控制任务实现
 *
 * 周期: 200ms
 * 功能:
 *   - 接收传感器数据
 *   - 执行闭环控制算法
 *   - 控制补光灯 & 通风扇 PWM
 *   - 支持手动/自动模式切换
 */

#include "pwm_control_task.h"
#include "control_algorithm.h"
#include "gpio_pwm.h"
#include "uart_dma.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ========== 控制模式 ========== */
static system_mode_t g_control_mode = SYS_MODE_AUTO;

/* ========== 任务创建 ========== */
void pwm_ctrl_task_create(void)
{
    xTaskCreate(
        pwm_ctrl_task_func,
        "PWMControl",
        STACK_SIZE_PWM_CTRL,
        NULL,
        PRIO_PWM_CTRL,
        &g_pwm_ctrl_task_handle
    );
}

/* ========== 执行控制输出 ========== */
static void apply_control(uint8_t fan_duty, uint8_t light_duty)
{
    uint8_t prev_fan = pwm_fan_get_duty();
    uint8_t prev_light = pwm_light_get_duty();

    if (fan_duty != prev_fan) {
        pwm_fan_set_duty(fan_duty);
    }
    if (light_duty != prev_light) {
        pwm_light_set_duty(light_duty);
    }

    /* 仅变化时打印日志 */
    if (fan_duty != prev_fan || light_duty != prev_light) {
        uart_printf("[Control] Fan: %d%% -> %d%%, Light: %d%% -> %d%%\r\n",
                    prev_fan, fan_duty, prev_light, light_duty);
    }
}

/* ========== 任务主函数 ========== */
void pwm_ctrl_task_func(void *pvParameters)
{
    (void)pvParameters;
    sensor_data_t sensor_data;
    control_cmd_t cmd;
    uint32_t last_wake_time;
    uint8_t fan_duty = 0, light_duty = 0;
    int sensor_timeout = 0;

    vTaskDelay(pdMS_TO_TICKS(500));
    last_wake_time = xTaskGetTickCount();

    uart_printf("[Control] Task started, period=200ms, mode=AUTO\r\n");

    while (1) {
        /* 检查手动控制指令 */
        while (xQueueReceive(g_ctrl_cmd_queue, &cmd, 0) == pdPASS) {
            switch (cmd.type) {
            case CTRL_AUTO_MODE:
                g_control_mode = SYS_MODE_AUTO;
                control_reset_integrator();
                uart_printf("[Control] Switched to AUTO mode\r\n");
                break;
            case CTRL_MANUAL_MODE:
                g_control_mode = SYS_MODE_MANUAL;
                uart_printf("[Control] Switched to MANUAL mode\r\n");
                break;
            case CTRL_FAN_SET_DUTY:
                if (g_control_mode == SYS_MODE_MANUAL) {
                    fan_duty = cmd.value;
                    apply_control(fan_duty, light_duty);
                }
                break;
            case CTRL_LIGHT_SET_DUTY:
                if (g_control_mode == SYS_MODE_MANUAL) {
                    light_duty = cmd.value;
                    apply_control(fan_duty, light_duty);
                }
                break;
            case CTRL_FAN_ON:
                if (g_control_mode == SYS_MODE_MANUAL) {
                    fan_duty = 100;
                    apply_control(fan_duty, light_duty);
                }
                break;
            case CTRL_FAN_OFF:
                if (g_control_mode == SYS_MODE_MANUAL) {
                    fan_duty = 0;
                    apply_control(fan_duty, light_duty);
                }
                break;
            default:
                break;
            }
        }

        /* 自动模式：接收传感器数据并计算控制输出 */
        if (g_control_mode == SYS_MODE_AUTO) {
            if (xQueueReceive(g_sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) == pdPASS) {
                sensor_timeout = 0;

                if (sensor_data.valid) {
                    int alarm = control_calculate(
                        sensor_data.temperature,
                        sensor_data.humidity,
                        &fan_duty,
                        &light_duty
                    );

                    apply_control(fan_duty, light_duty);

                    if (alarm) {
                        uart_printf("[Control] ALARM state - extreme environment!\r\n");
                    }
                } else {
                    /* 传感器无效：保持当前状态，3 次后切告警模式 */
                    sensor_timeout++;
                    if (sensor_timeout >= 3) {
                        g_control_mode = SYS_MODE_ALARM;
                        uart_printf("[Control] Sensor timeout, switch to ALARM mode\r\n");
                        /* 告警模式：风扇全开 */
                        apply_control(100, 0);
                    }
                }
            }
        }

        /* 检查按键切换模式（手动按键 PA4） */
        if (gpio_key_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(50)); /* 消抖 */
            if (gpio_key_is_pressed()) {
                if (g_control_mode == SYS_MODE_AUTO) {
                    g_control_mode = SYS_MODE_MANUAL;
                    uart_printf("[Control] Button: MANUAL mode\r\n");
                } else {
                    g_control_mode = SYS_MODE_AUTO;
                    control_reset_integrator();
                    uart_printf("[Control] Button: AUTO mode\r\n");
                }
                /* 等待按键释放 */
                while (gpio_key_is_pressed()) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(200));
    }
}
