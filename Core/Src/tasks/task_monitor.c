/**
 * @file    task_monitor.c
 * @brief   任务监控实现
 *
 * 功能：
 *   - 独立看门狗喂狗 (IWDG)
 *   - 各任务健康检查（水位线监测）
 *   - 子进程（任务）异常检测与告警
 */

#include "task_monitor.h"
#include "uart_dma.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* 外部任务句柄（来自各任务模块） */
extern TaskHandle_t g_temp_humi_task_handle;
extern TaskHandle_t g_pwm_ctrl_task_handle;
extern TaskHandle_t g_comm_task_handle;

/* ========== 检查单个任务的栈水位 ========== */
static void check_task_stack(TaskHandle_t task, const char *name, uint16_t min_free)
{
    if (task == NULL) {
        uart_printf("[Monitor] %s: NULL handle!\r\n", name);
        return;
    }

    UBaseType_t high_water = uxTaskGetStackHighWaterMark(task);
    if (high_water < min_free) {
        uart_printf("[Monitor] %s: LOW STACK! Free=%u words (min=%u)\r\n",
                    name, (unsigned)high_water, min_free);
    }
}

/* ========== 检查任务状态 ========== */
static void check_task_state(TaskHandle_t task, const char *name)
{
    if (task == NULL) return;

    eTaskState state = eTaskGetState(task);
    const char *state_str;

    switch (state) {
    case eRunning:   state_str = "RUNNING"; break;
    case eReady:     state_str = "READY"; break;
    case eBlocked:   state_str = "BLOCKED"; break;
    case eSuspended: state_str = "SUSPENDED"; break;
    case eDeleted:   state_str = "DELETED"; break;
    default:         state_str = "INVALID"; break;
    }

    if (state == eSuspended || state == eDeleted || state == eInvalid) {
        uart_printf("[Monitor] %s: ABNORMAL state=%s!\r\n", name, state_str);
    }
}

/* ========== 任务创建 ========== */
void monitor_task_create(void)
{
    xTaskCreate(
        monitor_task_func,
        "Monitor",
        STACK_SIZE_MONITOR,
        NULL,
        PRIO_MONITOR,
        &g_monitor_task_handle
    );
}

/* ========== 任务主函数 ========== */
void monitor_task_func(void *pvParameters)
{
    (void)pvParameters;
    uint32_t last_wake_time;
    uint32_t tick_count = 0;

    vTaskDelay(pdMS_TO_TICKS(2000));
    last_wake_time = xTaskGetTickCount();

    uart_printf("[Monitor] Task started, period=5s\r\n");

    while (1) {
        tick_count++;

        /* --- 喂狗 --- */
        IWDG_ReloadCounter();

        /* --- 每 10 个周期 (~50s) 做一次完整检查 --- */
        if (tick_count % 10 == 0) {
            uart_printf("[Monitor] === Health Check #%lu ===\r\n", tick_count / 10);

            /* 栈水位检查（预留 32 words 安全余量） */
            check_task_stack(g_temp_humi_task_handle, "TempHumi", 32);
            check_task_stack(g_pwm_ctrl_task_handle,  "PWMCtrl",  32);
            check_task_stack(g_comm_task_handle,       "Comm",     64);

            /* 任务状态检查 */
            check_task_state(g_temp_humi_task_handle, "TempHumi");
            check_task_state(g_pwm_ctrl_task_handle,  "PWM Ctrl");
            check_task_state(g_comm_task_handle,       "Comm");
            check_task_state(g_monitor_task_handle,    "Monitor");

            /* 堆内存统计 */
            size_t free_heap = xPortGetFreeHeapSize();
            size_t min_ever = xPortGetMinimumEverFreeHeapSize();
            uart_printf("[Monitor] Heap: free=%u, min_ever=%u\r\n",
                        (unsigned)free_heap, (unsigned)min_ever);

            /* 任务数量 */
            unsigned task_count = uxTaskGetNumberOfTasks();
            uart_printf("[Monitor] Tasks: %u running\r\n", task_count);

            /* 系统运行时间 */
            uint32_t uptime = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
            uart_printf("[Monitor] Uptime: %lu s\r\n", uptime);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5000));
    }
}
