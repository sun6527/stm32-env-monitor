/**
 * @file    task_monitor.h
 * @brief   任务监控 - 看门狗喂狗 + 任务健康检查
 */

#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* 监控任务句柄 */
extern TaskHandle_t g_monitor_task_handle;

/* 创建监控任务 */
void monitor_task_create(void);

/* 任务主函数 */
void monitor_task_func(void *pvParameters);

#endif /* TASK_MONITOR_H */
