/**
 * @file    comm_task.h
 * @brief   通信任务 - UART 数据上报与指令接收
 */

#ifndef COMM_TASK_H
#define COMM_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* 通信任务句柄 */
extern TaskHandle_t g_comm_task_handle;

/* 日志队列句柄 */
extern QueueHandle_t g_log_queue;

/* 创建通信任务 */
void comm_task_create(void);

/* 任务主函数 */
void comm_task_func(void *pvParameters);

#endif /* COMM_TASK_H */
