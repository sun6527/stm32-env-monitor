/**
 * @file    temp_humi_task.h
 * @brief   温湿度采集任务
 */

#ifndef TEMP_HUMI_TASK_H
#define TEMP_HUMI_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* 温湿度采集任务句柄 */
extern TaskHandle_t g_temp_humi_task_handle;

/* 传感器数据队列句柄 */
extern QueueHandle_t g_sensor_data_queue;

/* 创建温湿度采集任务 */
void temp_humi_task_create(void);

/* 任务主函数 */
void temp_humi_task_func(void *pvParameters);

#endif /* TEMP_HUMI_TASK_H */
