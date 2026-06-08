/**
 * @file    pwm_control_task.h
 * @brief   PWM 控制任务 - 闭环自动化控制
 */

#ifndef PWM_CONTROL_TASK_H
#define PWM_CONTROL_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* PWM 控制任务句柄 */
extern TaskHandle_t g_pwm_ctrl_task_handle;

/* 控制指令队列句柄 */
extern QueueHandle_t g_ctrl_cmd_queue;

/* 创建 PWM 控制任务 */
void pwm_ctrl_task_create(void);

/* 任务主函数 */
void pwm_ctrl_task_func(void *pvParameters);

#endif /* PWM_CONTROL_TASK_H */
