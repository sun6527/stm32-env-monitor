/**
 * @file    main.h
 * @brief   主头文件 - 系统级定义
 */

#ifndef MAIN_H
#define MAIN_H

#include "stm32f10x.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ========== 系统时钟 ========== */
#define SYSTEM_CLOCK_HZ         72000000UL
#define APB1_CLOCK_HZ           36000000UL
#define APB2_CLOCK_HZ           72000000UL

/* ========== 任务栈大小定义 ========== */
#define STACK_SIZE_TEMP_HUMI    256     /* 温湿度采集任务 */
#define STACK_SIZE_PWM_CTRL     128     /* PWM 控制任务 */
#define STACK_SIZE_COMM         256     /* 通信任务 */
#define STACK_SIZE_MONITOR      128     /* 监控任务 */

/* ========== 任务优先级 ========== */
#define PRIO_TEMP_HUMI          3       /* 最高 - 传感器采集 */
#define PRIO_PWM_CTRL           2       /* 中 - 控制响应 */
#define PRIO_COMM               2       /* 中 - 通信 */
#define PRIO_MONITOR            1       /* 低 - 系统监控 */

/* ========== 队列与信号量 ========== */
#define QUEUE_SENSOR_DATA_LEN   8       /* 传感器数据队列长度 */
#define QUEUE_CTRL_CMD_LEN      4       /* 控制指令队列长度 */
#define QUEUE_LOG_LEN           16      /* 日志队列长度 */

/* ========== 环境参数阈值 ========== */
#define TEMP_HIGH_THRESHOLD     30.0f   /* 温度过高阈值 (°C) */
#define TEMP_LOW_THRESHOLD      15.0f   /* 温度过低阈值 (°C) */
#define HUMI_HIGH_THRESHOLD     85.0f   /* 湿度过高阈值 (%) */
#define HUMI_LOW_THRESHOLD      40.0f   /* 湿度过低阈值 (%) */
#define TEMP_TARGET             25.0f   /* 目标温度 */
#define HUMI_TARGET             60.0f   /* 目标湿度 */

/* ========== PWM 参数 ========== */
#define PWM_PERIOD              999     /* PWM 周期 (ARR) */
#define PWM_FAN_MIN_DUTY        20      /* 风扇最小占空比 */
#define PWM_FAN_MAX_DUTY        100     /* 风扇最大占空比 */
#define PWM_LIGHT_MIN_DUTY      10      /* 补光灯最小占空比 */
#define PWM_LIGHT_MAX_DUTY      100     /* 补光灯最大占空比 */

/* ========== 看门狗 ========== */
#define WATCHDOG_TIMEOUT_MS     5000    /* 看门狗超时 (ms) */

/* ========== 传感器数据结构 ========== */
typedef struct {
    float       temperature;    /* 温度 (°C) */
    float       humidity;       /* 湿度 (%) */
    uint32_t    timestamp_ms;   /* 采集时间戳 */
    uint8_t     valid;          /* 数据有效性 */
} sensor_data_t;

/* ========== 控制指令结构 ========== */
typedef enum {
    CTRL_NONE = 0,
    CTRL_FAN_ON,
    CTRL_FAN_OFF,
    CTRL_FAN_SET_DUTY,
    CTRL_LIGHT_ON,
    CTRL_LIGHT_OFF,
    CTRL_LIGHT_SET_DUTY,
    CTRL_AUTO_MODE,
    CTRL_MANUAL_MODE,
} control_cmd_type_t;

typedef struct {
    control_cmd_type_t  type;
    uint8_t             value;      /* 占空比值 (0-100) */
} control_cmd_t;

/* ========== 系统状态 ========== */
typedef enum {
    SYS_MODE_AUTO = 0,          /* 自动控制模式 */
    SYS_MODE_MANUAL,            /* 手动控制模式 */
    SYS_MODE_ALARM,             /* 告警模式 */
} system_mode_t;

typedef struct {
    system_mode_t   mode;
    uint8_t         fan_duty;       /* 当前风扇占空比 */
    uint8_t         light_duty;     /* 当前补光灯占空比 */
    uint32_t        uptime_ms;      /* 系统运行时间 */
} system_status_t;

#endif /* MAIN_H */
