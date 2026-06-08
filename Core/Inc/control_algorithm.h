/**
 * @file    control_algorithm.h
 * @brief   闭环阈值联动控制算法
 *
 * 实现无人值守智能调节：
 *   - 温度过高 → 增加风扇转速
 *   - 湿度过低 → 开启补光灯
 *   - 正常范围内 → 节能模式
 */

#ifndef CONTROL_ALGORITHM_H
#define CONTROL_ALGORITHM_H

#include "main.h"

/* 控制参数 */
typedef struct {
    float   temp_kp;            /* 温度比例系数 */
    float   temp_ki;            /* 温度积分系数 */
    float   temp_kd;            /* 温度微分系数 */
    float   humi_kp;            /* 湿度比例系数 */
    float   humi_ki;            /* 湿度积分系数 */
    float   humi_kd;            /* 湿度微分系数 */
    float   deadband_temp;      /* 温度死区 (±) */
    float   deadband_humi;      /* 湿度死区 (±) */
} control_params_t;

/**
 * 计算控制输出
 * @param current_temp  当前温度
 * @param current_humi  当前湿度
 * @param fan_duty      输出 - 风扇占空比
 * @param light_duty    输出 - 补光灯占空比
 * @return 0=正常, 非0=告警
 */
int control_calculate(float current_temp, float current_humi,
                      uint8_t *fan_duty, uint8_t *light_duty);

/* 获取控制参数 */
void control_get_params(control_params_t *params);

/* 设置控制参数 */
void control_set_params(const control_params_t *params);

/* 重置控制器积分项 */
void control_reset_integrator(void);

#endif /* CONTROL_ALGORITHM_H */
