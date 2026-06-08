/**
 * @file    control_algorithm.c
 * @brief   闭环阈值联动控制算法实现
 *
 * 采用简化的 PI 控制 + 死区策略：
 *   - 误差在死区内 → 不调整（避免频繁抖动）
 *   - 温度控制风扇转速
 *   - 湿度控制补光灯亮度
 *   - 极端情况全速响应
 */

#include "control_algorithm.h"
#include <string.h>

/* 默认控制参数 */
static control_params_t g_params = {
    .temp_kp        = 5.0f,     /* 温度比例系数 */
    .temp_ki        = 0.5f,     /* 温度积分系数 */
    .temp_kd        = 0.0f,     /* 不使用微分 */
    .humi_kp        = 3.0f,
    .humi_ki        = 0.3f,
    .humi_kd        = 0.0f,
    .deadband_temp  = 0.5f,     /* ±0.5°C 死区 */
    .deadband_humi  = 2.0f,     /* ±2%RH 死区 */
};

/* 积分项（需在模式切换时重置） */
static float g_temp_integral = 0.0f;
static float g_humi_integral = 0.0f;

/* ========== 限幅函数 ========== */
static float clamp(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/* ========== 控制计算 ========== */
int control_calculate(float current_temp, float current_humi,
                      uint8_t *fan_duty, uint8_t *light_duty)
{
    int alarm = 0;
    float output;

    /* ---- 温度控制（风扇） ---- */
    float temp_error = current_temp - TEMP_TARGET;

    /* 死区检测 */
    if (fabsf(temp_error) < g_params.deadband_temp) {
        temp_error = 0.0f;
        g_temp_integral *= 0.5f;  /* 逐渐衰减积分 */
    }

    if (temp_error > 0) {
        /* 温度偏高 → 增加风扇转速 */
        g_temp_integral += temp_error * 0.2f;  /* 200ms 采样周期 */
        g_temp_integral = clamp(g_temp_integral, 0.0f, 50.0f);

        output = g_params.temp_kp * temp_error + g_params.temp_ki * g_temp_integral;
        *fan_duty = (uint8_t)clamp(output, PWM_FAN_MIN_DUTY, PWM_FAN_MAX_DUTY);

        if (current_temp > TEMP_HIGH_THRESHOLD) alarm = 1;

    } else if (temp_error < -0.5f) {
        /* 温度偏低 → 降低风扇转速（节能） */
        g_temp_integral += temp_error * 0.2f;
        g_temp_integral = clamp(g_temp_integral, -50.0f, 0.0f);

        output = g_params.temp_kp * temp_error + g_params.temp_ki * g_temp_integral;
        *fan_duty = (uint8_t)clamp(-output, 0, PWM_FAN_MIN_DUTY); /* 最小风速保持空气流动 */

        if (current_temp < TEMP_LOW_THRESHOLD) alarm = 1;
    } else {
        /* 正常范围：低风速维持 */
        g_temp_integral *= 0.5f;
        *fan_duty = PWM_FAN_MIN_DUTY;
    }

    /* ---- 湿度控制（补光灯） ---- */
    float humi_error = current_humi - HUMI_TARGET;

    if (fabsf(humi_error) < g_params.deadband_humi) {
        humi_error = 0.0f;
        g_humi_integral *= 0.5f;
    }

    if (humi_error < 0) {
        /* 湿度过低 → 开补光灯增加蒸腾 */
        g_humi_integral += humi_error * 0.2f;
        g_humi_integral = clamp(g_humi_integral, -30.0f, 0.0f);

        output = -(g_params.humi_kp * humi_error + g_params.humi_ki * g_humi_integral);
        *light_duty = (uint8_t)clamp(output, PWM_LIGHT_MIN_DUTY, PWM_LIGHT_MAX_DUTY);

        if (current_humi < HUMI_LOW_THRESHOLD) alarm = 1;

    } else if (humi_error > 5.0f) {
        /* 湿度过高 → 关闭补光灯 */
        g_humi_integral += humi_error * 0.2f;
        g_humi_integral = clamp(g_humi_integral, 0.0f, 30.0f);

        output = g_params.humi_kp * humi_error + g_params.humi_ki * g_humi_integral;
        *light_duty = (uint8_t)clamp(PWM_LIGHT_MIN_DUTY - output, 0, PWM_LIGHT_MIN_DUTY);

        if (current_humi > HUMI_HIGH_THRESHOLD) alarm = 1;
    } else {
        /* 正常范围 */
        g_humi_integral *= 0.5f;
        *light_duty = 0;  /* 关闭补光灯节能 */
    }

    return alarm;
}

/* ========== 参数访问 ========== */
void control_get_params(control_params_t *params)
{
    memcpy(params, &g_params, sizeof(control_params_t));
}

void control_set_params(const control_params_t *params)
{
    memcpy(&g_params, params, sizeof(control_params_t));
    control_reset_integrator();
}

void control_reset_integrator(void)
{
    g_temp_integral = 0.0f;
    g_humi_integral = 0.0f;
}
