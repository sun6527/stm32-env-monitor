/**
 * @file    gpio_pwm.h
 * @brief   GPIO 与 PWM 驱动
 *
 * GPIO:  PC13 补光灯开关（继电器）
 *        PA4  手动模式按键
 * PWM:   TIM2_CH1 (PA0) 补光灯亮度
 *        TIM2_CH2 (PA1) 通风扇转速
 */

#ifndef GPIO_PWM_H
#define GPIO_PWM_H

#include "main.h"

/* ========== GPIO 初始化 ========== */

/* 补光灯继电器 (PC13) */
void gpio_light_ctrl_init(void);
void gpio_light_on(void);
void gpio_light_off(void);

/* 手动模式按键 (PA4, 上拉输入) */
void gpio_key_init(void);
uint8_t gpio_key_is_pressed(void);

/* ========== PWM 初始化 ========== */

/* TIM2 PWM: PA0 (CH1-补光灯), PA1 (CH2-风扇) */
void pwm_init(void);

/* 设置补光灯占空比 (0-100%) */
void pwm_light_set_duty(uint8_t duty);

/* 设置风扇占空比 (0-100%) */
void pwm_fan_set_duty(uint8_t duty);

/* 获取当前占空比 */
uint8_t pwm_light_get_duty(void);
uint8_t pwm_fan_get_duty(void);

/* PWM 停止/恢复 */
void pwm_light_stop(void);
void pwm_fan_stop(void);

#endif /* GPIO_PWM_H */
