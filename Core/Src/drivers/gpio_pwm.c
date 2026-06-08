/**
 * @file    gpio_pwm.c
 * @brief   GPIO 与 PWM 驱动实现
 *
 * GPIO:
 *   PC13 - 补光灯继电器（开漏输出）
 *   PA4  - 手动模式按键（上拉输入）
 *
 * PWM (TIM2):
 *   PA0 (TIM2_CH1) - 补光灯亮度
 *   PA1 (TIM2_CH2) - 通风扇转速
 *   频率: 72MHz / (71+1) / (999+1) ≈ 1kHz
 */

#include "gpio_pwm.h"
#include "uart_dma.h"

/* 当前占空比 */
static uint8_t g_light_duty = 0;
static uint8_t g_fan_duty = 0;

/* ========== 补光灯继电器 GPIO 初始化 ========== */
void gpio_light_ctrl_init(void)
{
    GPIO_InitTypeDef gpio;

    /* PC13 - 推挽输出 (继电器) */
    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    gpio_light_off(); /* 默认关闭 */
}

void gpio_light_on(void)  { GPIO_SetBits(GPIOC, GPIO_Pin_13); }
void gpio_light_off(void) { GPIO_ResetBits(GPIOC, GPIO_Pin_13); }

/* ========== 按键 GPIO 初始化 ========== */
void gpio_key_init(void)
{
    GPIO_InitTypeDef gpio;

    /* PA4 - 上拉输入, 按键按下为低电平 */
    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
}

uint8_t gpio_key_is_pressed(void)
{
    return (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == Bit_RESET);
}

/* ========== PWM 初始化 ========== */
void pwm_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    /* --- GPIO: PA0(TIM2_CH1), PA1(TIM2_CH2) --- */
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;       /* 复用推挽输出 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* --- TIM2 时钟 --- */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* --- TIM2 时基 --- */
    /* PWM 频率 = 72MHz / (71+1) / (999+1) = 1kHz */
    tim.TIM_Prescaler = 71;         /* PSC */
    tim.TIM_Period = 999;           /* ARR */
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);

    /* --- CH1 (PA0) - 补光灯 --- */
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;               /* 初始占空比 0% */
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* --- CH2 (PA1) - 风扇 --- */
    oc.TIM_Pulse = 0;
    TIM_OC2Init(TIM2, &oc);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* 启动 TIM2 */
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    uart_printf("[PWM] Initialized: TIM2, Freq=1kHz, CH1=PA0(Light), CH2=PA1(Fan)\r\n");
}

/* ========== 占空比设置 ========== */
void pwm_light_set_duty(uint8_t duty)
{
    if (duty > 100) duty = 100;
    g_light_duty = duty;

    uint16_t pulse = (uint16_t)((uint32_t)duty * (PWM_PERIOD + 1) / 100);
    TIM_SetCompare1(TIM2, pulse);

    /* 同时控制继电器 */
    if (duty > 0) {
        gpio_light_on();
    } else {
        gpio_light_off();
    }
}

void pwm_fan_set_duty(uint8_t duty)
{
    if (duty > 100) duty = 100;
    g_fan_duty = duty;

    uint16_t pulse = (uint16_t)((uint32_t)duty * (PWM_PERIOD + 1) / 100);
    TIM_SetCompare2(TIM2, pulse);
}

uint8_t pwm_light_get_duty(void) { return g_light_duty; }
uint8_t pwm_fan_get_duty(void)   { return g_fan_duty; }

void pwm_light_stop(void) { TIM_SetCompare1(TIM2, 0); }
void pwm_fan_stop(void)   { TIM_SetCompare2(TIM2, 0); }
