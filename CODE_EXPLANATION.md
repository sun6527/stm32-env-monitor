# STM32 嵌入式环境监测与控制系统 —— 完整代码逐行讲解

---

## 一、项目概览

这是一个**智慧农业温室环境监测终端**，硬件为 STM32F103RBT6（ARM Cortex-M3, 72MHz, 20KB RAM, 128KB Flash），运行 FreeRTOS 实时操作系统。它自动采集温湿度（SHT30 传感器），通过 PI 闭环控制算法驱动 PWM 风扇和补光灯，通过串口上报 JSON 格式数据，并具备看门狗+任务监控的可靠性机制。

---

## 二、涉及的协议详解

### 2.1 I2C 总线协议（SHT30 传感器通信）

**物理层**：两根线——SCL（时钟）和 SDA（数据），开漏输出，需上拉电阻。本项目中 SCL = PB6, SDA = PB7。

**通信流程**：
```
主机（STM32）                          从机（SHT30 @ 0x44）
  │
  ├─ START: SCL高时SDA拉低                │
  ├─ 发送 7位地址(0x44<<1=0x88) + W(0)   │
  │                                        ├─ ACK(SDA拉低第9个时钟)
  ├─ 发送命令高字节(如0x24)                │
  │                                        ├─ ACK
  ├─ 发送命令低字节(如0x00)                │
  │                                        ├─ ACK
  ├─ STOP: SCL高时SDA拉高                 │
  │                                        │ [传感器开始测量，最多15ms]
  ├─ START                                │
  ├─ 发送地址 + R(1)                      │
  │                                        ├─ ACK
  ├─ 读第1字节(T_MSB) → 主机ACK          │
  ├─ 读第2字节(T_LSB) → 主机ACK          │
  ├─ 读第3字节(T_CRC) → 主机ACK          │
  ├─ 读第4字节(H_MSB) → 主机ACK          │
  ├─ 读第5字节(H_LSB) → 主机ACK          │
  ├─ 读第6字节(H_CRC) → 主机NACK         │  ← 最后字节不确认，告诉从机结束
  ├─ STOP                                 │
```

**SHT30 数据格式**：每个测量值（温度/湿度）由 2 字节原始数据 + 1 字节 CRC-8 组成。共 6 字节。

**CRC-8 校验**：多项式 `0x31`（即 `x^8 + x^5 + x^4 + 1`），初始值 `0xFF`。用于检测 I2C 总线上的电磁干扰导致的比特错误。

**数据转换公式**（来自 SHT30 数据手册 §4.13）：
- 温度(°C) = -45 + 175 × (原始值 / 65535)
- 湿度(%) = 100 × (原始值 / 65535)

### 2.2 UART + DMA 协议（串口通信）

**UART 配置**：USART1（PA9=TX, PA10=RX），115200 波特率，8 数据位，无校验，1 停止位（8N1）。

**DMA 工作原理**：DMA（直接存储器访问）控制器可以在 CPU 不参与的情况下，自动在内存和外设之间搬运数据。

- **TX（DMA1_Channel4）**：Normal 模式，一次性发送。CPU 只需配置好源地址和长度，DMA 自动逐字节搬到 USART1->DR。
- **RX（DMA1_Channel5）**：Circular 模式，环形缓冲区。DMA 持续从 USART1->DR 搬到 256 字节缓冲区，写满后自动回到开头覆盖旧数据。

**DMA vs 中断的对比**：

| 方式 | 500字节的开销 |
|------|-------------|
| 中断 | 500次进入ISR，每次保存/恢复上下文 ~12周期 = 6000周期 |
| DMA | 1次传输完成中断 + 硬件自动搬运 |

**环形缓冲区读取算法**：
```
写指针位置 = 缓冲区大小 - DMA_CNDTR（剩余传输计数）
如果 write_idx >= read_idx：可读 = write_idx - read_idx
如果 write_idx < read_idx：可读 = (BUF_SIZE - read_idx) + write_idx
```

**应用层协议**：基于文本行（`\r\n` 分隔）的简单命令协议：
- `AUTO` → 切换自动模式
- `MANUAL` → 切换手动模式
- `FAN:<0-100>` → 设置风扇占空比
- `LIGHT:<0-100>` → 设置补光灯占空比
- `STATUS` → 查询状态
- `HELP` → 列出命令

**数据上报格式**：JSON
```json
{"type":"status","temp":25.3,"humi":62.1,"fan":35,"light":0}
```

### 2.3 PWM 协议（定时器产生脉冲宽度调制）

**PWM 原理**：通过调节方波信号的高电平时间占比（占空比）来控制模拟设备。TIM2 是一个 16 位定时器，配置为向上计数模式：

```
计数器值
  ^
ARR(999) ──┐    ┌──────┐    ┌──────
           │    │      │    │
           │    │      │    │
CCR(500)───┼────┘      │    │      ← 比较值
           │           │    │
    0 ─────┴───────────┴────┴──────→ 时间

输出 (PWM1模式):
           ┌──┐       ┌──────┐
           │  │       │      │
           │  │       │      │
    ───────┘  └───────┘      └──────
    ←50%→ ←────── 50% ──────→
```

**频率计算**：
```
PWM频率 = TIM2时钟 / ((PSC+1) × (ARR+1))
       = 72,000,000 / (72 × 1000)
       = 1000 Hz = 1 kHz
```

- TIM2 挂载在 APB1 总线上，APB1 时钟 = 36MHz。但由于 APB1 预分频 ≠ 1，定时器时钟会 ×2，所以 TIM2 实际时钟 = 72MHz。
- 占空比分辨率 = 1000 步（ARR+1），每步 0.1%。

### 2.4 FreeRTOS 任务间通信协议

本系统使用 **消息队列（Queue）** 进行任务间通信，而非全局变量：

- `g_sensor_data_queue`：TempHumiTask → PWMControlTask，传递 `sensor_data_t`，深度 8
- `g_ctrl_cmd_queue`：CommTask → PWMControlTask，传递 `control_cmd_t`，深度 4
- `g_log_queue`：各任务 → CommTask，传递日志字符串，深度 16

**为什么用队列不用全局变量？**队列的 Send/Receive 操作内部有临界区保护，保证了数据完整性。全局变量在多任务并发读写时可能读到"半成品"数据。

---

## 三、逐文件逐行讲解

---

### 3.1 `Core/Inc/main.h` — 系统级定义头文件

```c
// 第1-4行：Doxygen 风格文件注释
/**
 * @file    main.h
 * @brief   主头文件 - 系统级定义
 */

// 第6-7行：头文件保护宏，防止重复包含
#ifndef MAIN_H
#define MAIN_H

// 第9-12行：标准库和 STM32 外设库头文件
#include "stm32f10x.h"   // STM32F10x 标准外设库，包含所有寄存器定义和驱动函数
#include <stdint.h>       // 标准整型：uint8_t, uint16_t, uint32_t 等
#include <string.h>       // 字符串操作：memcpy, memset, strncmp 等
#include <stdio.h>        // 格式化输入输出：snprintf, vsnprintf

// 第15-17行：系统时钟宏定义
#define SYSTEM_CLOCK_HZ         72000000UL   // HCLK = AHB 总线时钟，72MHz
#define APB1_CLOCK_HZ           36000000UL   // APB1 外设时钟，HCLK/2 = 36MHz
#define APB2_CLOCK_HZ           72000000UL   // APB2 外设时钟，HCLK/1 = 72MHz

// 第20-23行：每个任务的栈大小（单位：字 = 4字节）
#define STACK_SIZE_TEMP_HUMI    256     // 256字 = 1024字节，采集任务调用浮点运算需要较大栈
#define STACK_SIZE_PWM_CTRL     128     // 128字 = 512字节，控制任务逻辑简单
#define STACK_SIZE_COMM         256     // 256字 = 1024字节，通信任务有大量栈缓冲区
#define STACK_SIZE_MONITOR      128     // 128字 = 512字节，监控任务逻辑简单

// 第26-29行：任务优先级（数字越大优先级越高）
#define PRIO_TEMP_HUMI          3       // 最高-传感器采集最不能延迟
#define PRIO_PWM_CTRL           2       // 中-控制响应
#define PRIO_COMM               2       // 中-通信（与PWM同级，时间片轮转）
#define PRIO_MONITOR            1       // 低-系统监控不需要实时

// 第32-34行：队列长度
#define QUEUE_SENSOR_DATA_LEN   8       // 传感器数据队列最多缓存8条
#define QUEUE_CTRL_CMD_LEN      4       // 控制指令队列最多缓存4条
#define QUEUE_LOG_LEN           16      // 日志队列最多缓存16条

// 第37-42行：环境参数阈值
#define TEMP_HIGH_THRESHOLD     30.0f   // 温度超过30°C触发告警
#define TEMP_LOW_THRESHOLD      15.0f   // 温度低于15°C触发告警
#define HUMI_HIGH_THRESHOLD     85.0f   // 湿度超过85%触发告警
#define HUMI_LOW_THRESHOLD      40.0f   // 湿度低于40%触发告警
#define TEMP_TARGET             25.0f   // PI控制目标温度
#define HUMI_TARGET             60.0f   // PI控制目标湿度

// 第45-49行：PWM参数
#define PWM_PERIOD              999     // ARR寄存器值，对应1000步分辨率
#define PWM_FAN_MIN_DUTY        20      // 风扇最低20%占空比（避免停转卡死）
#define PWM_FAN_MAX_DUTY        100     // 风扇最高100%
#define PWM_LIGHT_MIN_DUTY      10      // 补光灯最低10%
#define PWM_LIGHT_MAX_DUTY      100     // 补光灯最高100%

// 第52行：看门狗超时
#define WATCHDOG_TIMEOUT_MS     5000    // IWDG 5秒超时

// 第55-60行：传感器数据结构体
typedef struct {
    float       temperature;    // 温度值 (°C)，浮点精度
    float       humidity;       // 湿度值 (%RH)
    uint32_t    timestamp_ms;   // 采集时间戳（从调度器启动算起的毫秒数）
    uint8_t     valid;          // 0=传感器离线/数据无效, 1=数据有效
} sensor_data_t;

// 第63-73行：控制指令类型枚举
typedef enum {
    CTRL_NONE = 0,          // 空指令
    CTRL_FAN_ON,            // 开风扇
    CTRL_FAN_OFF,           // 关风扇
    CTRL_FAN_SET_DUTY,      // 设置风扇占空比
    CTRL_LIGHT_ON,          // 开补光灯
    CTRL_LIGHT_OFF,         // 关补光灯
    CTRL_LIGHT_SET_DUTY,    // 设置补光灯占空比
    CTRL_AUTO_MODE,         // 切换到自动模式
    CTRL_MANUAL_MODE,       // 切换到手动模式
} control_cmd_type_t;

// 第75-78行：控制指令结构体
typedef struct {
    control_cmd_type_t  type;   // 指令类型
    uint8_t             value;  // 参数值（占空比 0-100）
} control_cmd_t;

// 第81-85行：系统工作模式
typedef enum {
    SYS_MODE_AUTO = 0,      // 自动控制模式（PI算法自动调节）
    SYS_MODE_MANUAL,        // 手动控制模式（通过串口命令控制）
    SYS_MODE_ALARM,         // 告警模式（传感器失效时风扇全开保安全）
} system_mode_t;

// 第87-92行：系统状态结构体
typedef struct {
    system_mode_t   mode;       // 当前工作模式
    uint8_t         fan_duty;   // 当前风扇占空比
    uint8_t         light_duty; // 当前补光灯占空比
    uint32_t        uptime_ms;  // 系统运行时间
} system_status_t;

#endif /* MAIN_H */
```

---

### 3.2 `Core/Src/main.c` — 主程序入口

```c
// 第1-16行：文件头注释，描述项目功能、硬件平台、RTOS版本
/**
 * @file    main.c
 * @brief   嵌入式环境监测与控制系统 - 主入口
 * 基于 STM32F103RBT6 + FreeRTOS 的温室环境监测终端
 * 功能：SHT30温湿度采集、PWM补光灯/风扇控制、UART+DMA数据上报、
 *        闭环阈值联动自动控制、看门狗+任务监控
 */

// 第18-22行：包含头文件
#include "main.h"
#include "FreeRTOS.h"       // FreeRTOS 内核
#include "task.h"           // 任务管理 API
#include "queue.h"          // 队列 API
#include "semphr.h"         // 信号量 API（虽然定义了但未使用）

// 第24-31行：包含项目各模块头文件
#include "sht30.h"              // SHT30 传感器驱动
#include "gpio_pwm.h"           // GPIO + PWM 驱动
#include "uart_dma.h"           // UART DMA 通信驱动
#include "temp_humi_task.h"     // 温湿度采集任务
#include "pwm_control_task.h"   // PWM 控制任务
#include "comm_task.h"          // 通信任务
#include "task_monitor.h"       // 监控任务

// 第34-36行：全局队列句柄定义（由 main.c 定义，其他文件通过 extern 引用）
QueueHandle_t g_sensor_data_queue = NULL;  // 传感器数据队列
QueueHandle_t g_ctrl_cmd_queue = NULL;     // 控制指令队列
QueueHandle_t g_log_queue = NULL;          // 日志队列

// 第39-42行：全局任务句柄定义（供 monitor 任务检查各任务状态）
TaskHandle_t g_temp_humi_task_handle = NULL;
TaskHandle_t g_pwm_ctrl_task_handle = NULL;
TaskHandle_t g_comm_task_handle = NULL;
TaskHandle_t g_monitor_task_handle = NULL;
```

#### `system_clock_config()` — 系统时钟配置（第45-77行）

```c
static void system_clock_config(void)
{
    ErrorStatus HSEStartUpStatus;

    // 第50行：复位所有 RCC 寄存器到默认值
    RCC_DeInit();

    // 第53-54行：使能外部 8MHz 高速晶振（HSE），等待起振
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS) {
        // 第58-59行：Flash 配置——开启预取缓冲、设置 2 个等待周期
        // 为什么是 2？因为 STM32F103 Flash 访问速度只有 ~24MHz，
        // 而 CPU 跑 72MHz，必须等 Flash 准备好才能读指令
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        FLASH_SetLatency(FLASH_Latency_2);

        // 第62-64行：配置总线时钟分频
        // AHB(HCLK) = SYSCLK / 1 = 72MHz
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        // APB2(PCLK2) = HCLK / 1 = 72MHz（高速外设：GPIO, USART1, SPI1）
        RCC_PCLK2Config(RCC_HCLK_Div1);
        // APB1(PCLK1) = HCLK / 2 = 36MHz（低速外设：TIM2~7, I2C, USART2~5）
        // 注意：TIM2~7 的定时器时钟 = APB1×2 = 72MHz（因为 APB1 分频 ≠ 1）
        RCC_PCLK1Config(RCC_HCLK_Div2);

        // 第67行：PLL 配置——HSE(8MHz) / 1 × 9 = 72MHz
        // 为什么不用 ×16 得到 128MHz？因为 72MHz 是 STM32F103 官方保证稳定频率
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

        // 第68-69行：使能 PLL，等待锁定（PLL 需要时间稳定输出）
        RCC_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);  // 忙等 PLL 就绪

        // 第71-72行：将系统时钟切换到 PLL 输出
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (RCC_GetSYSCLKSource() != 0x08);  // 确认切换成功（0x08 = PLL）
    } else {
        // 第74-76行：HSE 起振失败——进入死循环
        // 这是一个基础处理，更好的做法是 fallback 到 HSI（内部 8MHz RC）
        // HSI 的缺点是精度只有 ±1%（HSE 是 ±20ppm），UART 可能误码
        while (1);
    }
}
```

#### `peripheral_clock_init()` — 外设时钟使能（第80-92行）

```c
static void peripheral_clock_init(void)
{
    // 第82-86行：使能 APB2 总线上的外设时钟
    // GPIOA/GPIOB/GPIOC：PA0~PA15, PB6~PB7, PC13
    // USART1：调试串口
    // AFIO：复用功能 IO（PWM 输出等需要）
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC | RCC_APB2Periph_USART1 |
        RCC_APB2Periph_AFIO, ENABLE);

    // 第87-90行：使能 APB1 总线上的外设时钟
    // TIM2：PWM 定时器
    // USART2：预留的上位机通信（实际代码中未使用）
    // I2C1：SHT30 传感器通信
    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART2 |
        RCC_APB1Periph_I2C1, ENABLE);

    // 第91行：使能 AHB 总线上的 DMA1（UART 收发用）
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}
```

#### `nvic_config()` — 中断向量配置（第95-112行）

```c
static void nvic_config(void)
{
    // 第97行：设置优先级分组为 4（4 位全用于抢占优先级，0 位子优先级）
    // 这意味着中断之间可以互相抢占，但没有子优先级排序
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    NVIC_InitTypeDef nvic;

    // 第100-105行：DMA1 通道4（USART1_TX）中断配置
    nvic.NVIC_IRQChannel = DMA1_Channel4_IRQn;        // 中断号
    nvic.NVIC_IRQChannelPreemptionPriority = 5;       // 抢占优先级 5（较低）
    nvic.NVIC_IRQChannelSubPriority = 0;               // 子优先级（分组4下无意义）
    nvic.NVIC_IRQChannelCmd = ENABLE;                  // 使能中断
    NVIC_Init(&nvic);

    // 第107-111行：DMA1 通道5（USART1_RX）中断配置
    nvic.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;       // 同优先级，子优先级区分
    nvic.NVIC_IRQChannelSubPriority = 1;               // 子优先级略低
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}
```

#### `iwdg_init()` — 独立看门狗初始化（第115-125行）

```c
static void iwdg_init(void)
{
    // 第118行：使能 IWDG 寄存器写访问（默认写保护）
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    // 第119行：设置预分频器
    // LSI 内部低速时钟 ≈ 40kHz
    // 预分频 64 → 40kHz / 64 = 625Hz
    IWDG_SetPrescaler(IWDG_Prescaler_64);

    // 第121行：设置重装载值
    // 3125 / 625Hz = 5 秒超时
    // 为什么是 5 秒？因为最慢任务 MonitorTask 周期是 5 秒
    IWDG_SetReload(3125);

    // 第123-124行：重装载计数器（喂一次初始值），使能看门狗
    IWDG_ReloadCounter();
    IWDG_Enable();
    // 一旦使能，IWDG 就会开始倒数。必须在 5 秒内再次喂狗，否则 MCU 复位。
}
```

#### `main()` — 主函数（第128-173行）

```c
int main(void)
{
    // === 第130-133行：硬件初始化 ===
    system_clock_config();      // 1. 时钟（必须最先配，因为所有外设依赖时钟）
    peripheral_clock_init();    // 2. 外设时钟使能
    nvic_config();              // 3. 中断配置

    // === 第136-141行：外设驱动初始化 ===
    uart_dma_init();            // 1. 串口+DMA（最先，后续初始化需要打印日志）
    gpio_light_ctrl_init();     // 2. 补光灯继电器默认关
    gpio_key_init();            // 3. 按键输入
    pwm_init();                 // 4. PWM 定时器
    sht30_init(I2C1);           // 5. SHT30 传感器
    iwdg_init();                // 6. 看门狗（最后启，前面的初始化出问题不触发复位）

    // === 第143-147行：打印启动横幅 ===
    uart_printf("\r\n========================================\r\n");
    uart_printf("  STM32 Environmental Monitor v1.0\r\n");
    uart_printf("  MCU: STM32F103RBT6 @ 72MHz\r\n");
    uart_printf("  RTOS: FreeRTOS\r\n");
    uart_printf("========================================\r\n\r\n");

    // === 第150-152行：创建 FreeRTOS 内核对象 ===
    // 每个队列的创建参数：(队列长度, 每项字节数)
    g_sensor_data_queue = xQueueCreate(QUEUE_SENSOR_DATA_LEN, sizeof(sensor_data_t));
    g_ctrl_cmd_queue     = xQueueCreate(QUEUE_CTRL_CMD_LEN, sizeof(control_cmd_t));
    g_log_queue          = xQueueCreate(QUEUE_LOG_LEN, 128);  // 日志最大128字节

    // 第154-157行：队列创建失败检查（内存不足时可能失败）
    if (!g_sensor_data_queue || !g_ctrl_cmd_queue || !g_log_queue) {
        uart_printf("[FATAL] Queue creation failed!\r\n");
        while (1);  // 死循环——没有队列系统无法工作
    }

    // === 第160-163行：创建四个 FreeRTOS 任务 ===
    temp_humi_task_create();    // 优先级3, 500ms周期
    pwm_ctrl_task_create();     // 优先级2, 200ms周期
    comm_task_create();         // 优先级2, 200ms周期
    monitor_task_create();      // 优先级1, 5s周期

    uart_printf("[INIT] All tasks created, starting scheduler...\r\n");

    // === 第168行：启动 FreeRTOS 调度器 ===
    // 此后 CPU 控制权交给 FreeRTOS，vTaskStartScheduler() 不会返回
    vTaskStartScheduler();

    // 第171-172行：如果调度器启动失败（如内存不足），到这里
    uart_printf("[FATAL] Scheduler failed to start!\r\n");
    while (1);
}
```

---

### 3.3 `Core/Inc/sht30.h` + `Core/Src/drivers/sht30.c` — SHT30 驱动

#### 头文件（sht30.h）

```c
// 第16行：SHT30 I2C 地址（ADDR 引脚接 GND 时默认 0x44）
#define SHT30_I2C_ADDR          0x44

// 第19-22行：SHT30 命令字（来自数据手册 Table 9-10）
#define SHT30_CMD_SINGLE_HIGH   0x2400  // 单次测量，高重复性，精度最高，耗时12.5ms
#define SHT30_CMD_SINGLE_MED    0x240B  // 中重复性，6ms（项目中未使用）
#define SHT30_CMD_SINGLE_LOW    0x2416  // 低重复性，3ms（项目中未使用）
#define SHT30_CMD_SOFT_RESET    0x30A2  // 软件复位——等同于上电复位

// 第25-26行：状态寄存器命令
#define SHT30_CMD_READ_STATUS   0xF32D  // 读状态寄存器（含告警标志位）
#define SHT30_CMD_CLEAR_STATUS  0x3041  // 清除状态寄存器

// 第29-41行：函数声明
int  sht30_init(I2C_TypeDef *i2c);                          // 初始化 I2C 和传感器
int  sht30_measure(I2C_TypeDef *i2c, float *temp, float *humi); // 测量温湿度
int  sht30_read_status(I2C_TypeDef *i2c, uint16_t *status); // 读状态寄存器
void sht30_reset(I2C_TypeDef *i2c);                          // 软件复位
uint8_t sht30_crc8(const uint8_t *data, int len);            // CRC-8 校验
```

#### 实现文件（sht30.c）

**CRC-8 校验（第14-28行）**：
```c
uint8_t sht30_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;                    // 初始值 0xFF
    for (int i = 0; i < len; i++) {        // 逐字节处理
        crc ^= data[i];                    // 将数据字节异或到 CRC
        for (int j = 0; j < 8; j++) {      // 逐位处理（8 位）
            if (crc & 0x80)                // 如果最高位是 1
                crc = (crc << 1) ^ 0x31;   // 左移后与多项式 0x31 异或
            else
                crc <<= 1;                 // 否则只左移
        }
    }
    return crc;
}
```
多项式 `0x31` = `x^8 + x^5 + x^4 + 1`。这是 Sensirion 专为温湿度传感器定义的 CRC。

**I2C 发送命令（第31-54行）**：
```c
static int sht30_send_command(I2C_TypeDef *i2c, uint16_t cmd)
{
    uint8_t buf[2];
    buf[0] = (cmd >> 8) & 0xFF;  // 高字节（如 0x24）
    buf[1] = cmd & 0xFF;         // 低字节（如 0x00）

    // 第38-39行：生成起始条件（SCL高时SDA↓）
    I2C_GenerateSTART(i2c, ENABLE);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT));
    // ↑ 等待 EV5 事件：起始条件已发送，主机模式已选定

    // 第42-43行：发送 7 位地址 + 写方向位
    // SHT30_I2C_ADDR << 1 = 0x88（7位地址左移1位，最低位=0表示写）
    I2C_Send7bitAddress(i2c, SHT30_I2C_ADDR << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    // ↑ 等待 EV6 事件：地址已发送，收到 ACK

    // 第46-49行：发送 2 字节命令
    for (int i = 0; i < 2; i++) {
        I2C_SendData(i2c, buf[i]);
        while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
        // ↑ 等待 EV8 事件：字节发送完成，收到 ACK
    }

    // 第52行：生成停止条件（SCL高时SDA↑）
    I2C_GenerateSTOP(i2c, ENABLE);
    return 0;
}
```

**初始化（第57-82行）**：
```c
int sht30_init(I2C_TypeDef *i2c)
{
    I2C_InitTypeDef i2c_init;

    // 第62-68行：I2C 配置参数
    i2c_init.I2C_ClockSpeed = 100000;           // 100kHz（标准模式）
    i2c_init.I2C_Mode = I2C_Mode_I2C;           // I2C 模式（非 SMBus）
    i2c_init.I2C_DutyCycle = I2C_DutyCycle_2;   // 占空比 2:1（标准）
    i2c_init.I2C_OwnAddress1 = 0x00;            // 本机地址（主机模式不需要）
    i2c_init.I2C_Ack = I2C_Ack_Enable;          // 使能 ACK 应答
    i2c_init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 7位地址
    I2C_Init(i2c, &i2c_init);
    I2C_Cmd(i2c, ENABLE);

    // 第72行：延时等待传感器上电稳定（忙等约 1ms @ 72MHz）
    for (volatile int i = 0; i < 72000; i++);
    // volatile 防止编译器优化掉空循环

    // 第75行：软件复位传感器（确保初始状态干净）
    sht30_reset(i2c);

    // 第78行：复位后等待 2ms（传感器内部重新校准需要时间）
    for (volatile int i = 0; i < 144000; i++);

    uart_printf("[SHT30] Initialized (I2C addr: 0x%02X)\r\n", SHT30_I2C_ADDR);
    return 0;
}
```

**单次测量（第85-149行）**：
```c
int sht30_measure(I2C_TypeDef *i2c, float *temperature, float *humidity)
{
    uint8_t data[6];        // 接收6字节：T_MSB, T_LSB, T_CRC, H_MSB, H_LSB, H_CRC
    uint16_t raw_temp, raw_humi;
    int timeout;

    // 第92-94行：发送测量命令（高重复性，精度最高）
    if (sht30_send_command(i2c, SHT30_CMD_SINGLE_HIGH) < 0) {
        return -1;
    }

    // 第97-99行：等待测量完成（高重复性模式最多 15ms）
    // 这里用忙等+NOP，比 vTaskDelay 更适合（vTaskDelay 会触发任务切换）
    for (volatile int i = 0; i < 15000; i++) {
        __NOP();  // 空操作，防止编译器优化
    }

    // 第102-106行：重新发起起始条件，准备读取
    timeout = 100000;
    I2C_GenerateSTART(i2c, ENABLE);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT)) {
        if (--timeout < 0) return -2;  // 超时保护：防止 I2C 总线卡死
    }

    // 第108-112行：发送地址 + 读方向
    I2C_Send7bitAddress(i2c, SHT30_I2C_ADDR << 1, I2C_Direction_Receiver);
    timeout = 100000;
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        if (--timeout < 0) return -2;
    }

    // 第114-127行：读取 6 字节
    for (int i = 0; i < 6; i++) {
        if (i == 5) {
            // 第116行：最后一字节前设置 NACK，告诉传感器"不读了"
            I2C_AcknowledgeConfig(i2c, DISABLE);
        }
        timeout = 100000;
        while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
            if (--timeout < 0) {
                // 超时恢复：重开 ACK，发 STOP，防止总线锁死
                I2C_AcknowledgeConfig(i2c, ENABLE);
                I2C_GenerateSTOP(i2c, ENABLE);
                return -2;
            }
        }
        data[i] = I2C_ReceiveData(i2c);  // 读数据寄存器
    }

    // 第129-130行：恢复正常 ACK，发停止条件
    I2C_AcknowledgeConfig(i2c, ENABLE);
    I2C_GenerateSTOP(i2c, ENABLE);

    // 第133-140行：CRC 校验
    // data[0..1] 是温度原始值，data[2] 是其 CRC
    if (sht30_crc8(data, 2) != data[2]) {
        uart_printf("[SHT30] Temperature CRC error\r\n");
        return -3;  // CRC不匹配 → 数据可能被电磁干扰损坏
    }
    // data[3..4] 是湿度原始值，data[5] 是其 CRC
    if (sht30_crc8(data + 3, 2) != data[5]) {
        uart_printf("[SHT30] Humidity CRC error\r\n");
        return -3;
    }

    // 第143-144行：拼接 16 位原始值（大端序：先收到的字节是高位）
    raw_temp = ((uint16_t)data[0] << 8) | data[1];
    raw_humi = ((uint16_t)data[3] << 8) | data[4];

    // 第146-147行：转换为物理量（SHT30 数据手册公式）
    // 先转 float 再除法——如果先整数除 65535，会截断为 0
    *temperature = -45.0f + 175.0f * (float)raw_temp / 65535.0f;
    *humidity    = 100.0f * (float)raw_humi / 65535.0f;

    return 0;
}
```

**其余函数**：`sht30_read_status()`（第153-190行）和 `sht30_reset()`（第193-197行）的结构与 `sht30_send_command()` 相同，只是命令字不同。

---

### 3.4 `Core/Inc/gpio_pwm.h` + `Core/Src/drivers/gpio_pwm.c` — GPIO 和 PWM 驱动

#### 头文件

```c
// GPIO 控制 PC13（补光灯继电器）
void gpio_light_ctrl_init(void);
void gpio_light_on(void);   // PC13 置位 → 继电器吸合 → 灯亮
void gpio_light_off(void);  // PC13 复位 → 继电器断开 → 灯灭

// GPIO 按键 PA4（手动/自动模式切换）
void gpio_key_init(void);           // 上拉输入模式
uint8_t gpio_key_is_pressed(void);  // 返回 1=按下（低电平）, 0=未按下

// PWM 控制（TIM2）
void pwm_init(void);                     // PA0=CH1(补光灯), PA1=CH2(风扇)
void pwm_light_set_duty(uint8_t duty);   // duty: 0~100%
void pwm_fan_set_duty(uint8_t duty);
uint8_t pwm_light_get_duty(void);
uint8_t pwm_fan_get_duty(void);
void pwm_light_stop(void);
void pwm_fan_stop(void);
```

#### 实现文件

**GPIO 继电器（第23-37行）**：
```c
void gpio_light_ctrl_init(void) {
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_13;            // PC13
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;      // 推挽输出（驱动继电器）
    gpio.GPIO_Speed = GPIO_Speed_50MHz;     // 50MHz 驱动能力
    GPIO_Init(GPIOC, &gpio);
    gpio_light_off();  // 默认关
}

void gpio_light_on(void)  { GPIO_SetBits(GPIOC, GPIO_Pin_13); }   // PC13=高
void gpio_light_off(void) { GPIO_ResetBits(GPIOC, GPIO_Pin_13); } // PC13=低
```

**按键 GPIO（第40-53行）**：
```c
void gpio_key_init(void) {
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入——未按下时读到高电平
    GPIO_Init(GPIOA, &gpio);
}

uint8_t gpio_key_is_pressed(void) {
    // 按键按下将 PA4 拉到 GND → 读到低电平(Bit_RESET) → 返回 1
    return (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == Bit_RESET);
}
```

**PWM 初始化（第56-97行）**：
```c
void pwm_init(void) {
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    // 第63-66行：PA0 和 PA1 配置为复用推挽输出
    // 复用推挽 = GPIO 不由 GPIO 模块控制，而由 TIM2 外设接管
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;       // AF = Alternate Function
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // 第69行：TIM2 时钟使能（在 peripheral_clock_init 中已使能，重复调用无副作用）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 第73-77行：时基配置
    // 频率 = 72MHz / ((71+1) * (999+1)) = 1000 Hz
    tim.TIM_Prescaler = 71;                 // PSC：预分频器
    tim.TIM_Period = 999;                   // ARR：自动重装载（决定周期）
    tim.TIM_ClockDivision = TIM_CKD_DIV1;   // 输入时钟不分频
    tim.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM2, &tim);

    // 第80-85行：通道1 输出比较配置（PA0 = 补光灯）
    oc.TIM_OCMode = TIM_OCMode_PWM1;        // PWM 模式1：CNT<CCR时输出高
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;                       // CCR1 初始值 = 0（占空比 0%）
    oc.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
    TIM_OC1Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable); // 使能预装载（避免更新时产生毛刺）

    // 第88-90行：通道2 输出比较配置（PA1 = 风扇）
    oc.TIM_Pulse = 0;  // CCR2 初始值 = 0
    TIM_OC2Init(TIM2, &oc);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // 第93-94行：使能 ARR 预装载，启动 TIM2
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}
```

**占空比设置（第100-129行）**：
```c
void pwm_light_set_duty(uint8_t duty) {
    if (duty > 100) duty = 100;          // 上限保护

    // 第105行：占空比 → 比较值
    // (uint32_t)duty 防止溢出：duty=100 时, 100*1000=100000 超出 uint16_t(65535)
    // 先转 uint32_t(4294967295范围)计算，再截断为 uint16_t
    uint16_t pulse = (uint16_t)((uint32_t)duty * (PWM_PERIOD + 1) / 100);
    TIM_SetCompare1(TIM2, pulse);        // 设置 CCR1 = pulse

    // 第109-113行：同步控制继电器（占空比>0 时开，=0 时关）
    if (duty > 0) gpio_light_on();
    else gpio_light_off();
}

void pwm_fan_set_duty(uint8_t duty) {
    if (duty > 100) duty = 100;
    uint16_t pulse = (uint16_t)((uint32_t)duty * (PWM_PERIOD + 1) / 100);
    TIM_SetCompare2(TIM2, pulse);        // 设置 CCR2
}
```

---

### 3.5 `Core/Inc/uart_dma.h` + `Core/Src/drivers/uart_dma.c` — UART DMA 通信

#### 头文件

```c
#define UART_RX_BUF_SIZE    256   // 接收环形缓冲区大小
#define UART_TX_BUF_SIZE    256   // 发送缓冲区大小

void uart_dma_init(void);           // 初始化 USART1 + DMA
int  uart_dma_send(const uint8_t *data, uint16_t len);  // 发送数据
uint16_t uart_dma_rx_available(void);  // 查询可读字节数
uint16_t uart_dma_rx_read(uint8_t *buf, uint16_t max_len); // 读取数据
void uart_printf(const char *fmt, ...);  // 格式化输出
void uart_send_sensor_json(float temp, float humi, uint8_t fan, uint8_t light);
```

#### 实现文件

**静态变量（第17-24行）**：
```c
static uint8_t g_uart_tx_buf[UART_TX_BUF_SIZE]; // 发送缓冲区（DMA 从这里读）
static uint8_t g_uart_rx_buf[UART_RX_BUF_SIZE]; // 接收环形缓冲区（DMA 写到这里）
static volatile uint16_t g_rx_read_idx = 0;      // CPU 读索引（环形缓冲区读指针）
static volatile uint8_t g_tx_busy = 0;           // 发送忙标志
```

**DMA 初始化（第27-93行）**：
```c
void uart_dma_init(void) {
    // ----- GPIO 配置（第33-43行）-----
    // PA9 = USART1_TX：复用推挽输出
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;    // AF=复用功能，由USART外设控制
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // PA10 = USART1_RX：浮空输入
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入——由外部信号驱动
    GPIO_Init(GPIOA, &gpio);

    // ----- USART1 配置（第46-58行）-----
    usart.USART_BaudRate = 115200;                       // 波特率 115200
    usart.USART_WordLength = USART_WordLength_8b;         // 8 数据位
    usart.USART_StopBits = USART_StopBits_1;              // 1 停止位
    usart.USART_Parity = USART_Parity_No;                 // 无校验
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无流控
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;     // 收发双工
    USART_Init(USART1, &usart);

    // 第55-56行：使能 USART1 的 DMA 请求
    // 当 USART 发送寄存器空时 → 触发 DMA TX 请求
    // 当 USART 接收寄存器非空时 → 触发 DMA RX 请求
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART1, ENABLE);

    // ----- DMA1 通道4：USART1_TX（第61-74行）-----
    DMA_DeInit(DMA1_Channel4);                           // 复位到默认状态
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;  // 外设地址 = USART数据寄存器
    dma.DMA_MemoryBaseAddr = (uint32_t)g_uart_tx_buf;    // 内存地址 = 发送缓冲区
    dma.DMA_DIR = DMA_DIR_PeripheralDST;                 // 方向：内存→外设
    dma.DMA_BufferSize = 0;                              // 初始传输数量=0（发送时动态设）
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   // 外设地址不变（始终是DR）
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;            // 内存地址递增
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 8位传输
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;                      // 普通模式（发完停止）
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_M2M = DMA_M2M_Disable;                       // 非内存到内存
    DMA_Init(DMA1_Channel4, &dma);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);      // 传输完成中断

    // ----- DMA1 通道5：USART1_RX（第77-92行）-----
    DMA_DeInit(DMA1_Channel5);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)g_uart_rx_buf;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;                 // 方向：外设→内存
    dma.DMA_BufferSize = UART_RX_BUF_SIZE;               // 缓冲区大小（环形）
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;                    // ★环形模式★
    // 环形模式：DMA 写到缓冲区末尾后自动回到开头，覆盖旧数据
    dma.DMA_Priority = DMA_Priority_High;                // RX 优先级高于 TX
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &dma);

    // 第92行：立即启动接收（不需要等串口数据——DMA 会等待外设请求）
    DMA_Cmd(DMA1_Channel5, ENABLE);
}
```

**DMA 发送（第96-118行）**：
```c
int uart_dma_send(const uint8_t *data, uint16_t len) {
    if (len == 0 || len > UART_TX_BUF_SIZE) return -1;

    // 第101-103行：等待上一次发送完成（忙等+超时）
    uint32_t timeout = 100000;
    while (g_tx_busy) {
        if (--timeout == 0) return -2;  // 超时——上一次发送可能卡住了
    }

    g_tx_busy = 1;  // 标记发送忙

    // 第109行：拷贝数据到发送缓冲区（DMA 会从这里读）
    memcpy(g_uart_tx_buf, data, len);

    // 第112-115行：配置 DMA 并启动
    // 必须先 DISABLE 才能修改 CNDTR 和 CMAR
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CNDTR = len;                       // 传输数量
    DMA1_Channel4->CMAR = (uint32_t)g_uart_tx_buf;    // 内存地址（可从开头开始）
    DMA_Cmd(DMA1_Channel4, ENABLE);

    return len;
}
```

**环形缓冲区读取（第121-139行）**：
```c
uint16_t uart_dma_rx_available(void) {
    // DMA_CNDTR 寄存器：当前还剩多少字节未传输
    // 初始值 = BUF_SIZE(256)，每收到1字节就减1
    // 环形模式下到0后自动重装为256
    uint16_t write_idx = UART_RX_BUF_SIZE - DMA1_Channel5->CNDTR;

    if (write_idx >= g_rx_read_idx) {
        // 正常情况：写指针在读指针前面
        // 例如：read=10, write=50 → 可读 40 字节
        return write_idx - g_rx_read_idx;
    } else {
        // 环形绕回：写指针已绕回开头，读指针还在末尾
        // 例如：read=200, write=30 → 可读 (256-200)+30 = 86 字节
        return UART_RX_BUF_SIZE - g_rx_read_idx + write_idx;
    }
}

uint16_t uart_dma_rx_read(uint8_t *buf, uint16_t max_len) {
    uint16_t count = 0;
    while (count < max_len && uart_dma_rx_available() > 0) {
        buf[count++] = g_uart_rx_buf[g_rx_read_idx];         // 读一个字节
        g_rx_read_idx = (g_rx_read_idx + 1) % UART_RX_BUF_SIZE; // 读指针前进
    }
    return count;  // 返回实际读取的字节数
}
```

**格式化输出（第142-165行）**：
```c
void uart_printf(const char *fmt, ...) {
    char buf[256];                          // 栈上 256 字节缓冲区
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);  // 格式化到 buf
    va_end(args);
    if (len > 0) {
        uart_dma_send((uint8_t*)buf, (uint16_t)len);
    }
}

void uart_send_sensor_json(float temp, float humi, uint8_t fan, uint8_t light) {
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "{\"type\":\"status\",\"temp\":%.1f,\"humi\":%.1f,\"fan\":%d,\"light\":%d}\r\n",
        temp, humi, fan, light);
    // JSON 格式：
    // {"type":"status","temp":25.3,"humi":62.1,"fan":35,"light":0}
    // \r\n 作为帧结束标记
    if (len > 0) uart_dma_send((uint8_t*)buf, (uint16_t)len);
}
```

**DMA 中断处理（第179-199行）**：
```c
// DMA1 通道4 中断（TX 完成）
void DMA1_Channel4_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC4)) {     // 传输完成标志
        DMA_ClearITPendingBit(DMA1_IT_TC4); // 清除标志
        DMA_Cmd(DMA1_Channel4, DISABLE);    // 停止 DMA 通道
        uart_dma_tx_complete_callback();    // → g_tx_busy = 0
    }
}

// DMA1 通道5 中断（RX 完成/半完成）
void DMA1_Channel5_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC5)) {     // 传输完成（256字节全满）
        DMA_ClearITPendingBit(DMA1_IT_TC5);
        uart_dma_rx_complete_callback(UART_RX_BUF_SIZE);
    }
    if (DMA_GetITStatus(DMA1_IT_HT5)) {     // 半传输完成（128字节）
        DMA_ClearITPendingBit(DMA1_IT_HT5);
        // 半传输中断确保 CPU 在数据被覆盖前有机会处理它
        uart_dma_rx_complete_callback(UART_RX_BUF_SIZE / 2);
    }
}
```

---

### 3.6 `Core/Src/tasks/temp_humi_task.c` — 温湿度采集任务

```c
// 第24行：连续错误阈值——5次失败后认为传感器离线
#define MAX_CONSECUTIVE_ERRORS  5
```

**数据校验（第27-38行）**：
```c
static int sensor_data_is_valid(float temp, float humi) {
    // 第一层：SHT30 物理极限检查
    if (temp < -40.0f || temp > 125.0f) return 0;  // 超出传感器量程
    if (humi < 0.0f || humi > 100.0f) return 0;    // 湿度不可能<0或>100

    // 第二层：温室场景合理范围检查（超出说明传感器故障或异常环境）
    if (temp < -10.0f || temp > 60.0f) return 0;   // 温室不可能 -10°C
    if (humi < 5.0f || humi > 99.0f) return 0;     // 极端干燥/饱和

    return 1;  // 数据合法
}
```

**阈值告警（第41-64行）**：
```c
static void check_threshold_alarm(float temp, float humi) {
    char log_buf[128];

    // 温度过高：>30°C → 告警
    if (temp > TEMP_HIGH_THRESHOLD) {
        snprintf(log_buf, sizeof(log_buf),
                 "[ALARM] Temp HIGH: %.1f°C > %.1f°C", temp, TEMP_HIGH_THRESHOLD);
        xQueueSend(g_log_queue, log_buf, 0);  // 非阻塞发送（0=不等）
    }
    // 温度过低：<15°C → 告警
    else if (temp < TEMP_LOW_THRESHOLD) { /* ... */ }

    // 湿度过高：>85% → 告警
    if (humi > HUMI_HIGH_THRESHOLD) { /* ... */ }
    // 湿度过低：<40% → 告警
    else if (humi < HUMI_LOW_THRESHOLD) { /* ... */ }
}
```

**任务主循环（第80-135行）**：
```c
void temp_humi_task_func(void *pvParameters) {
    (void)pvParameters;  // 未使用参数（消除编译器警告）
    sensor_data_t sensor_data;
    float temp, humi;
    uint32_t last_wake_time;
    int consecutive_errors = 0;

    // 第89行：启动延时 1 秒——等待 SHT30 和外设稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 第91行：记录初始时间戳（用于 vTaskDelayUntil 精确周期）
    last_wake_time = xTaskGetTickCount();

    while (1) {
        // 第97行：读 SHT30
        int ret = sht30_measure(I2C1, &temp, &humi);

        if (ret == 0 && sensor_data_is_valid(temp, humi)) {
            // === 成功路径 ===
            consecutive_errors = 0;  // 重置错误计数

            // 第103-106行：填充传感器数据结构
            sensor_data.temperature = temp;
            sensor_data.humidity = humi;
            // xTaskGetTickCount() 返回 tick 数，乘以 portTICK_PERIOD_MS(1ms) = 毫秒数
            sensor_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            sensor_data.valid = 1;  // 标记有效

            // 第109-111行：发送到控制任务队列（100ms 超时）
            if (xQueueSend(g_sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) != pdPASS) {
                // 队列满——控制任务消费不过来，丢弃本条数据
                uart_printf("[TempHumi] Queue full, data dropped\r\n");
            }

            // 第114行：检查是否需要告警
            check_threshold_alarm(temp, humi);

            // 第116行：调试输出
            uart_printf("[TempHumi] T=%.1f°C H=%.1f%%\r\n", temp, humi);

        } else {
            // === 失败路径 ===
            consecutive_errors++;

            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                // 第123行：连续5次失败 → 传感器离线
                uart_printf("[TempHumi] SHT30 fatal error, sensor offline!\r\n");

                // 第126-128行：发送 valid=0 的数据，通知控制任务
                // 控制任务收到后会切换到 ALARM 模式（风扇全开保安全）
                sensor_data.valid = 0;
                sensor_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                xQueueSend(g_sensor_data_queue, &sensor_data, 0);
            }
        }

        // 第133行：精确周期延时 500ms
        // vTaskDelayUntil 保证每次循环的间隔精确为 500ms（补偿执行时间）
        // 比普通 vTaskDelay 更精确，因为考虑了代码执行耗时
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(500));
    }
}
```

---

### 3.7 `Core/Src/tasks/control_algorithm.c` — PI 闭环控制算法

```c
// 第16-25行：默认 PI 控制参数
static control_params_t g_params = {
    .temp_kp        = 5.0f,     // 温度 P 系数：每偏离1°C，P项贡献5%占空比
    .temp_ki        = 0.5f,     // 温度 I 系数：累积误差
    .temp_kd        = 0.0f,     // 不使用微分（温室环境变化慢，D项对噪声敏感）
    .humi_kp        = 3.0f,
    .humi_ki        = 0.3f,
    .humi_kd        = 0.0f,
    .deadband_temp  = 0.5f,     // 温度死区 ±0.5°C：在此范围内不做调整
    .deadband_humi  = 2.0f,     // 湿度死区 ±2%RH
};

// 第28-29行：积分累积项（静态全局，跨函数调用保持）
static float g_temp_integral = 0.0f;
static float g_humi_integral = 0.0f;
```

**限幅辅助函数（第32-37行）**：
```c
static float clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}
```

**核心控制计算（第40-113行）**：
```c
int control_calculate(float current_temp, float current_humi,
                      uint8_t *fan_duty, uint8_t *light_duty) {
    int alarm = 0;
    float output;

    // ============ 温度控制（风扇）============
    float temp_error = current_temp - TEMP_TARGET;  // 误差 = 当前 - 目标(25°C)

    // 第50-53行：死区检测
    // fabsf = 浮点绝对值
    if (fabsf(temp_error) < g_params.deadband_temp) {  // 误差 < ±0.5°C
        temp_error = 0.0f;               // 强制误差为0 → 不做调整
        g_temp_integral *= 0.5f;         // 积分项减半衰减（慢慢忘掉旧误差）
    }

    if (temp_error > 0) {
        // === 温度偏高 → 增加风扇转速 ===
        // 第57行：积分累积（error * 0.2 是因为采样周期 200ms）
        g_temp_integral += temp_error * 0.2f;
        // 第58行：积分限幅（上限 50）——防止积分饱和
        g_temp_integral = clamp(g_temp_integral, 0.0f, 50.0f);

        // 第60行：PI 输出 = P项 + I项
        output = g_params.temp_kp * temp_error + g_params.temp_ki * g_temp_integral;
        // 第61行：映射到风扇占空比范围 [20%, 100%]
        *fan_duty = (uint8_t)clamp(output, PWM_FAN_MIN_DUTY, PWM_FAN_MAX_DUTY);

        if (current_temp > TEMP_HIGH_THRESHOLD) alarm = 1;  // >30°C 告警

    } else if (temp_error < -0.5f) {
        // === 温度偏低 → 降低风扇转速（节能）===
        g_temp_integral += temp_error * 0.2f;   // error 是负数，积分项减小
        g_temp_integral = clamp(g_temp_integral, -50.0f, 0.0f); // 下限-50

        output = g_params.temp_kp * temp_error + g_params.temp_ki * g_temp_integral;
        // 第71行：output 是负数 → 取反后映射到 [0, 20%]
        // 最低保持 20% 占空比维持空气流通
        *fan_duty = (uint8_t)clamp(-output, 0, PWM_FAN_MIN_DUTY);

        if (current_temp < TEMP_LOW_THRESHOLD) alarm = 1;  // <15°C 告警
    } else {
        // === 正常范围（-0.5~0°C，即 24.5~25.0°C）===
        g_temp_integral *= 0.5f;        // 积分衰减
        *fan_duty = PWM_FAN_MIN_DUTY;   // 最小风速维持
    }

    // ============ 湿度控制（补光灯）============
    float humi_error = current_humi - HUMI_TARGET;  // 误差 = 当前 - 目标(60%)

    // 第83-86行：死区检测（±2% RH）
    if (fabsf(humi_error) < g_params.deadband_humi) {
        humi_error = 0.0f;
        g_humi_integral *= 0.5f;
    }

    if (humi_error < 0) {
        // === 湿度过低 → 开补光灯 ===
        // 原理：补光灯产热 → 植物蒸腾加速 → 湿度上升
        g_humi_integral += humi_error * 0.2f;
        g_humi_integral = clamp(g_humi_integral, -30.0f, 0.0f);

        // 第93行：error 是负数，取反得正值 → 映射到补光灯占空比
        output = -(g_params.humi_kp * humi_error + g_params.humi_ki * g_humi_integral);
        *light_duty = (uint8_t)clamp(output, PWM_LIGHT_MIN_DUTY, PWM_LIGHT_MAX_DUTY);

        if (current_humi < HUMI_LOW_THRESHOLD) alarm = 1;

    } else if (humi_error > 5.0f) {
        // === 湿度过高（>65%）→ 关闭补光灯 ===
        g_humi_integral += humi_error * 0.2f;
        g_humi_integral = clamp(g_humi_integral, 0.0f, 30.0f);

        output = g_params.humi_kp * humi_error + g_params.humi_ki * g_humi_integral;
        // 第104行：output 大 → 占空比小（反向关系）
        *light_duty = (uint8_t)clamp(PWM_LIGHT_MIN_DUTY - output, 0, PWM_LIGHT_MIN_DUTY);

        if (current_humi > HUMI_HIGH_THRESHOLD) alarm = 1;
    } else {
        // === 正常范围 ===
        g_humi_integral *= 0.5f;
        *light_duty = 0;  // 关闭补光灯节能
    }

    return alarm;  // 返回值：0=正常, 1=有告警
}
```

---

### 3.8 `Core/Src/tasks/pwm_control_task.c` — PWM 控制任务

```c
// 第23行：初始模式为自动模式
static system_mode_t g_control_mode = SYS_MODE_AUTO;
```

**应用控制输出（第39-56行）**：
```c
static void apply_control(uint8_t fan_duty, uint8_t light_duty) {
    uint8_t prev_fan = pwm_fan_get_duty();      // 读当前占空比
    uint8_t prev_light = pwm_light_get_duty();

    // 第44-48行：只有实际变化时才更新 PWM
    // 避免不需要的寄存器写入（减少电磁干扰）
    if (fan_duty != prev_fan)   pwm_fan_set_duty(fan_duty);
    if (light_duty != prev_light) pwm_light_set_duty(light_duty);

    // 第52-55行：只在有变化时打印（减少串口噪声）
    if (fan_duty != prev_fan || light_duty != prev_light) {
        uart_printf("[Control] Fan: %d%% -> %d%%, Light: %d%% -> %d%%\r\n",
                    prev_fan, fan_duty, prev_light, light_duty);
    }
}
```

**任务主循环（第59-167行）**：
```c
void pwm_ctrl_task_func(void *pvParameters) {
    (void)pvParameters;
    sensor_data_t sensor_data;
    control_cmd_t cmd;
    uint32_t last_wake_time;
    uint8_t fan_duty = 0, light_duty = 0;
    int sensor_timeout = 0;   // 连续收到无效数据的次数

    vTaskDelay(pdMS_TO_TICKS(500));  // 启动延时
    last_wake_time = xTaskGetTickCount();

    while (1) {
        // === 第75-113行：处理控制指令队列 ===
        // xQueueReceive 超时=0（非阻塞，有就取，没有就跳过）
        while (xQueueReceive(g_ctrl_cmd_queue, &cmd, 0) == pdPASS) {
            switch (cmd.type) {
            case CTRL_AUTO_MODE:
                g_control_mode = SYS_MODE_AUTO;
                control_reset_integrator();  // ★关键：切换时重置积分项
                break;
            case CTRL_MANUAL_MODE:
                g_control_mode = SYS_MODE_MANUAL;
                break;
            case CTRL_FAN_SET_DUTY:
                // 第87-90行：只在手动模式下生效
                if (g_control_mode == SYS_MODE_MANUAL) {
                    fan_duty = cmd.value;
                    apply_control(fan_duty, light_duty);
                }
                break;
            // ... LIGHT, ON, OFF 同理
            }
        }

        // === 第116-143行：自动模式 ===
        if (g_control_mode == SYS_MODE_AUTO) {
            // 第117行：等待传感器数据（100ms 超时）
            if (xQueueReceive(g_sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) == pdPASS) {
                sensor_timeout = 0;  // 收到数据，重置超时计数

                if (sensor_data.valid) {
                    // 第121-128行：调用 PI 算法计算输出
                    int alarm = control_calculate(
                        sensor_data.temperature, sensor_data.humidity,
                        &fan_duty, &light_duty);
                    apply_control(fan_duty, light_duty);
                } else {
                    // 传感器无效 → 累加超时计数
                    sensor_timeout++;
                    if (sensor_timeout >= 3) {
                        // 第137-141行：3次无效 → 切换到 ALARM 模式
                        g_control_mode = SYS_MODE_ALARM;
                        apply_control(100, 0);  // 风扇全开，灯关（保安全）
                    }
                }
            }
        }

        // === 第147-163行：按键检测（手动/自动切换） ===
        if (gpio_key_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(50));  // 50ms 消抖延时
            if (gpio_key_is_pressed()) {    // 二次确认（消抖后仍按下）
                // 切换模式
                if (g_control_mode == SYS_MODE_AUTO) {
                    g_control_mode = SYS_MODE_MANUAL;
                } else {
                    g_control_mode = SYS_MODE_AUTO;
                    control_reset_integrator();  // 切回自动时重置积分
                }
                // 第159-161行：等待按键释放（阻塞直到松手）
                while (gpio_key_is_pressed()) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
            }
        }

        // 第165行：精确周期 200ms
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(200));
    }
}
```

---

### 3.9 `Core/Src/tasks/comm_task.c` — 通信任务

**命令解析（第27-73行）**：
```c
static void parse_command(const char *cmd) {
    control_cmd_t ctrl_cmd;
    memset(&ctrl_cmd, 0, sizeof(ctrl_cmd));  // 清零

    // strncmp(cmd, "AUTO", 4)：
    //   比较 cmd 的前 4 个字符是否为 "AUTO"
    //   用户输入 "AUTO" 或 "AUTO\r\n" 都能匹配
    if (strncmp(cmd, "AUTO", 4) == 0) {
        ctrl_cmd.type = CTRL_AUTO_MODE;
        xQueueSend(g_ctrl_cmd_queue, &ctrl_cmd, 0);
    }
    else if (strncmp(cmd, "MANUAL", 6) == 0) {
        ctrl_cmd.type = CTRL_MANUAL_MODE;
        xQueueSend(g_ctrl_cmd_queue, &ctrl_cmd, 0);
    }
    else if (strncmp(cmd, "FAN:", 4) == 0) {
        // atoi(cmd + 4)：跳过 "FAN:" 前缀，解析后续数字
        int duty = atoi(cmd + 4);       // "FAN:50" → duty=50
        if (duty >= 0 && duty <= 100) {
            ctrl_cmd.type = CTRL_FAN_SET_DUTY;
            ctrl_cmd.value = (uint8_t)duty;
            xQueueSend(g_ctrl_cmd_queue, &ctrl_cmd, 0);
        }
    }
    else if (strncmp(cmd, "LIGHT:", 6) == 0) {
        int duty = atoi(cmd + 6);       // 跳过 "LIGHT:"(6字符)
        // ... 同上
    }
    else if (strncmp(cmd, "STATUS", 6) == 0) {
        // 立即查询当前状态
        uart_send_sensor_json(0, 0, pwm_fan_get_duty(), pwm_light_get_duty());
    }
    else if (strncmp(cmd, "HELP", 4) == 0) {
        // 打印帮助信息
        uart_printf("[Comm] Commands:\r\n");
        uart_printf("  AUTO / MANUAL - mode switch\r\n");
        // ...
    }
    else {
        uart_printf("[Comm] Unknown CMD: %s\r\n", cmd);
    }
}
```

**接收指令解析（第76-92行）**：
```c
static void check_rx_commands(void) {
    uint8_t byte;
    // 逐字节读取 DMA 环形缓冲区
    while (uart_dma_rx_available() > 0) {
        uart_dma_rx_read(&byte, 1);  // 每次读 1 字节

        if (byte == '\r' || byte == '\n') {
            // 遇到换行符 → 一行命令结束
            if (g_cmd_pos > 0) {
                g_cmd_buf[g_cmd_pos] = '\0';      // 添加字符串结束符
                parse_command((const char*)g_cmd_buf); // 解析命令
                g_cmd_pos = 0;  // 重置缓冲区位置
            }
        } else if (g_cmd_pos < sizeof(g_cmd_buf) - 1) {
            // 普通字符 → 追加到命令缓冲区（留1字节给 '\0'）
            g_cmd_buf[g_cmd_pos++] = byte;
        }
    }
}
```

**任务主循环（第117-145行）**：
```c
void comm_task_func(void *pvParameters) {
    (void)pvParameters;
    uint32_t last_wake_time;
    vTaskDelay(pdMS_TO_TICKS(500));
    last_wake_time = xTaskGetTickCount();

    while (1) {
        // 第130行：处理接收到的命令
        check_rx_commands();

        // 第133行：发送积累的日志
        flush_logs();  // 从 g_log_queue 非阻塞取日志并串口输出

        // 第136-141行：定期状态上报（每 REPORT_INTERVAL_MS/200 = 10 次循环=2秒）
        static int report_counter = 0;
        if (++report_counter >= (REPORT_INTERVAL_MS / 200)) {  // 2000/200 = 10
            report_counter = 0;
            uart_send_sensor_json(0, 0, pwm_fan_get_duty(), pwm_light_get_duty());
        }

        // 第143行：周期 200ms
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(200));
    }
}
```

---

### 3.10 `Core/Src/tasks/task_monitor.c` — 任务监控

```c
// 第19-21行：引用其他文件定义的任务句柄
extern TaskHandle_t g_temp_humi_task_handle;
extern TaskHandle_t g_pwm_ctrl_task_handle;
extern TaskHandle_t g_comm_task_handle;
```

**栈水位检查（第24-36行）**：
```c
static void check_task_stack(TaskHandle_t task, const char *name, uint16_t min_free) {
    if (task == NULL) {  // 任务可能还未创建
        uart_printf("[Monitor] %s: NULL handle!\r\n", name);
        return;
    }
    // uxTaskGetStackHighWaterMark：返回任务栈中从未被使用过的剩余栈（单位：字）
    // 值越小 → 栈越接近溢出
    UBaseType_t high_water = uxTaskGetStackHighWaterMark(task);
    if (high_water < min_free) {
        uart_printf("[Monitor] %s: LOW STACK! Free=%u words (min=%u)\r\n",
                    name, (unsigned)high_water, min_free);
    }
}
```

**任务状态检查（第39-58行）**：
```c
static void check_task_state(TaskHandle_t task, const char *name) {
    if (task == NULL) return;

    eTaskState state = eTaskGetState(task);  // 获取任务当前状态

    // 只对异常状态告警
    if (state == eSuspended || state == eDeleted || state == eInvalid) {
        uart_printf("[Monitor] %s: ABNORMAL state=%s!\r\n", name, state_str);
    }
}
```

**监控主循环（第74-123行）**：
```c
void monitor_task_func(void *pvParameters) {
    (void)pvParameters;
    uint32_t last_wake_time;
    uint32_t tick_count = 0;

    vTaskDelay(pdMS_TO_TICKS(2000));  // 启动延时 2s
    last_wake_time = xTaskGetTickCount();

    while (1) {
        tick_count++;

        // 第89行：★每次循环都喂狗★（5秒周期，看门狗5秒超时→刚好）
        IWDG_ReloadCounter();

        // 第92行：每 10 个周期（约 50 秒）做一次完整健康检查
        if (tick_count % 10 == 0) {
            // 第96-98行：检查所有任务的栈水位
            // TempHumi 和 PWMCtrl 阈值 32 words = 128 bytes
            // Comm 阈值 64 words（通信任务栈使用更多）
            check_task_stack(g_temp_humi_task_handle, "TempHumi", 32);
            check_task_stack(g_pwm_ctrl_task_handle,  "PWMCtrl",  32);
            check_task_stack(g_comm_task_handle,       "Comm",     64);

            // 第101-104行：检查所有任务状态（包括自己）
            check_task_state(g_temp_humi_task_handle, "TempHumi");
            check_task_state(g_pwm_ctrl_task_handle,  "PWM Ctrl");
            check_task_state(g_comm_task_handle,       "Comm");
            check_task_state(g_monitor_task_handle,    "Monitor");

            // 第107-110行：堆内存统计
            size_t free_heap = xPortGetFreeHeapSize();              // 当前空闲堆
            size_t min_ever = xPortGetMinimumEverFreeHeapSize();    // 历史最低空闲堆
            uart_printf("[Monitor] Heap: free=%u, min_ever=%u\r\n",
                        (unsigned)free_heap, (unsigned)min_ever);

            // 第113行：当前运行的任务总数
            unsigned task_count = uxTaskGetNumberOfTasks();

            // 第117-118行：系统运行时间（tick → 秒）
            uint32_t uptime = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
            uart_printf("[Monitor] Uptime: %lu s\r\n", uptime);
        }

        // 第121行：精确周期 5 秒
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5000));
    }
}
```

---

### 3.11 `Drivers/FreeRTOSConfig.h` — FreeRTOS 内核配置

```c
// 第12行：抢占式调度——高优先级任务可抢占低优先级任务
#define configUSE_PREEMPTION                    1

// 第13行：空闲钩子——不需要在空闲任务中执行自定义代码
#define configUSE_IDLE_HOOK                     0

// 第14行：Tick 钩子——每次时钟节拍中断时调用（可用于性能统计）
#define configUSE_TICK_HOOK                     1

// 第15行：CPU 时钟频率——72MHz，FreeRTOS 用于计算延时
#define configCPU_CLOCK_HZ                      (72000000)

// 第16行：时钟节拍频率——1000Hz = 每 1ms 产生一次中断
// 这意味着 vTaskDelay(1) = 延时 1ms
#define configTICK_RATE_HZ                      (1000)

// 第17行：最大优先级数——5个级别(0~4)，本项目用了1/2/3
#define configMAX_PRIORITIES                    (5)

// 第18行：最小任务栈——64字 = 256字节（Idle 任务等使用）
#define configMINIMAL_STACK_SIZE                (64)

// 第19行：总堆大小——15KB
// 4个任务各分配栈 + 3个队列 + 内核开销 = 总计约10KB，留5KB余量
#define configTOTAL_HEAP_SIZE                   (15 * 1024)

// 第20行：任务名最大长度——16字符
#define configMAX_TASK_NAME_LEN                 (16)

// 第21行：Trace 工具——使能后可调用 uxTaskGetStackHighWaterMark 等
#define configUSE_TRACE_FACILITY                1

// 第22行：使用 32 位 Tick（0=使用16位，但长时间运行会溢出）
#define configUSE_16_BIT_TICKS                  0

// 第24-29行：使能的内核功能
#define configUSE_MUTEXES                       1  // 互斥锁（本项目未用但保留）
#define configUSE_RECURSIVE_MUTEXES             1  // 递归互斥锁
#define configUSE_COUNTING_SEMAPHORES           1  // 计数信号量
#define configUSE_TIMERS                        1  // 软件定时器

// 第26行：栈溢出检测方法2
// 方法2：在任务栈底部放置已知模式，切换任务时检查是否被破坏
#define configCHECK_FOR_STACK_OVERFLOW          2

// 第37-38行：使用动态内存分配（heap_4.c），不使用静态分配
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1

// 第43行：内存分配失败钩子
#define configUSE_MALLOC_FAILED_HOOK            1

// 第69行：内核中断优先级（Cortex-M3 使用低4位 = 15）
#define configKERNEL_INTERRUPT_PRIORITY         255

// 第70行：可在 ISR 中调用 FreeRTOS API 的最高优先级
// 191 = 0xBF → 优先级 11（0~15 中 11 及以上不能调用 API）
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    191

// 第75-90行：选择性包含的 API 函数
#define INCLUDE_vTaskPrioritySet                1  // 可动态改任务优先级
#define INCLUDE_uxTaskPriorityGet               1  // 可查询任务优先级
#define INCLUDE_vTaskDelete                     1  // 可删除任务
#define INCLUDE_vTaskSuspend                    1  // 可挂起任务
#define INCLUDE_vTaskDelayUntil                 1  // 精确周期延时
#define INCLUDE_uxTaskGetStackHighWaterMark     1  // 栈水位查询
#define INCLUDE_eTaskGetState                   1  // 任务状态查询

// 第95行：断言宏——条件为假时关中断并死循环
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

// 第100-102行：将 FreeRTOS 的 ISR 名映射到 STM32 标准外设库的中断向量名
#define vPortSVCHandler     SVC_Handler       // FreeRTOS 启动第一个任务
#define xPortPendSVHandler  PendSV_Handler    // 任务上下文切换
#define xPortSysTickHandler SysTick_Handler   // 时钟节拍
```

---

## 四、系统数据流全景

```
        ┌─────────────────────────────────────────────────────┐
        │                  SHT30 传感器                        │
        │            I2C1 (PB6=SCL, PB7=SDA)                  │
        └────────────────────┬────────────────────────────────┘
                             │ I2C 6字节 + CRC校验
                             ▼
        ┌─────────────────────────────────────────────────────┐
        │  TempHumiTask [PRIO 3, 500ms]                       │
        │  · sht30_measure() 读取                             │
        │  · sensor_data_is_valid() 校验                      │
        │  · check_threshold_alarm() → 告警日志 → g_log_queue │
        │  · xQueueSend → g_sensor_data_queue                 │
        └────────────────────┬────────────────────────────────┘
                             │ sensor_data_t (队列深度8)
                             ▼
        ┌─────────────────────────────────────────────────────┐
        │  PWMControlTask [PRIO 2, 200ms]                     │
        │  · xQueueReceive ← g_sensor_data_queue              │
        │  · xQueueReceive ← g_ctrl_cmd_queue                 │
        │  · control_calculate() PI算法 → fan_duty, light_duty│
        │  · apply_control() → pwm_fan_set_duty / light       │
        └──┬──────────────────────┬───────────────────────────┘
           │ PWM                  │ 继电器
           ▼                      ▼
    ┌──────────┐          ┌──────────────┐
    │ TIM2 PWM │          │ GPIO PC13     │
    │ PA0=灯   │          │ 补光灯继电器   │
    │ PA1=风扇 │          └──────────────┘
    └──────────┘

        ┌─────────────────────────────────────────────────────┐
        │  CommTask [PRIO 2, 200ms]                           │
        │  · check_rx_commands() ← UART DMA RX (环形缓冲区)    │
        │  · parse_command() → g_ctrl_cmd_queue               │
        │  · flush_logs() ← g_log_queue → uart_printf()       │
        │  · uart_send_sensor_json() 每2秒上报                │
        └─────────────────────────────────────────────────────┘
                             │ USART1 DMA TX/RX
                             ▼
                    ┌─────────────────┐
                    │  USB-TTL 模块    │
                    │  115200 8N1     │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  上位机 (PC)     │
                    │  串口终端        │
                    └─────────────────┘

        ┌─────────────────────────────────────────────────────┐
        │  MonitorTask [PRIO 1, 5s]                           │
        │  · IWDG_ReloadCounter() 喂狗                        │
        │  · check_task_stack() 栈水位 (每50s)                │
        │  · check_task_state() 任务状态                      │
        │  · xPortGetFreeHeapSize() 堆统计                    │
        └─────────────────────────────────────────────────────┘
```

---

## 五、关键设计决策总结

| 决策 | 选择 | 原因 |
|------|------|------|
| RTOS vs 裸机 | FreeRTOS | 4个并发任务对实时性要求不同，裸机 super loop 难以管理 |
| 调度策略 | 抢占式 | 传感器采集必须能打断低优先级任务 |
| 任务通信 | 消息队列 | 临界区保护，比全局变量安全 |
| 控制算法 | PI（无D） | 温室惯性大，D项对噪声太敏感 |
| PWM 频率 | 1kHz | 1000步分辨率，可接受轻微噪音 |
| PWM 占空比下限 | 风扇20%/灯10% | 避免完全停转卡死 |
| USART RX | DMA 环形 | 500字节只需1次中断 vs 500次中断 |
| USART TX | DMA 普通 | 发送完成即停止，不浪费 |
| I2C 速度 | 100kHz | 标准模式，兼容性好 |
| 看门狗超时 | 5秒 | 匹配最慢任务周期 |
| 栈溢出检测 | 方法2 | 运行时检查栈底已知模式 |
| 传感器失效策略 | 3次→ALARM | 风扇全开保安全，不冒险 |

---

## 六、涉及协议汇总

| 协议层 | 协议 | 用途 |
|--------|------|------|
| 物理层 | I2C 总线 (100kHz) | SHT30 传感器通信 |
| 物理层 | UART (115200 8N1) | 上位机调试通信 |
| 数据链路层 | CRC-8 (多项式 0x31) | SHT30 数据传输完整性校验 |
| 传输层 | DMA (STM32 片上控制器) | UART 数据搬运，释放 CPU |
| 应用层 | 文本命令行 (\\r\\n 分隔) | 上位机 → 终端 指令通信 |
| 应用层 | JSON | 终端 → 上位机 状态上报 |
| 操作系统层 | FreeRTOS Message Queue | 任务间数据交换 |
| 控制层 | PI 闭环控制 + Deadband | 温湿度自动调节 |
