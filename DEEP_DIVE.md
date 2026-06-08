# STM32嵌入式环境监测与控制系统 —— 深度讲解

> 读完本文后，你应该能回答：为什么 FreeRTOS 任务这样分配优先级？PWM 频率为什么是 1kHz？PI 控制的死区是什么意思？DMA 比中断好在哪？

---

## 一、项目背景

### 1.1 场景

智慧农业温室：
- 面积 500㎡，种植高价值作物（如兰花）
- 需要温湿度精确控制：22~28°C, 50~70% RH
- 传统方案：人工定时查看温湿度计 → 手动开关风扇/灯 → **效率低、响应慢**
- 本方案：STM32 终端自动采集 → 自动调节 → 串口上报数据

### 1.2 硬件选型

| 组件 | 型号 | 为何选它 |
|------|------|----------|
| MCU | STM32F103RBT6 | 72MHz Cortex-M3，20KB RAM，128KB Flash，量产价约 ¥8 |
| 温湿度 | SHT30 | I2C 接口，±0.3°C 精度，3.3V 供电 |
| 风扇 | 5V PWM 风扇 | TIM2_CH2 (PA1) 输出 PWM 调速 |
| 补光灯 | LED + 继电器 | TIM2_CH1 (PA0) + GPIO PC13 |
| 调试 | USART1 | 115200 bps，接 USB-TTL 模块 |

**为什么选 STM32F103 而不是 ESP32 或 Arduino？**

```
面试要点：
1. 工业级温度范围（-40~85°C），温室夏天可达 50°C，ESP32 商规仅 0~70°C
2. 5 个 USART、2 个 I2C、2 个 SPI——外设丰富，可扩展
3. 标准外设库成熟稳定，工业界验证了十几年
4. 价格低，批量采购有优势
```

---

## 二、系统时钟树 —— 为什么是 72MHz？

### 2.1 时钟链路

```
HSE 8MHz (外部晶振)
    │
    ▼
PLL (×9 倍频) → 72MHz (SYSCLK)
    │
    ├── AHB 分频器 /1 → HCLK = 72MHz (CPU + DMA + 总线矩阵)
    │
    ├── APB2 分频器 /1 → PCLK2 = 72MHz (GPIO, TIM1, USART1, SPI1)
    │
    └── APB1 分频器 /2 → PCLK1 = 36MHz (TIM2~7, USART2~5, I2C, SPI2/3)
```

**关键配置代码解析**：

```c
FLASH_SetLatency(FLASH_Latency_2);  // ← 为什么是 2？

// STM32F103 的 Flash 访问速度最大 ~24MHz，而 CPU 跑在 72MHz。
// 如果不插入等待周期，CPU 从 Flash 取指令时数据还没准备好。
// Latency_2 意味着 CPU 每读一次 Flash 等 2 个 HCLK 周期。
// 配合 FLASH_PrefetchBufferCmd(ENABLE) 预取缓冲，
// 大部分指令可以从预取缓冲直接读，实际性能接近 1 等待周期。
```

```c
RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
// 8MHz / 1 * 9 = 72MHz
// 为什么不是 16 倍频（128MHz）？—— STM32F103 超频不可靠，
// 72MHz 是官方保证的稳定频率
```

**面试追问：HSE 起振失败怎么办？**

```
我的代码中 HSE 失败后进了 while(1) 死循环，这是最基础的方案。

更好的做法：切换为 HSI（内部 8MHz RC 振荡器）作为备用时钟，
至少让系统能跑起来并通过串口报错。

HSI 的缺点：精度只有 ±1%，做 UART 通信可能误码率偏高，
因为波特率时钟不准。HSE 晶振精度是 ±20ppm，差了 50000 倍。
```

---

## 三、FreeRTOS 任务设计 —— 为什么这样分配？

### 3.1 四任务架构

```
优先级高
    │
    ├── [PRIO 3] TempHumiTask    采集线程 (500ms 周期)
    │      │  传感器数据队列 (sensor_data_t, 深度 8)
    │      ▼
    ├── [PRIO 2] PWMControlTask  控制线程 (200ms 周期)
    │      ▲   控制指令队列 (control_cmd_t, 深度 4)
    │      │
    ├── [PRIO 2] CommTask        通信线程 (200ms 周期)
    │      │
    ▼
    [PRIO 1] MonitorTask         监控线程 (5s 周期)
优先级低
```

### 3.2 为什么采集任务优先级最高？

```
控制论基本功：传感器数据是决策的基础。如果采集延迟了，
后面的计算和控制都是基于过期数据。

假设：温度实际已升到 35°C（危险），但采集任务被低优先级的
日志打印阻塞了 2 秒。2 秒后才发现高温，此时作物已经受损。

所以 TempHumiTask 必须最高优先级，且周期 500ms 确保了
数据采集延迟 < 1s。
```

### 3.3 为什么用消息队列而不是全局变量？

```c
// ❌ 错误做法：全局变量
float g_temperature;  // 采集任务写，控制任务读
// 问题：多线程同时读写，没有保护，可能读到"半成品"数据

// ✅ 正确做法：消息队列
QueueHandle_t g_sensor_data_queue;  // 深度 8
// xQueueSend() 和 xQueueReceive() 内部有临界区保护
// 而且队列满时采集任务会阻塞等待（或超时丢弃），不会丢最新数据
```

### 3.4 栈大小是怎么定的？

```c
#define STACK_SIZE_TEMP_HUMI    256  // words = 1024 bytes
#define STACK_SIZE_PWM_CTRL     128  // words = 512 bytes
#define STACK_SIZE_COMM         256
#define STACK_SIZE_MONITOR      128
```

**如何验证栈大小是否足够？**

```
在 MonitorTask 中调用了：
    UBaseType_t high_water = uxTaskGetStackHighWaterMark(task);

这个函数返回任务栈中从未被使用过的最小剩余量。如果返回值 < 32 words，
MonitorTask 会打印 "LOW STACK" 警告。

实际做法：先给一个较大的栈（如 512 words），跑一段时间，
看 high_water_mark 稳定在多少（比如 380），然后调为 high_water + 64。

这样既不会溢出，也不会浪费 RAM。
```

---

## 四、SHT30 驱动 —— I2C 通信详解

### 4.1 I2C 总线协议

```
I2C 只用两根线：SCL（时钟）+ SDA（数据），可以挂 127 个设备。

通信过程：
Master                           Slave(SHT30)
  │   START 条件（SCL高时SDA↓）      │
  │   发送 7-bit 地址 + R/W         │
  │                                 ├─ 返回 ACK（SDA↓）
  │   发送命令高字节 + 等待 ACK       │
  │   发送命令低字节 + 等待 ACK       │
  │   STOP 条件（SCL高时SDA↑）       │
  │                                 │
  │   START + 地址(读)              │
  │   读 6 字节（T_MSB,T_LSB,T_CRC, H_MSB,H_LSB,H_CRC）
  │   最后字节回复 NACK              │
  │   STOP                          │
```

### 4.2 SHT30 CRC-8 校验

```c
uint8_t sht30_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;  // 多项式 0x31
            else
                crc <<= 1;
        }
    }
    return crc;
}
```

**为什么需要 CRC？I2C 在 PCB 上只有几厘米，怎么会出错？**

```
面试要点：
1. 车间/温室环境有强电磁干扰（大功率电机启停产生电磁脉冲）
2. SHT30 传感器可能通过长排线连接（有时几米），不是焊接在 PCB 上
3. 数据手册显示了 CRC 字段——不校验等于浪费了芯片工程师给你提供的保护

实际遇到过的情况：I2C 线路靠近 380V 变频器，无屏蔽，T_CRC 校验
失败率 ~0.1%。不校验的话，偶尔会出现 "温度 255°C" 的荒诞读数。
```

### 4.3 数据转换公式

```c
// SHT30 数据手册 Section 4.13
*temperature = -45.0f + 175.0f * (float)raw_temp / 65535.0f;
*humidity    = 100.0f * (float)raw_humi / 65535.0f;

// 面试追问：为什么不直接用 (raw_temp * 175 / 65535)？
// 因为 raw_temp 和 65535 都是整数，整数除法会截断小数部分。
// 先转 float 再除，保留精度。
```

---

## 五、PWM 驱动 —— 定时器配置

### 5.1 TIM2 时基计算

```c
// 目标：PWM 频率 = 1kHz
// 公式：PWM_Freq = TIM_CLK / ((PSC+1) * (ARR+1))
// TIM2 挂 APB1，APB1 时钟 = 36MHz，由于 APB1 预分频 ≠ 1，
// TIM2 时钟 = 36MHz × 2 = 72MHz
//
// 72,000,000 / ((71+1) * (999+1)) = 72,000,000 / 72,000 = 1,000 Hz ✓

tim.TIM_Prescaler = 71;   // PSC
tim.TIM_Period    = 999;  // ARR
```

**为什么选 1kHz？**

```
1. 风扇是机械部件，PWM 频率太低会听到"嗡嗡"声（人耳可听范围 20~20kHz 内）
   → 1kHz 的 PWM 频率会让风扇线圈发出可闻噪声
   → 更好的选择是 >20kHz（超声波，人耳听不到）
   → 但频率越高，TIM2 的分辨率越低（1kHz 时占空比分辨率 = 1000 步）

2. 这是一个权衡：
   - 1kHz：分辨率 1000 步（每步 0.1%），但可能有点噪音
   - 25kHz：无噪音，但分辨率只有 40 步（每步 2.5%）
   
   我选了 1kHz 因为：温室环境本来就有风扇噪音，不在乎多这一点。
   如果是对噪音敏感的场景（如室内智能家居），选 25kHz。
```

### 5.2 占空比计算

```c
void pwm_fan_set_duty(uint8_t duty)  // duty: 0~100
{
    uint16_t pulse = (uint16_t)((uint32_t)duty * (PWM_PERIOD + 1) / 100);
    // duty=50 → pulse = 50 * 1000 / 100 = 500 → 50% 占空比
    TIM_SetCompare2(TIM2, pulse);
}
```

**为什么先转到 uint32_t？**

```c
// 如果不用 uint32_t 中间转换：
uint16_t pulse = duty * 1000 / 100;
// duty=100 时：100 * 1000 = 100000，溢出 uint16_t(65535)！
// 结果：100000 % 65536 = 34464，等于 34.5% 占空比，完全错误。

// 用 (uint32_t)duty：在 32 位空间做乘法，不会溢出。
```

---

## 六、PI 闭环控制算法 —— 逐行分析

### 6.1 为什么是 PI 而不是 PID 或简单阈值？

```
简单阈值（如 "温度>30°C → 风扇开"）的问题：
- 温度在 29.9°C 和 30.1°C 之间来回跳 → 风扇频繁启停 → 继电器寿命耗尽
- 没有过渡——风扇要么全速要么停止

PID 的问题：
- D（微分）项对测量噪声极度敏感
- 温湿度变化本来就很慢（惯性大），D 项贡献不大
- 省掉 D 项 → 少一个需要调参的系数，工业现场调参时间宝贵

PI 控制的效果：
- P（比例）：温度偏离越多，风扇转得越快——平滑调节
- I（积分）：温度一直偏高 0.5°C，积分项累积后会逐渐加大输出
  → 消除稳态误差
```

### 6.2 死区（Deadband）

```c
if (fabsf(temp_error) < g_params.deadband_temp) {  // 0.5°C
    temp_error = 0.0f;
    g_temp_integral *= 0.5f;  // 积分项逐渐衰减
}
```

**死区解决什么问题？**

```
没有死区时：
温度 25.01°C → error=0.01 → 风扇开 1% → 温度 24.99°C → error=-0.01
→ 风扇关 → 温度 25.01°C → ...

结果：风扇以高频开关，发出"咔嗒咔嗒"声，MOSFET 驱动管发热。

有死区（±0.5°C）后：
温度在 24.5~25.5°C 范围内不动作，风扇保持低风速维持空气流通。
```

### 6.3 积分饱和（Integral Windup）处理

```c
// 温度控制
g_temp_integral = clamp(g_temp_integral, 0.0f, 50.0f);  // 上限
g_temp_integral = clamp(g_temp_integral, -50.0f, 0.0f);  // 下限
```

**积分饱和是什么意思？**

```
场景：早上 6 点，温室 10°C，目标 25°C。

没有限幅：error=15 持续了 30 分钟，积分项累积到 15×1800×0.2 = 5400。
风扇输出 = 5×15 + 0.5×5400 = 2775% → 截断到 100%。

然后太阳出来了，温度升到 25°C。但积分项还是 5400——需要等 error 变成负数，
花同样 30 分钟才能"降回来"。这期间风扇一直全速，浪费电。

有限幅（±50）：积分项被限制在 -50~50。当温度恢复正常时，
积分项最多只会导致一段短时间的过度调整。
```

### 6.4 模式切换时的积分重置

```c
void control_reset_integrator(void)
{
    g_temp_integral = 0.0f;
    g_humi_integral = 0.0f;
}

// 在切换模式时调用：
// CTRL_AUTO_MODE → control_reset_integrator()
// CTRL_MANUAL_MODE → 手动调完切回 AUTO 时也要 reset
```

```
不重置的后果：手动模式下风扇开到 100%，积分项一直在累积。
切回自动模式瞬间，输出 = P项 + 巨大的积分项 → 行为不可预测。
```

---

## 七、UART + DMA —— 为什么不用中断方式？

### 7.1 中断 vs DMA

```
中断方式（USART_ITConfig）：
PC 端发来 "STATUS\r\n"（8 字节）：
  每个字节触发一次 USART1_IRQHandler
  → 8 次中断
  → 每次中断：保存/恢复上下文 + 读 DR 寄存器 + 存入缓冲区
  → 总共约 8 × 12 个 CPU 周期 = 96 周期 ← 看起来不多

但如果每 200ms 就要处理 500 字节的数据流（JSON 格式状态上报）：
  500 次中断 × 12 周期 = 6000 周期，占 CPU 时间 6000/72000000 = 0.008%
  ← 好像也不多？

但问题是：这 500 次中断随时可能发生在任何任务的执行过程中，
打断正在运行的 PI 控制计算。如果中断在内核临界区中到来，
会被延迟处理，累计延迟影响实时性。

DMA 方式：DMA 控制器在后台自动搬运数据，只在传输完成时
触发 1 次中断。CPU 可以专注于控制算法。
```

### 7.2 DMA 环形缓冲区

```c
// DMA 配置为 Circular 模式
dma.DMA_Mode = DMA_Mode_Circular;

// RX 缓冲区是循环的：
// DMA 写指针 → 自动递增，到末尾自动回到开头
// CPU 读指针 → 手动管理，追上 DMA 写指针
//
// [D D D * * * * * W W W W]
//        R           W
//  已读区域   空白    新数据

uint16_t uart_dma_rx_available(void)
{
    // DMA_CNDTR 从 BUF_SIZE 递减到 0（表示缓冲区写满了一圈）
    // 实际写入位置 = BUF_SIZE - CNDTR
    uint16_t write_idx = UART_RX_BUF_SIZE - DMA1_Channel5->CNDTR;
    // 环形差值计算
    if (write_idx >= g_rx_read_idx)
        return write_idx - g_rx_read_idx;
    else
        return UART_RX_BUF_SIZE - g_rx_read_idx + write_idx;
}
```

---

## 八、看门狗 + 任务监控 —— 可靠性设计

### 8.1 独立看门狗 (IWDG)

```c
IWDG_SetPrescaler(IWDG_Prescaler_64);  // 40kHz / 64 = 625Hz
IWDG_SetReload(3125);                  // 3125 / 625 = 5 秒超时
```

**为什么是 5 秒？**

```
系统最慢的任务周期是 MonitorTask（5 秒）。
如果任务卡死，最坏情况是 5 秒后才被看门狗发现，然后 MCU 复位。

如果设 2 秒：MonitorTask 来不及喂狗就复位了。
如果设 10 秒：系统真的挂了要 10 秒才恢复，温室可能已经过热了。

5 秒是一个合理的中间值。
```

### 8.2 任务栈水位监测

```c
check_task_stack(g_temp_humi_task_handle, "TempHumi", 32);
// 如果 uxTaskGetStackHighWaterMark < 32 words → 马上告警
```

**为什么发现栈快溢出了不直接复位？**

```
1. 打印告警日志（串口输出），方便调试
2. 把日志发出去后，再由看门狗复位
3. 如果下次启动还是 LOW STACK → 说明栈设置太小，需要调大

如果直接复位就没有机会看到诊断信息了。
```

---

## 九、面试模拟

### Q1: "为什么用 FreeRTOS 而不是裸机 super loop？"

```
A: 因为四个功能对实时性的要求不同：
- 传感器采集必须准时（500ms），裸机需要手动管理时钟
- 通信任务可能在等待 UART 发送完成时阻塞，裸机下会拖慢采集
- FreeRTOS 的抢占式调度让高优先级任务能打断低优先级

裸机 super loop 适合功能单一、时序简单的场景（如只测温度+LED显示）。
本系统有采集+控制+通信+监控四个并发任务，RTOS 模块化好得多。
```

### Q2: "PI 控制的三个参数是怎么调的？"

```
A: 工程上调参的顺序通常是：
1. 先调 P：从 0 开始，逐步增大，直到系统出现轻微振荡，记录此时 Kp 为 Ku
2. Kp = 0.6 * Ku（Ziegler-Nichols 法经验值）
3. 再调 I：从 0 开始，逐步增大，直到稳态误差在可接受范围内
4. 最终 temp_kp=5.0, temp_ki=0.5 是这个项目调出来的值

死区是根据传感器精度（SHT30 ±0.3°C）加上工程余量定的 ±0.5°C。
```

### Q3: "串口通信为什么不加协议帧（像项目1那样）？"

```
A: 设计决策：UART 通信距离短（< 3 米），面向调试，数据丢包概率低。
JSON 格式自带边界（{...}），用换行 (\r\n) 做帧边界足够。

如果面向产品化，产线环境干扰大，会考虑加 MODBUS-RTU 协议：
帧格式 [地址][功能码][数据][CRC16]，比我的项目1协议更标准。
```

### Q4: "I2C 通信中如果 SHT30 不响应怎么办？"

```
A: 代码中多处有 timeout 保护：
while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    if (--timeout < 0) return -2;  // 超时返回，不会死等
}

TempHumiTask 中有连续错误计数：
if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
    // 传感器离线告警
    sensor_data.valid = 0;
}
控制任务收到 valid=0 的数据后会切换到告警模式（风扇全开，保证安全）。
```
