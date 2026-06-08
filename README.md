# STM32嵌入式环境监测与控制系统

[![MCU](https://img.shields.io/badge/MCU-STM32F103RBT6-blue.svg)]()
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green.svg)]()
[![Framework](https://img.shields.io/badge/Library-Standard_Peripheral-red.svg)]()

面向智慧农业温室环境监控场景的**低成本、低功耗**环境监测终端。基于 STM32F103RBT6 + FreeRTOS，实现温湿度实时监控与自动化调节，系统数据采集延迟 **< 1s**。

---

## 🎯 项目背景

针对智慧农业温室环境监控滞后、人工调控效率低的问题，研发低成本、低功耗环境监测终端，实现温湿度及设备状态实时监控与自动化调节。

## 🏗 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                   FreeRTOS Scheduler                       │
│                                                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │
│  │ 温湿度采集   │  │ PWM控制      │  │ 通信任务     │       │
│  │ Task        │  │ Task        │  │ Task        │       │
│  │ Priority: 2 │  │ Priority: 1 │  │ Priority: 1 │       │
│  ├─────────────┤  ├─────────────┤  ├─────────────┤       │
│  │·SHT30 I2C   │  │·GPIO 补光灯 │  │·UART+DMA    │       │
│  │·DMA 采集    │  │·TIM PWM 风扇│  │·数据上报     │       │
│  │·阈值检测    │  │·闭环联动    │  │·指令接收     │       │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘       │
│         │                │                │               │
│  ┌──────┴────────────────┴────────────────┴──────┐       │
│  │              Hardware Abstraction              │       │
│  │  SHT30 | GPIO | TIM | PWM | UART | DMA | I2C │       │
│  └───────────────────────────────────────────────┘       │
└──────────────────────────────────────────────────────────┘
```

## 📦 目录结构

```
stm32-env-monitor/
├── README.md
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── sht30.h              # SHT30 温湿度传感器驱动
│   │   ├── gpio_pwm.h           # GPIO/PWM 外设驱动
│   │   ├── uart_dma.h           # UART DMA 通信驱动
│   │   ├── temp_humi_task.h     # 温湿度采集任务
│   │   ├── pwm_control_task.h   # PWM 控制任务
│   │   ├── comm_task.h          # 通信任务
│   │   ├── control_algorithm.h  # 闭环控制算法
│   │   └── task_monitor.h       # 任务监控
│   └── Src/
│       ├── main.c               # 主入口 + 系统初始化
│       ├── tasks/
│       │   ├── temp_humi_task.c  # 温湿度采集任务
│       │   ├── pwm_control_task.c# PWM控制任务
│       │   └── comm_task.c       # 通信任务
│       └── drivers/
│           ├── sht30.c           # SHT30 I2C 驱动
│           ├── gpio_pwm.c        # GPIO & PWM 驱动
│           └── uart_dma.c        # UART DMA 驱动
├── Drivers/
│   └── FreeRTOSConfig.h         # FreeRTOS 配置
└── docs/
    └── system_diagram.md        # 系统设计文档
```

## 🔧 硬件配置

| 外设 | 引脚 | 功能 |
|------|------|------|
| I2C1 | PB6(SCL) / PB7(SDA) | SHT30 温湿度传感器 |
| TIM2_CH1 | PA0 | PWM 补光灯控制 |
| TIM2_CH2 | PA1 | PWM 通风扇控制 |
| GPIO | PC13 | 补光灯开关（紧急） |
| GPIO | PA4 | 按键输入（手动模式） |
| USART1 | PA9(TX) / PA10(RX) | 调试串口 (DMA) |
| USART2 | PA2(TX) / PA3(RX) | 上位机通信 |

## 🧵 FreeRTOS 任务设计

| 任务 | 优先级 | 栈大小 | 周期 | 功能 |
|------|--------|--------|------|------|
| TempHumiTask | 2 (高) | 256 words | 500ms | SHT30 读取 + 阈值检测 |
| PWMControlTask | 1 (中) | 128 words | 200ms | PID/阈值联动控制 |
| CommTask | 1 (中) | 256 words | 事件驱动 | UART+DMA 收发 |
| MonitorTask | 0 (低) | 128 words | 2s | 任务监控 + 看门狗喂狗 |

## 🚀 快速开始

### 环境要求

- **IDE**: Keil MDK v5.30+
- **MCU**: STM32F103RBT6
- **调试器**: ST-Link V2
- **FreeRTOS**: v10.4.3 (已包含配置)

### 编译烧录

1. 用 Keil MDK 打开项目
2. 选择目标 `STM32F103RBT6`
3. `Project → Build Target` (F7)
4. `Flash → Download` (F8)

### 串口调试

```bash
# 波特率 115200, 8N1
screen /dev/ttyUSB0 115200
# 或
minicom -D /dev/ttyUSB0 -b 115200
```

## 🧪 核心测试场景

| 场景 | 触发条件 | 预期行为 |
|------|----------|----------|
| 温度过高 | > 30°C | 通风扇启动 (PWM 70%+) |
| 湿度过低 | < 40% | 补光灯开启 |
| 正常范围 | 22-28°C, 50-70% | 风扇/灯关闭（节能） |
| 通信异常 | UART 连续 3 次无ACK | 切换为本地自动模式 |
| 任务卡死 | 看门狗 5s 未喂狗 | MCU 自动复位 |
| 数据采集延迟 | - | < 1s (实测 ~800ms) |

## 🛠 关键技术点

1. **FreeRTOS 并发调度**：3 个核心任务独立运行，通过消息队列通信
2. **DMA + UART**：DMA 传输降低 CPU 占用，确保多任务流畅执行
3. **定时器中断替代阻塞 Delay**：提高系统响应速度
4. **闭环阈值联动控制**：支持无人值守智能调节
5. **任务监控 + Watchdog**：异常任务自动检测与恢复
6. **串口日志调试**：配合 FreeRTOS 任务信息快速定位问题

## 📄 License

MIT License - 仅供学习与展示使用
