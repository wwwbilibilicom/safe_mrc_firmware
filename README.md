# Embeded code for safeMRC used in pHRI

## How to install the program

```bash
git clone https://gitee.com/li-wenbo-946/safe-mrc.git
```

## Project Overview

This project provides embedded software for the control of a Magnetorheological Clutch (MRC), specifically designed for applications in safe physical Human-Robot Interaction (pHRI). The system is implemented on STM32 microcontrollers and aims to ensure both high-performance actuation and safety in collaborative environments where humans and robots interact physically.

### Key Features

- **Magnetorheological Clutch Control:** Implements precise control algorithms for the MRC, enabling real-time adjustment of torque and engagement/disengagement states.
- **Safety Mechanisms:** Includes collision detection and automatic response logic to ensure user safety during physical interaction.
- **Modular Design:** The codebase is organized into modules for device drivers, communication, control algorithms (including PID), and hardware abstraction.
- **Real-Time Communication:** Supports UART-based communication for command and feedback exchange, suitable for integration with higher-level robotic systems.
- **Extensibility:** Designed for easy adaptation to different hardware configurations and research needs in pHRI.

### Application Scenario

The software is intended for use in research and development of safe human-robot collaboration, such as assistive robotics, rehabilitation devices, and collaborative industrial robots. By leveraging the unique properties of magnetorheological clutches, the system can provide both strong actuation and rapid disengagement for safety.

---

## Code Structure

```
Core/         # Main application logic (main.c, initialization, main loop)
Device/       # Device drivers (MRC, encoder, keys, LEDs, H-bridge, etc.)
Common/       # Common utilities (PID controller, filters)
Drivers/      # STM32 HAL and CMSIS drivers
MDK-ARM/      # Keil project files and startup code
```

- **Core/**: Contains the main entry point and system initialization.
- **Device/**: Implements hardware abstraction for the MRC and peripherals.
- **Common/**: Provides reusable modules like PID and filters.
- **Drivers/**: Vendor-provided MCU drivers.
- **MDK-ARM/**: Project files for building and debugging with Keil.

## Main Logic Flow

1. System and peripherals are initialized (GPIO, ADC, TIM, UART, etc.).
2. The MRC device is initialized, including communication and control modules.
3. The main loop handles:
   - Communication with host/controller via UART
   - Key input processing
   - Control loop for MRC (torque, state, safety)
   - Collision detection and safety response
4. Real-time feedback and command exchange with upper-level systems.

## Deployment

### Requirements

- STM32H7 series microcontroller
- Keil MDK-ARM or compatible toolchain
- ST-Link or compatible programmer/debugger

### Build & Flash

1. Open `MDK-ARM/safeMRC.uvprojx` in Keil MDK.
2. Build the project (default target: STM32H750xx).
3. Connect the STM32 board via ST-Link.
4. Flash the firmware to the board.

### Run

- After flashing, the device will start running automatically.
- Use UART to communicate with the device for command and feedback.

## Contributing

Pull requests and issues are welcome. Please follow standard C code style and document your changes.

## License

See the LICENSE file for details.

---

## 中文简介

## 如何安装

```bash
git clone https://gitee.com/li-wenbo-946/safe-mrc.git
```

### 项目概述

本项目为磁流变离合器（MRC）的控制提供嵌入式软件，专为安全物理人机交互（pHRI）应用设计。系统基于STM32微控制器实现，旨在确保人机协作环境下的高性能驱动与安全保障。

### 主要特性

- **磁流变离合器控制：** 实现了对MRC的精确控制算法，支持实时扭矩调节及离合器的吸合/断开。
- **安全机制：** 集成了碰撞检测与自动响应逻辑，保障物理交互过程中的用户安全。
- **模块化设计：** 代码结构清晰，分为设备驱动、通信、控制算法（含PID）及硬件抽象等模块。
- **实时通信：** 支持基于UART的命令与反馈通信，便于与上位机或机器人系统集成。
- **易于扩展：** 便于适配不同硬件配置，满足pHRI领域的科研与开发需求。

### 应用场景

本软件适用于安全人机协作的科研与开发，如助力机器人、康复设备及协作工业机器人。通过磁流变离合器的独特特性，系统可在提供强力驱动的同时，实现快速断开以保障安全。

---

## 代码结构

```
Core/         # 主应用逻辑（main.c，初始化，主循环）
Device/       # 设备驱动（MRC、编码器、按键、LED、H桥等）
Common/       # 通用工具模块（PID控制器、滤波器）
Drivers/      # STM32官方驱动
MDK-ARM/      # Keil工程文件及启动代码
```

- **Core/**：主入口和系统初始化
- **Device/**：MRC及外设的硬件抽象实现
- **Common/**：可复用模块如PID和滤波器
- **Drivers/**：芯片厂商提供的驱动
- **MDK-ARM/**：Keil开发环境的工程文件

## 主要逻辑流程

1. 初始化系统和外设（GPIO、ADC、定时器、串口等）
2. 初始化MRC设备，包括通信和控制模块
3. 主循环处理：
   - 通过UART与主控/上位机通信
   - 按键输入处理
   - MRC控制回路（扭矩、状态、安全）
   - 碰撞检测与安全响应
4. 与上位系统进行实时命令与反馈交互

## 部署说明

### 环境要求

- STM32H7系列微控制器
- Keil MDK-ARM或兼容工具链
- ST-Link或兼容下载器

### 编译与烧录

1. 用Keil MDK打开 `MDK-ARM/safeMRC.uvprojx`
2. 编译工程（默认目标：STM32H750xx）
3. 用ST-Link连接开发板
4. 烧录固件到开发板

### 运行

- 烧录后设备自动运行
- 通过UART与设备进行命令和反馈通信

## 贡献

欢迎提交PR和Issue。请遵循C语言代码风格并注释你的更改。

## 许可证

详见LICENSE文件。


