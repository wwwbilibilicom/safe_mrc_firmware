# SafeMRC 上位机应用程序

## 项目简介

SafeMRC 上位机应用程序是一个基于 PyQt5 的图形界面应用，用于与 SafeMRC 设备和扭矩传感器进行通信。应用程序提供了实时数据显示、曲线绘制和数据记录功能。

## 功能特点

- **SafeMRC 设备通信**：
  - 串口连接/断开
  - 参数配置（设备ID、工作模式、电流值等）
  - 实时数据显示（编码器角度、速度、电流等）
  - 十六进制通信帧显示

- **扭矩传感器通信**：
  - 串口连接/断开
  - 参数配置（波特率、采样频率等）
  - 实时扭矩数据显示

- **数据可视化**：
  - 实时曲线绘制
  - 可调节时间窗口
  - 多通道数据显示

- **数据记录**：
  - CSV 格式数据保存
  - 支持同时记录 SafeMRC 和扭矩传感器数据
  - 高精度时间戳

## 系统架构

项目采用模块化设计，主要包含以下模块：

- **主界面模块** (`main.py`/`main_refactored.py`)：
  - 提供用户界面
  - 协调各模块工作

- **SafeMRC 通信模块** (`safemrc_comm.py`)：
  - 实现与 SafeMRC 设备的通信协议
  - 提供命令打包和反馈解析功能

- **扭矩传感器模块** (`torque_sensor.py`, `torque_sensor_thread.py`)：
  - 实现与扭矩传感器的通信
  - 提供数据采集和处理功能

- **UI 组件模块** (`ui_components.py`)：
  - 提供可重用的 UI 组件
  - 封装常用的界面元素

- **数据管理模块** (`data_manager.py`)：
  - 提供数据处理和管理功能
  - 实现环形缓冲区和数据记录器

- **错误处理模块** (`error_handler.py`)：
  - 提供统一的异常处理机制
  - 实现日志记录和错误提示

- **配置模块** (`config.py`)：
  - 存储全局配置和常量
  - 提供集中的参数管理

## 通信协议

### SafeMRC 通信协议

- **命令帧格式**：
  ```
  0xFE 0xEE id(1) mode(1) current(int32, 4 bytes) CRC(2)
  ```

- **反馈帧格式**：
  ```
  0xFE 0xEE id(1) mode(1) collision(1) encoder(4) velocity(4) current(2) CRC(2)
  ```

### 扭矩传感器通信协议

- **命令帧格式**：
  ```
  0x01 0x03 0x00 0x1E 0x00 0x02 0xA4 0x0D
  ```

- **响应帧格式**：
  ```
  0x01 0x03 0x04 data_high data_low crc_low crc_high
  ```

## 安装与运行

### 环境要求

- Python 3.6+
- PyQt5
- pyqtgraph
- numpy
- pyserial

### 安装依赖

```bash
# 使用 conda 创建环境
conda env create -f environment.yml

# 或使用 pip 安装依赖
pip install pyqt5 pyqtgraph numpy pyserial
```

### 运行应用

```bash
# 切换到项目目录
cd scripts

# 运行原始版本
python main.py

# 或运行重构版本
python main_refactored.py
```

## 代码优化说明

本项目经过全面重构，主要优化点包括：

1. **模块化设计**：
   - 将原单一文件拆分为多个功能模块
   - 实现关注点分离，提高代码可维护性

2. **错误处理机制**：
   - 添加统一的错误处理框架
   - 实现日志记录，便于调试和问题追踪

3. **资源管理**：
   - 使用上下文管理器确保资源正确释放
   - 改进线程管理，避免资源泄漏

4. **性能优化**：
   - 使用环形缓冲区优化内存使用
   - 改进绘图算法，减少CPU占用

5. **代码质量**：
   - 添加详细注释和文档字符串
   - 统一命名规范，提高代码可读性
   - 移除硬编码常量，使用配置文件集中管理

6. **用户体验**：
   - 改进错误提示，使用户更容易理解问题
   - 优化UI布局，提高操作便捷性

## 文件结构

```
scripts/
├── config.py              # 配置文件
├── data_manager.py        # 数据管理模块
├── error_handler.py       # 错误处理模块
├── main.py                # 原始主界面
├── main_refactored.py     # 重构后的主界面
├── safemrc_comm.py        # SafeMRC通信模块
├── torque_sensor.py       # 扭矩传感器基础类
├── torque_sensor_thread.py # 扭矩传感器线程类
├── ui_components.py       # UI组件模块
├── test_torque_sensor.py  # 扭矩传感器测试脚本
├── environment.yml        # 环境配置文件
├── README.md              # 项目说明文档
└── data/                  # 数据存储目录
```

## 贡献者

- 项目开发团队

## 许可证

本项目采用 MIT 许可证 