# SafeMRC 应用程序使用说明

## 安装

### 方法一：使用 conda 环境

1. 安装 [Miniconda](https://docs.conda.io/en/latest/miniconda.html) 或 [Anaconda](https://www.anaconda.com/products/distribution)

2. 创建环境并安装依赖：

```bash
# 克隆仓库
git clone https://github.com/your-username/safemrc-app.git
cd safemrc-app/scripts

# 创建环境
conda env create -f environment.yml

# 激活环境
conda activate safemrc
```

### 方法二：使用 pip 安装

```bash
# 克隆仓库
git clone https://github.com/your-username/safemrc-app.git
cd safemrc-app/scripts

# 安装依赖
pip install -r requirements.txt

# 或者使用 setup.py 安装
pip install -e .
```

## 运行应用程序

### 方法一：直接运行

```bash
# 运行重构版本（推荐）
python run_app.py

# 运行原始版本
python run_app.py --legacy

# 启用调试模式
python run_app.py --debug
```

### 方法二：使用安装的命令

如果使用 `setup.py` 安装了应用程序，可以直接运行：

```bash
safemrc-app
```

## 使用指南

### SafeMRC 设备连接

1. 在顶部控制栏选择 SafeMRC 设备的串口
2. 点击「连接」按钮
3. 连接成功后，可以设置以下参数：
   - MRC ID：设备 ID
   - 模式：工作模式（FREE、FIX_LIMIT、ADAPTATION、DEBUG）
   - 发送频率：命令发送频率（Hz）
   - 电流：目标电流值（A）
4. 点击「开始发送」按钮开始发送命令
5. 点击「停止发送」按钮停止发送命令
6. 点击「断开」按钮断开设备连接

### 扭矩传感器连接

1. 在扭矩传感器控制栏选择扭矩传感器的串口
2. 设置波特率和采样频率
3. 点击「连接扭矩传感器」按钮
4. 连接成功后，点击「开始采样」按钮开始采样
5. 点击「停止采样」按钮停止采样
6. 点击「断开扭矩传感器」按钮断开设备连接

### 数据可视化

1. 在绘图控制栏设置绘图窗口大小（秒）
2. 实时数据将在图表中显示
3. 点击「清空图表」按钮清空图表数据

### 数据记录

1. 点击「开始记录」按钮开始记录数据
2. 点击「停止记录」按钮停止记录数据
3. 在弹出的对话框中选择保存文件的路径
4. 数据将以 CSV 格式保存

## 常见问题

### 无法连接设备

1. 确保设备已正确连接到计算机
2. 确保选择了正确的串口
3. 确保没有其他程序占用该串口
4. 尝试刷新串口列表

### 数据显示异常

1. 检查设备连接是否正常
2. 检查参数设置是否正确
3. 尝试清空图表重新开始
4. 检查设备是否正常工作

### 应用程序崩溃

1. 检查日志文件 `safemrc_app.log` 查看错误信息
2. 尝试使用调试模式运行应用程序：`python run_app.py --debug`
3. 如果问题仍然存在，请联系技术支持

## 技术支持

如有任何问题，请联系：

- 邮箱：safemrc@example.com
- 网站：https://www.example.com/safemrc
- GitHub：https://github.com/your-username/safemrc-app 