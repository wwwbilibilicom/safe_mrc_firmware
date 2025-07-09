"""
扭矩传感器通信线程模块，提供与扭矩传感器的异步通信功能
"""
from PyQt5 import QtCore
import time
import serial
from torque_sensor import TorqueSensor
from error_handler import ErrorHandler, ErrorSeverity, logger
from config import (
    TORQUE_COMMAND, 
    TORQUE_RESPONSE_MIN_LENGTH, 
    TORQUE_CONVERSION_FACTOR,
    RX_POLL_INTERVAL
)

class TorqueSensorThread(QtCore.QThread):
    """
    扭矩传感器通信线程类
    
    使用Qt的事件驱动模型实现与扭矩传感器的异步通信，
    避免阻塞主线程，提高UI响应性能
    """
    # 信号定义
    data_received = QtCore.pyqtSignal(float, float)  # (timestamp, torque)
    status_changed = QtCore.pyqtSignal(bool)
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        """初始化扭矩传感器线程"""
        super().__init__()
        
        # 通信参数
        self.port = None
        self.baudrate = 115200
        self.freq = 1000
        self.running = False
        self.ser = None
        self._stop_event = False
        
        # 线程同步
        self.lock = QtCore.QMutex()
        
        # 数据处理
        self.expected_len = TORQUE_RESPONSE_MIN_LENGTH
        self._buffer = b''
        
        # 定时器设置
        self._timer = QtCore.QTimer()
        self._timer.moveToThread(self)
        self._timer.timeout.connect(self._send_command)
        
        # 接收定时器
        self._rx_timer = QtCore.QTimer()
        self._rx_timer.moveToThread(self)
        self._rx_timer.timeout.connect(self._poll_read)

    def configure(self, port, baudrate, freq):
        """
        配置扭矩传感器通信参数
        
        Args:
            port: 串口名称
            baudrate: 波特率
            freq: 采样频率 (Hz)
        """
        with QtCore.QMutexLocker(self.lock):
            self.port = port
            self.baudrate = baudrate
            self.freq = freq
            logger.info(f"扭矩传感器配置更新: 端口={port}, 波特率={baudrate}, 频率={freq}Hz")

    def run(self):
        """线程主函数，启动事件循环和定时器"""
        try:
            # 打开串口
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            if not self.ser.is_open:
                self.status_changed.emit(False)
                self.error_signal.emit(f"无法打开扭矩传感器端口 {self.port}")
                return
                
            logger.info(f"扭矩传感器串口已打开: {self.port}")
            self.running = True
            self.status_changed.emit(True)
            
            # 计算定时器间隔
            interval = max(1, int(1000.0 / self.freq))
            logger.info(f"扭矩传感器采样间隔: {interval}ms ({self.freq}Hz)")
            
            # 启动发送定时器
            self._timer.start(interval)
            
            # 启动接收定时器
            self._rx_timer.start(RX_POLL_INTERVAL)
            
            # 进入事件循环
            self.exec_()
            
        except Exception as e:
            ErrorHandler.log_exception(e, "扭矩传感器线程启动失败")
            self.error_signal.emit(f"扭矩传感器错误: {str(e)}")
        finally:
            # 清理资源
            self._cleanup()

    def _cleanup(self):
        """清理资源并通知状态变化"""
        # 停止定时器
        if hasattr(self, '_timer') and self._timer.isActive():
            self._timer.stop()
            
        if hasattr(self, '_rx_timer') and self._rx_timer.isActive():
            self._rx_timer.stop()
            
        # 关闭串口
        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
                    logger.info("扭矩传感器串口已关闭")
            except Exception as e:
                ErrorHandler.log_exception(e, "关闭扭矩传感器串口时出错", ErrorSeverity.WARNING)
                
        # 更新状态
        self.running = False
        self.status_changed.emit(False)

    def _send_command(self):
        """定时器触发的发送命令函数"""
        if not self.ser or not self.ser.is_open:
            return
            
        try:
            # 清空接收缓冲区
            self.ser.reset_input_buffer()
            # 发送读取命令
            self.ser.write(TORQUE_COMMAND)
        except Exception as e:
            ErrorHandler.log_exception(e, "发送扭矩传感器命令失败", ErrorSeverity.WARNING)

    def _poll_read(self):
        """定时器触发的数据接收函数"""
        if not self.ser or not self.ser.is_open:
            return
            
        try:
            if self.ser.in_waiting:
                new_data = self.ser.read(self.ser.in_waiting)
                if new_data:
                    self._buffer += new_data
                    self._process_buffer()
        except Exception as e:
            ErrorHandler.log_exception(e, "读取扭矩传感器数据失败", ErrorSeverity.WARNING)

    def _process_buffer(self):
        """处理接收缓冲区数据"""
        if len(self._buffer) < self.expected_len:
            return
            
        # 查找帧头并解析数据
        for i in range(len(self._buffer) - 6):
            if self._buffer[i] == 0x01 and self._buffer[i + 1] == 0x03:
                # 提取数据字段
                data_high = self._buffer[i + 5]
                data_low = self._buffer[i + 6]
                u_data = (data_high << 8) | data_low
                
                # 转换为扭矩值
                torque = TorqueSensor.hex2dec(u_data) / TORQUE_CONVERSION_FACTOR
                timestamp = time.perf_counter()
                
                # 发送数据信号
                self.data_received.emit(timestamp, torque)
                
                # 清空缓冲区，等待下一次数据
                self._buffer = b''
                return
                
        # 如果没找到有效帧头，保留最后几个字节，其余清空
        if len(self._buffer) > 20:
            self._buffer = self._buffer[-10:]

    def stop(self):
        """安全停止线程"""
        logger.info("正在停止扭矩传感器线程...")
        self._stop_event = True
        QtCore.QMetaObject.invokeMethod(self, 'quit')
        
        # 等待线程结束，最多等待1秒
        if not self.wait(1000):
            logger.warning("扭矩传感器线程未能在1秒内停止")
        else:
            logger.info("扭矩传感器线程已停止")

    def update_params(self, port=None, baudrate=None, freq=None):
        """
        更新通信参数
        
        Args:
            port: 串口名称 (可选)
            baudrate: 波特率 (可选)
            freq: 采样频率 (Hz) (可选)
        """
        with QtCore.QMutexLocker(self.lock):
            if port is not None:
                self.port = port
                
            if baudrate is not None:
                self.baudrate = baudrate
                
            if freq is not None:
                self.freq = freq
                # 更新定时器间隔
                if self._timer.isActive():
                    interval = max(1, int(1000.0 / self.freq))
                    self._timer.setInterval(interval)
                    logger.info(f"扭矩传感器采样频率已更新: {self.freq}Hz (间隔: {interval}ms)") 