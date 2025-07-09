from PyQt5 import QtCore
import time
import serial

class TorqueSensorThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(float, float)  # (timestamp, torque)
    status_changed = QtCore.pyqtSignal(bool)
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.port = None
        self.baudrate = 115200
        self.freq = 1000
        self.running = False
        self.ser = None
        self._stop_event = False
        self._sampling = False  # 添加采样控制标志
        self.lock = QtCore.QMutex()
        self.expected_len = 8  # 期望的最小数据长度
        self.rx_buffer = b''
        
        # 创建发送定时器，但不要立即moveToThread，而是在run方法中创建
        self.send_timer = None
        self.last_send_time = 0
        self.send_interval = 0.001  # 默认1000Hz
        
        # 发送频率监控
        self.send_count = 0
        self.last_send_count_time = 0
        self.send_freq_monitor = True
        
        # 性能优化参数
        self.ui_update_interval = 0.1  # 每100ms更新一次UI
        self.last_ui_update_time = 0
        self.batch_size = 1  # 默认每次发送1个命令
        self.data_buffer = []  # 数据缓冲区，用于批量处理

    def configure(self, port, baudrate, freq):
        self.port = port
        self.baudrate = baudrate
        self.freq = freq
        self.send_interval = 1.0 / freq if freq > 0 else 0.001

    def run(self):
        try:
            print(f"扭矩传感器: 尝试打开串口 {self.port}，波特率 {self.baudrate}")
            # 添加更多串口配置参数
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.5
            )
            print(f"扭矩传感器: 串口打开成功，配置: {self.ser}")
            
            if not self.ser.is_open:
                self.status_changed.emit(False)
                self.error_signal.emit(f"Failed to open torque sensor port {self.port}")
                return
                
            self.rx_buffer = b''
            self.running = True
            self.status_changed.emit(True)
            
            # 使用更可靠的时间控制方法
            self.last_send_time = time.perf_counter()
            self.last_send_count_time = time.perf_counter()
            self.last_ui_update_time = time.perf_counter()
            self.send_count = 0
            self.data_buffer = []
            
            # 预先准备好发送数据，避免重复创建
            self.awake_cmd = bytes([0x01, 0x03, 0x00, 0x1E, 0x00, 0x02, 0xA4, 0x0D])
            
            # 根据发送频率调整批处理大小
            if self.freq > 100:  # 如果频率高于100Hz
                self.batch_size = max(1, int(self.freq / 50))  # 每批最多发送频率/50个命令
                print(f"扭矩传感器: 高频率模式({self.freq}Hz)，批处理大小设为 {self.batch_size}")
            
            # 接收循环
            while not self._stop_event:
                try:
                    # 定时发送
                    current_time = time.perf_counter()
                    if self._sampling and current_time - self.last_send_time >= self.send_interval:
                        # 高频率发送时，可能需要多次发送以赶上时间
                        send_count_this_cycle = 0
                        while self._sampling and current_time - self.last_send_time >= self.send_interval and send_count_this_cycle < self.batch_size:
                            self.send_command()
                            self.last_send_time += self.send_interval  # 使用固定间隔，避免时间漂移
                            self.send_count += 1
                            send_count_this_cycle += 1
                        
                        # 监控发送频率
                        if self.send_freq_monitor and current_time - self.last_send_count_time >= 1.0:
                            send_freq = self.send_count / (current_time - self.last_send_count_time)
                            print(f"扭矩传感器: 实际发送频率 {send_freq:.2f} Hz，目标频率 {1.0/self.send_interval:.2f} Hz")
                            self.send_count = 0
                            self.last_send_count_time = current_time
                    
                    # 处理接收
                    if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                        self.process_incoming_data()
                        
                    # 发送缓冲的数据到UI
                    if self.data_buffer and current_time - self.last_ui_update_time >= self.ui_update_interval:
                        # 只发送最新的数据点
                        if len(self.data_buffer) > 1:
                            timestamp, torque = self.data_buffer[-1]
                            self.data_received.emit(timestamp, torque)
                        self.data_buffer = []
                        self.last_ui_update_time = current_time
                        
                    # 根据发送频率动态调整休眠时间，降低CPU占用
                    sleep_time = min(1, max(0.5 * self.send_interval, 0.001))
                    QtCore.QThread.msleep(int(sleep_time * 1000))
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    # 接收错误不中断线程，只记录
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.error_signal.emit(str(e))
        finally:
            if self.send_timer:
                self.send_timer.stop()
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.running = False
            self.status_changed.emit(False)

    def send_command(self):
        """发送命令函数"""
        if not self._sampling or not self.running or not self.ser or not self.ser.is_open:
            return
        try:
            # 使用预先准备好的命令
            self.ser.write(self.awake_cmd)
        except Exception as e:
            import traceback
            print(f"Torque sensor send_command异常: {e}")
            traceback.print_exc()
            # 发送错误不中断线程，只记录

    def process_incoming_data(self):
        """处理接收到的数据"""
        if not self.ser or not self.ser.is_open:
            return
            
        buffer = self.ser.read(self.ser.in_waiting)
        if not buffer:
            return
            
        # 将新数据追加到接收缓冲区
        self.rx_buffer += buffer
        
        # 查找并处理完整帧
        if len(self.rx_buffer) >= self.expected_len:
            torque = 0.0
            # 查找帧头并解析数据
            for i in range(len(self.rx_buffer) - 6):
                if self.rx_buffer[i] == 0x01 and self.rx_buffer[i + 1] == 0x03:
                    U_DATA = self.rx_buffer[i + 6]
                    U_DATA |= self.rx_buffer[i + 5] << 8
                    torque = self.hex2dec(U_DATA) / 30.0
                    # 添加到数据缓冲区，而不是直接发送
                    timestamp = time.perf_counter()
                    self.data_buffer.append((timestamp, torque))
                    break
            
            # 清空接收缓冲区，准备下一次接收
            self.rx_buffer = b''

    def stop(self):
        self._stop_event = True
        if self.send_timer:
            self.send_timer.stop()
        self.wait()

    def update_params(self, port=None, baudrate=None, freq=None):
        with QtCore.QMutexLocker(self.lock):
            if port is not None:
                self.port = port
            if baudrate is not None:
                self.baudrate = baudrate
            if freq is not None:
                self.freq = freq
                self.send_interval = 1.0 / freq if freq > 0 else 0.001
                
    def enable_sampling(self, enable):
        """启用或禁用采样"""
        print(f"扭矩传感器: {'启用' if enable else '禁用'}采样，当前状态: _sampling={self._sampling}, running={self.running}")
        with QtCore.QMutexLocker(self.lock):
            self._sampling = enable
            
        # 如果启用采样，重置发送时间
        if enable and self.running:
            self.last_send_time = time.perf_counter() - self.send_interval  # 确保立即发送一次
    
    @staticmethod
    def hex2dec(hex_data):
        return int.from_bytes(hex_data.to_bytes(2, 'big', signed=False), 'big', signed=True) 