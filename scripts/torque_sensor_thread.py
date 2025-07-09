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
        self.lock = QtCore.QMutex()
        self.expected_len = 8  # 期望的最小数据长度
        self.rx_buffer = b''
        
        # 创建发送定时器，在run方法中启动
        self.send_timer = QtCore.QTimer()
        self.send_timer.moveToThread(self)
        self.send_timer.timeout.connect(self.send_command)

    def configure(self, port, baudrate, freq):
        self.port = port
        self.baudrate = baudrate
        self.freq = freq

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            if not self.ser.is_open:
                self.status_changed.emit(False)
                self.error_signal.emit(f"Failed to open torque sensor port {self.port}")
                return
                
            self.rx_buffer = b''
            self.running = True
            self.status_changed.emit(True)
            
            # 启动发送定时器，使用毫秒
            interval_ms = int(1000.0 / self.freq) if self.freq > 0 else 1
            self.send_timer.start(interval_ms)
            
            # 接收循环
            while not self._stop_event:
                try:
                    if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                        self.process_incoming_data()
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    # 接收错误不中断线程，只记录
                QtCore.QThread.msleep(1)  # 非阻塞短暂休眠，降低CPU占用
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.error_signal.emit(str(e))
        finally:
            self.send_timer.stop()
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.running = False
            self.status_changed.emit(False)

    def send_command(self):
        """定时器触发的发送命令函数"""
        if not self.running or not self.ser or not self.ser.is_open:
            return
        try:
            # 发送读取命令
            awake = bytes([0x01, 0x03, 0x00, 0x1E, 0x00, 0x02, 0xA4, 0x0D])
            self.ser.reset_input_buffer()
            self.ser.write(awake)
        except Exception as e:
            import traceback
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
                    # 发送信号通知UI
                    timestamp = time.perf_counter()
                    self.data_received.emit(timestamp, torque)
                    break
            
            # 清空接收缓冲区，准备下一次接收
            self.rx_buffer = b''

    def stop(self):
        self._stop_event = True
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
                
        # 更新发送定时器间隔
        if self.running and freq is not None:
            interval_ms = int(1000.0 / self.freq) if self.freq > 0 else 1
            self.send_timer.setInterval(interval_ms)
    
    @staticmethod
    def hex2dec(hex_data):
        return int.from_bytes(hex_data.to_bytes(2, 'big', signed=False), 'big', signed=True) 