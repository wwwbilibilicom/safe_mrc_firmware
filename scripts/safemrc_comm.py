import threading
import time
import serial
import serial.tools.list_ports
from PyQt5 import QtCore

# CRC-CCITT查表法，与嵌入式C端完全一致
CRC_CCITT_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbef3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
]
def crc_ccitt(data, crc=0xFFFF):
    for b in data:
        crc = ((crc >> 8) ^ CRC_CCITT_TABLE[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc

class SerialThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(dict)
    status_changed = QtCore.pyqtSignal(bool)
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.running = False
        self.send_interval = 0.05  # default 20Hz
        self.mode = 0
        self.current = 0.0
        self.device_id = 1
        self.lock = QtCore.QMutex()
        self._stop_event = False
        self._sending = False
        self._last_cmd = b''
        self.rx_buffer = b''
        
        # 创建发送定时器，但不要立即moveToThread，而是在run方法中创建
        self.send_timer = None
        self.last_send_time = 0
        
        # 发送频率监控
        self.send_count = 0
        self.last_send_count_time = 0
        self.send_freq_monitor = True
        
        # 性能优化参数
        self.ui_update_interval = 0.1  # 每100ms更新一次UI
        self.last_ui_update_time = 0
        self.batch_size = 1  # 默认每次发送1个命令

    def configure(self, port, baudrate, send_interval, mode, current, device_id):
        self.port = port
        self.baudrate = baudrate
        self.send_interval = send_interval
        self.mode = mode
        self.current = current
        self.device_id = device_id

    def run(self):
        try:
            print(f"SafeMRC: 尝试打开串口 {self.port}，波特率 {self.baudrate}")
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
            print(f"SafeMRC: 串口打开成功，配置: {self.ser}")
            
            self.running = True
            self.rx_buffer = b''
            self.status_changed.emit(True)
            
            # 使用更可靠的时间控制方法
            self.last_send_time = time.perf_counter()
            self.last_send_count_time = time.perf_counter()
            self.last_ui_update_time = time.perf_counter()
            self.send_count = 0
            
            # 根据发送频率调整批处理大小
            if 1.0/self.send_interval > 100:  # 如果频率高于100Hz
                self.batch_size = max(1, int((1.0/self.send_interval) / 50))  # 每批最多发送频率/50个命令
                print(f"SafeMRC: 高频率模式，批处理大小设为 {self.batch_size}")
            
            # 接收循环
            while not self._stop_event:
                try:
                    # 定时发送
                    current_time = time.perf_counter()
                    if self._sending and current_time - self.last_send_time >= self.send_interval:
                        # 高频率发送时，可能需要多次发送以赶上时间
                        send_count_this_cycle = 0
                        while self._sending and current_time - self.last_send_time >= self.send_interval and send_count_this_cycle < self.batch_size:
                            self.send_command(update_ui=(send_count_this_cycle == 0 and current_time - self.last_ui_update_time >= self.ui_update_interval))
                            self.last_send_time += self.send_interval  # 使用固定间隔，避免时间漂移
                            self.send_count += 1
                            send_count_this_cycle += 1
                            
                            # 更新UI时间
                            if current_time - self.last_ui_update_time >= self.ui_update_interval:
                                self.last_ui_update_time = current_time
                        
                        # 监控发送频率
                        if self.send_freq_monitor and current_time - self.last_send_count_time >= 1.0:
                            send_freq = self.send_count / (current_time - self.last_send_count_time)
                            print(f"SafeMRC: 实际发送频率 {send_freq:.2f} Hz，目标频率 {1.0/self.send_interval:.2f} Hz")
                            self.send_count = 0
                            self.last_send_count_time = current_time
                    
                    # 处理接收
                    if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                        self.process_incoming_data()
                        
                    # 根据发送频率动态调整休眠时间，降低CPU占用
                    sleep_time = min(1, max(0.5 * self.send_interval, 0.001))
                    QtCore.QThread.msleep(int(sleep_time * 1000))
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    # 接收错误不中断线程，只记录
                
        except Exception as e:
            import traceback
            msg = f'Failed to open serial port {self.port} at {self.baudrate} baud:\n{e}'
            traceback.print_exc()
            self.error_signal.emit(msg)
        finally:
            if self.send_timer:
                self.send_timer.stop()
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.running = False
            self.status_changed.emit(False)

    def send_command(self, update_ui=True):
        """发送命令函数"""
        if not self._sending or not self.running:
            return
        try:
            with QtCore.QMutexLocker(self.lock):
                cmd = self.pack_cmd(self.mode, self.current)
                self._last_cmd = cmd
            
            if self.ser and self.ser.is_open:
                bytes_written = self.ser.write(cmd)
                # 发送信号通知UI，但不是每次都更新，减少UI负担
                if update_ui:
                    self.data_received.emit({'raw_frame': cmd, 'direction': 'TX'})
            else:
                print(f"SafeMRC serial not open: ser={self.ser}, is_open={self.ser.is_open if self.ser else False}")
        except Exception as e:
            import traceback
            print(f"SafeMRC send_command异常: {e}")
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
        while len(self.rx_buffer) >= 17:  # 最小完整帧长度
            idx = self.rx_buffer.find(b'\xFE\xEE')
            if idx == -1:
                self.rx_buffer = b''
                break
            if len(self.rx_buffer) - idx < 17:
                self.rx_buffer = self.rx_buffer[idx:]
                break
                
            frame = self.rx_buffer[idx:idx+17]
            self.rx_buffer = self.rx_buffer[idx+17:]
            
            data = self.parse_feedback(frame)
            if data:
                data['raw_frame'] = frame
                data['direction'] = 'RX'
                data['timestamp'] = time.perf_counter()  # 高精度时间戳
                self.data_received.emit(data)

    def stop(self):
        self._stop_event = True
        if self.send_timer:
            self.send_timer.stop()
        self.wait()

    def update_params(self, send_interval, mode, current, device_id):
        with QtCore.QMutexLocker(self.lock):
            self.send_interval = send_interval
            self.mode = mode
            self.current = current
            self.device_id = device_id
            
        # 更新发送定时器间隔
        if self.running:
            # interval_ms = int(self.send_interval * 1000) # This line is no longer needed
            # self.send_timer.setInterval(interval_ms) # This line is no longer needed
            pass # The send_command will now use the new time control logic

    def pack_cmd(self, mode, current):
        # Command protocol: 0xFE 0xEE id(1) mode(1) current(int32, 4 bytes) CRC(2)
        head = b'\xFE\xEE'
        id_byte = self.device_id.to_bytes(1, 'little')
        mode_byte = mode.to_bytes(1, 'little')
        cur = int(current * 1000)
        cur_bytes = cur.to_bytes(4, 'little', signed=True)  # int32_t, little-endian
        payload = head + id_byte + mode_byte + cur_bytes
        crc = crc_ccitt(payload)
        crc_bytes = crc.to_bytes(2, 'little')  # little-endian for CRC, to match embedded
        return payload + crc_bytes

    def parse_feedback(self, frame):
        # Feedback: 0xFE 0xEE id(1) mode(1) collision(1) encoder(4) velocity(4) current(2) CRC(2)
        try:
            if frame[0] != 0xFE or frame[1] != 0xEE:
                return None
            id_ = frame[2]
            mode = frame[3]
            collision = frame[4]
            encoder = int.from_bytes(frame[5:9], 'little', signed=True) / 65535.0
            velocity = int.from_bytes(frame[9:13], 'little', signed=True) / 1000.0
            current = int.from_bytes(frame[13:15], 'little', signed=True) / 1000.0
            crc_recv = int.from_bytes(frame[15:17], 'little')
            crc_calc = crc_ccitt(frame[:15])
            return {
                'id': id_,
                'mode': mode,
                'collision': collision,
                'encoder': encoder,
                'velocity': velocity,
                'current': current,
                'crc_recv': crc_recv,
                'crc_calc': crc_calc
            }
        except Exception:
            return None

    def enable_sending(self, enable):
        print(f"SafeMRC: {'启用' if enable else '禁用'}发送，当前状态: _sending={self._sending}, running={self.running}")
        with QtCore.QMutexLocker(self.lock):
            self._sending = enable
        
        # 如果启用发送，重置发送时间
        if enable and self.running:
            self.last_send_time = time.perf_counter() - self.send_interval  # 确保立即发送一次
    
    @staticmethod
    def get_available_ports():
        return [p.device for p in serial.tools.list_ports.comports()] 