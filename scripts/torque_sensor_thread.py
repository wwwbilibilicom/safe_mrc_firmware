from PyQt5 import QtCore
import time
from torque_sensor import TorqueSensor
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
        self.sensor = None
        self._stop_event = False
        self.lock = QtCore.QMutex()
        self.expected_len = 8  # 期望的最小数据长度

    def configure(self, port, baudrate, freq):
        self.port = port
        self.baudrate = baudrate
        self.freq = freq

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            if not ser.is_open:
                self.status_changed.emit(False)
                self.error_signal.emit(f"Failed to open torque sensor port {self.port}")
                return
            self.status_changed.emit(True)
            self.running = True
            interval = 1.0 / self.freq if self.freq > 0 else 0.001
            awake = bytes([0x01, 0x03, 0x00, 0x1E, 0x00, 0x02, 0xA4, 0x0D])
            while not self._stop_event:
                t_send = time.perf_counter()
                ser.reset_input_buffer()
                ser.write(awake)
                # 阻塞式等待数据
                buffer = b''
                start_wait = time.perf_counter()
                while (time.perf_counter() - start_wait) < 0.5:
                    if ser.in_waiting >= self.expected_len:
                        buffer = ser.read(ser.in_waiting)
                        break
                    time.sleep(0.001)
                t_recv = time.perf_counter()
                torque = 0.0
                if buffer and len(buffer) >= self.expected_len:
                    # 查找帧头并解析数据
                    for i in range(len(buffer) - 6):
                        if buffer[i] == 0x01 and buffer[i + 1] == 0x03:
                            U_DATA = buffer[i + 6]
                            U_DATA |= buffer[i + 5] << 8
                            torque = TorqueSensor.hex2dec(U_DATA) / 30.0
                            break
                self.data_received.emit(t_recv, torque)
                elapsed = time.perf_counter() - t_send
                sleep_time = max(0, interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
            ser.close()
        except Exception as e:
            self.error_signal.emit(str(e))
        finally:
            self.running = False
            self.status_changed.emit(False)

    def stop(self):
        self._stop_event = True
        self.wait()

    def update_params(self, port=None, baudrate=None, freq=None):
        self.lock.lock()
        if port is not None:
            self.port = port
        if baudrate is not None:
            self.baudrate = baudrate
        if freq is not None:
            self.freq = freq
        self.lock.unlock() 