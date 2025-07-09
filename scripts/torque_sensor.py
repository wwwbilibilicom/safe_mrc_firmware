import serial
import serial.tools.list_ports
import time
from PyQt5 import QtCore

class TorqueSensorThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(dict)  # {'timestamp':..., 'torque':...}
    status_changed = QtCore.pyqtSignal(bool)
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.serial = None
        self.running = False
        self.port = None
        self.baudrate = 115200
        self.sampling_interval = 0.05  # 默认20Hz
        self._stop_event = False
        self.lock = QtCore.QMutex()

    @staticmethod
    def get_available_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    def configure(self, port, baudrate, sampling_interval):
        self.port = port
        self.baudrate = baudrate
        self.sampling_interval = sampling_interval

    def run(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.2)
            self.running = True
            self.status_changed.emit(True)
        except Exception as e:
            self.error_signal.emit(f"Failed to open torque sensor port {self.port}: {e}")
            self.status_changed.emit(False)
            return
        while not self._stop_event:
            t0 = time.perf_counter()
            torque = self.read_torque_raw()
            if torque is not None:
                self.data_received.emit({'timestamp': t0, 'torque': torque})
            t1 = time.perf_counter()
            dt = t1 - t0
            if dt < self.sampling_interval:
                time.sleep(self.sampling_interval - dt)
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.running = False
        self.status_changed.emit(False)

    def stop(self):
        self._stop_event = True
        self.wait()

    def read_torque_raw(self):
        if not self.serial or not self.serial.is_open:
            return None
        try:
            self.serial.reset_input_buffer()
            awake = bytes([0x01, 0x03, 0x00, 0x1E, 0x00, 0x02, 0xA4, 0x0D])
            self.serial.write(awake)
            time.sleep(0.05)
            available = self.serial.in_waiting
            if available < 8:
                return None
            buffer = self.serial.read(available)
            for i in range(len(buffer) - 6):
                if buffer[i] == 0x01 and buffer[i + 1] == 0x03:
                    U_DATA = buffer[i + 6]
                    U_DATA |= buffer[i + 5] << 8
                    return self.hex2dec(U_DATA) / 30.0
        except Exception:
            return None
        return None

    @staticmethod
    def hex2dec(hex_data):
        return int.from_bytes(hex_data.to_bytes(2, 'big', signed=False), 'big', signed=True) 