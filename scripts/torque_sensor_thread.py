from PyQt5 import QtCore
import time
from torque_sensor import TorqueSensor

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

    def configure(self, port, baudrate, freq):
        self.port = port
        self.baudrate = baudrate
        self.freq = freq

    def run(self):
        self.sensor = TorqueSensor()
        try:
            if not self.sensor.initialize(self.port, self.baudrate):
                self.status_changed.emit(False)
                self.error_signal.emit(f"Failed to open torque sensor port {self.port}")
                return
            self.status_changed.emit(True)
            self.running = True
            interval = 1.0 / self.freq if self.freq > 0 else 0.001
            while not self._stop_event:
                t0 = time.perf_counter()
                torque = self.sensor.read_torque()
                t1 = time.perf_counter()
                self.data_received.emit(t1, torque)
                elapsed = t1 - t0
                sleep_time = max(0, interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except Exception as e:
            self.error_signal.emit(str(e))
        finally:
            self.running = False
            self.status_changed.emit(False)
            if self.sensor:
                self.sensor.close()

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