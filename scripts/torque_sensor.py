import serial
import time
import threading
import serial.tools.list_ports

class TorqueSensor:
    def __init__(self):
        self.serial = None
        self.initialized = False
        self.lock = threading.Lock()

    def initialize(self, port_name, baudrate):
        try:
            self.serial = serial.Serial()
            self.serial.port = port_name
            self.serial.baudrate = baudrate
            self.serial.timeout = 0.5  # 500ms timeout
            self.serial.open()
            if not self.serial.is_open:
                print(f"Unable to open RS485 port: {port_name}")
                return False
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.1)
            self.initialized = True
            print(f"Torque sensor initialized successfully on port {port_name}")
            return True
        except serial.SerialException as e:
            print(f"Error initializing torque sensor: {e}")
            return False

    def read_torque(self):
        if not self.initialized or not self.serial.is_open:
            print("Torque sensor not initialized or port closed")
            return 0.0
        try:
            with self.lock:
                self.serial.reset_input_buffer()
                # 发送读取命令
                awake = bytes([0x01, 0x03, 0x00, 0x1E, 0x00, 0x02, 0xA4, 0x0D])
                bytes_written = self.serial.write(awake)
                if bytes_written != len(awake):
                    print("Failed to write complete command")
                    return 0.0
                time.sleep(0.05)
                available = self.serial.in_waiting
                if available < 8:
                    print(f"Insufficient data available: {available} bytes")
                    return 0.0
                buffer = self.serial.read(available)
                # 查找帧头并解析数据
                for i in range(len(buffer) - 6):
                    if buffer[i] == 0x01 and buffer[i + 1] == 0x03:
                        U_DATA = buffer[i + 6]
                        U_DATA |= buffer[i + 5] << 8
                        return self.hex2dec(U_DATA) / 100.0
                print("Valid frame header not found in response")
        except serial.SerialException as e:
            print(f"Error reading torque sensor: {e}")
        return 0.0

    def is_initialized(self):
        return self.initialized

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.initialized = False

    @staticmethod
    def get_available_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    @staticmethod
    def hex2dec(hex_data):
        return int.from_bytes(hex_data.to_bytes(2, 'big', signed=False), 'big', signed=True) 