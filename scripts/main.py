import warnings
warnings.filterwarnings("ignore", category=UserWarning, module='pyqtgraph')
import os
os.environ['QT_LOGGING_RULES'] = '*.debug=false;qt.qpa.*=false'
import sys
import time
import threading
import serial
import serial.tools.list_ports
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from PyQt5.QtGui import QTextCursor
import csv
from PyQt5.QtWidgets import QFileDialog, QLabel

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

# -----------------------------
# Serial Communication Thread
# -----------------------------
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
        self.lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sending = False
        self._last_cmd = b''

    def configure(self, port, baudrate, send_interval, mode, current, device_id):
        self.port = port
        self.baudrate = baudrate
        self.send_interval = send_interval
        self.mode = mode
        self.current = current
        self.device_id = device_id

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            self.status_changed.emit(True)
        except Exception as e:
            import traceback
            msg = f'Failed to open serial port {self.port} at {self.baudrate} baud:\n{e}'
            print(msg)
            traceback.print_exc()
            self.error_signal.emit(msg)
            self.status_changed.emit(False)
            return
        last_send = time.time()
        buffer = b''
        while not self._stop_event.is_set():
            now = time.time()
            # Only send command if enabled
            if self._sending and (now - last_send >= self.send_interval):
                with self.lock:
                    cmd = self.pack_cmd(self.mode, self.current)
                    self._last_cmd = cmd
                self.ser.write(cmd)
                last_send = now
                # Emit TX hex log
                self.data_received.emit({'raw_frame': cmd, 'direction': 'TX'})
            # Read feedback
            try:
                if self.ser.in_waiting:
                    buffer += self.ser.read(self.ser.in_waiting)
                    while len(buffer) >= 17:
                        idx = buffer.find(b'\xFE\xEE')
                        if idx == -1:
                            buffer = b''
                            break
                        if len(buffer) - idx < 17:
                            break
                        frame = buffer[idx:idx+17]
                        buffer = buffer[idx+17:]
                        data = self.parse_feedback(frame)
                        if data:
                            data['raw_frame'] = frame
                            data['direction'] = 'RX'
                            data['timestamp'] = time.perf_counter()  # 高精度时间戳
                            self.data_received.emit(data)
            except Exception:
                pass
            time.sleep(0.002)
        if self.ser:
            self.ser.close()
        self.running = False
        self.status_changed.emit(False)

    def stop(self):
        self._stop_event.set()
        self.wait()

    def update_params(self, send_interval, mode, current, device_id):
        with self.lock:
            self.send_interval = send_interval
            self.mode = mode
            self.current = current
            self.device_id = device_id

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
            crc_calc = self.crc_ccitt(frame[:15])
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

    def crc_ccitt(self, data):
        # Old bitwise implementation, replaced by crcmod for full compatibility
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def enable_sending(self, enable):
        with self.lock:
            self._sending = enable

# -----------------------------
# Main Application Window
# -----------------------------
class MainWindow(QtWidgets.QMainWindow):
    MODES = {0: 'FREE', 1: 'FIX_LIMIT', 2: 'ADAPTATION', 3: 'DEBUG'}
    def __init__(self):
        super().__init__()
        self.setWindowTitle('SafeMRC Host UI')
        self.resize(1000, 700)
        self.serial_thread = SerialThread()
        self.serial_thread.data_received.connect(self.on_data_received)
        self.serial_thread.status_changed.connect(self.on_status_changed)
        self.serial_thread.error_signal.connect(self.show_serial_error)
        self._setup_ui()
        self._init_data_buffers()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(30)
        self.connected = False
        self.last_data_time = time.time()

    def _setup_ui(self):
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        # Top row: Serial port selection and connect/disconnect
        top_row = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_btn = QtWidgets.QPushButton('Refresh')
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.refresh_ports()
        self.connect_btn = QtWidgets.QPushButton('Connect')
        self.disconnect_btn = QtWidgets.QPushButton('Disconnect')
        self.start_btn = QtWidgets.QPushButton('Start Sending')
        self.start_btn.setEnabled(False)
        self.start_btn.setCheckable(True)
        self.start_btn.clicked.connect(self.toggle_sending)
        self.disconnect_btn.setEnabled(False)
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        top_row.addWidget(self.connect_btn)
        top_row.addWidget(self.disconnect_btn)
        top_row.addWidget(self.start_btn)
        top_row.addWidget(QtWidgets.QLabel('Serial Port:'))
        top_row.addWidget(self.port_combo)
        top_row.addWidget(self.refresh_btn)
        top_row.addStretch()
        layout.addLayout(top_row)
        # Add hex log display below serial row
        self.hex_log = QtWidgets.QTextEdit()
        self.hex_log.setReadOnly(True)
        self.hex_log.setMaximumHeight(80)
        layout.addWidget(self.hex_log)
        # Control parameters
        ctrl_row = QtWidgets.QHBoxLayout()
        ctrl_row.addWidget(QtWidgets.QLabel('MRC ID:'))
        self.id_spin = QtWidgets.QSpinBox()
        self.id_spin.setRange(1, 255)
        self.id_spin.setValue(1)
        ctrl_row.addWidget(self.id_spin)
        ctrl_row.addWidget(QtWidgets.QLabel('Mode:'))
        self.mode_combo = QtWidgets.QComboBox()
        for k, v in self.MODES.items():
            self.mode_combo.addItem(v, k)
        ctrl_row.addWidget(self.mode_combo)
        ctrl_row.addWidget(QtWidgets.QLabel('Send Freq (Hz):'))
        self.freq_spin = QtWidgets.QSpinBox()
        self.freq_spin.setRange(1, 1000)
        self.freq_spin.setValue(20)
        ctrl_row.addWidget(self.freq_spin)
        ctrl_row.addWidget(QtWidgets.QLabel('Current (A):'))
        self.current_spin = QtWidgets.QDoubleSpinBox()
        self.current_spin.setRange(-5.0, 5.0)
        self.current_spin.setDecimals(3)
        self.current_spin.setSingleStep(0.01)
        self.current_spin.setValue(0.0)
        ctrl_row.addWidget(self.current_spin)
        ctrl_row.addStretch()
        layout.addLayout(ctrl_row)
        # Feedback display
        fbk_group = QtWidgets.QGroupBox('Feedback')
        fbk_layout = QtWidgets.QGridLayout(fbk_group)
        self.fbk_labels = {}
        fields = ['CRC (recv)', 'CRC (calc)', 'Current (A)', 'Mode', 'Encoder Angle (rad)', 'Encoder Velocity (rad/s)', 'Collision']
        for i, name in enumerate(fields):
            label = QtWidgets.QLabel('--')
            fbk_layout.addWidget(QtWidgets.QLabel(name+':'), i, 0)
            fbk_layout.addWidget(label, i, 1)
            self.fbk_labels[name] = label
        layout.addWidget(fbk_group)
        # Plot controls
        plot_ctrl_row = QtWidgets.QHBoxLayout()
        plot_ctrl_row.addWidget(QtWidgets.QLabel('Plot Window (s):'))
        self.plot_window_spin = QtWidgets.QSpinBox()
        self.plot_window_spin.setRange(1, 30)
        self.plot_window_spin.setValue(1)
        plot_ctrl_row.addWidget(self.plot_window_spin)
        # Add clear plots button
        self.clear_plots_btn = QtWidgets.QPushButton('Clear Plots')
        self.clear_plots_btn.clicked.connect(self.clear_plots)
        plot_ctrl_row.addWidget(self.clear_plots_btn)
        plot_ctrl_row.addStretch()
        layout.addLayout(plot_ctrl_row)
        # Plots
        plot_layout = QtWidgets.QHBoxLayout()
        self.plot_widgets = []
        self.plot_curves = []
        self.plot_titles = ['Encoder Angle (rad)', 'Encoder Velocity (rad/s)', 'Current (A)']
        for i in range(3):
            pw = pg.PlotWidget()
            pw.setTitle(self.plot_titles[i])
            pw.showGrid(x=True, y=True)
            curve = pw.plot([], [], pen=pg.mkPen('b', width=2))
            self.plot_widgets.append(pw)
            self.plot_curves.append(curve)
            plot_layout.addWidget(pw)
        layout.addLayout(plot_layout)
        # --- Data recording controls ---
        record_row = QtWidgets.QHBoxLayout()
        self.record_start_btn = QtWidgets.QPushButton('Start Recording')
        self.record_stop_btn = QtWidgets.QPushButton('Stop Recording')
        self.record_stop_btn.setEnabled(False)
        self.record_status = QLabel('Not recording')
        self.record_start_btn.clicked.connect(self.start_recording)
        self.record_stop_btn.clicked.connect(self.stop_recording)
        record_row.addWidget(self.record_start_btn)
        record_row.addWidget(self.record_stop_btn)
        record_row.addWidget(self.record_status)
        record_row.addStretch()
        layout.addLayout(record_row)
        self.setCentralWidget(central)

    def _init_data_buffers(self):
        self.plot_window = 1.0
        self.max_points = 2000
        self.data_time = np.zeros(self.max_points)
        self.data_angle = np.zeros(self.max_points)
        self.data_velocity = np.zeros(self.max_points)
        self.data_current = np.zeros(self.max_points)
        self.data_ptr = 0
        self.data_count = 0
        # --- Data recording ---
        self.recording = False
        self.record_data = []
        self.record_start_time = None
        # --- Plot time zero for clear plots ---
        self.plot_time_zero = None

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)

    def connect_serial(self):
        port = self.port_combo.currentText()
        if not port:
            QtWidgets.QMessageBox.warning(self, 'Warning', 'No serial port selected!')
            return
        baudrate = 4000000  # fixed for SafeMRC
        freq = self.freq_spin.value()
        send_interval = 1.0 / freq
        mode = self.mode_combo.currentData()
        current = self.current_spin.value()
        device_id = self.id_spin.value()
        self.serial_thread.configure(port, baudrate, send_interval, mode, current, device_id)
        try:
            self.serial_thread.start()
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.start_btn.setEnabled(True)
        except Exception as e:
            import traceback
            QtWidgets.QMessageBox.critical(self, 'Serial Error', f'Failed to start serial thread:\n{e}')
            print('Serial thread start error:', e)
            traceback.print_exc()
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.start_btn.setEnabled(False)

    def disconnect_serial(self):
        self.serial_thread.stop()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.start_btn.setEnabled(False)
        self.start_btn.setChecked(False)

    def on_status_changed(self, ok):
        self.connected = ok
        if not ok:
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.start_btn.setEnabled(False)

    def on_data_received(self, data):
        # Show TX/RX hex frame
        if 'raw_frame' in data and data.get('direction') == 'RX':
            self.append_hex_log('RX', data['raw_frame'])
        elif 'raw_frame' in data and data.get('direction') == 'TX':
            self.append_hex_log('TX', data['raw_frame'])
        # Only update feedback labels for RX with feedback fields
        if data.get('direction') == 'RX' and 'crc_recv' in data:
            # --- Timestamp for recording ---
            msg_time = data.get('timestamp', time.perf_counter())
            # --- Plot time zero logic ---
            if self.plot_time_zero is None:
                self.plot_time_zero = msg_time
            if self.recording:
                if self.record_start_time is None:
                    self.record_start_time = msg_time
                timestamp = msg_time - self.record_start_time
                self.record_data.append({
                    'timestamp': timestamp,
                    'encoder': data.get('encoder'),  # now in rad
                    'velocity': data.get('velocity'),  # now in rad/s
                    'current': data.get('current'),
                    'mode': data.get('mode'),
                    'collision': data.get('collision'),
                    'crc_recv': data.get('crc_recv'),
                    'crc_calc': data.get('crc_calc'),
                    'raw_frame_hex': ' '.join(f'{b:02X}' for b in data['raw_frame'])
                })
            self.fbk_labels['CRC (recv)'].setText(f"0x{data['crc_recv']:04X}")
            self.fbk_labels['CRC (calc)'].setText(f"0x{data['crc_calc']:04X}")
            self.fbk_labels['Current (A)'].setText(f"{data['current']:.3f}")
            self.fbk_labels['Mode'].setText(self.MODES.get(data['mode'], str(data['mode'])))
            self.fbk_labels['Encoder Angle (rad)'].setText(f"{data['encoder']:.6f}")
            self.fbk_labels['Encoder Velocity (rad/s)'].setText(f"{data['velocity']:.6f}")
            self.fbk_labels['Collision'].setText('Yes' if data['collision'] else 'No')
            # Store data for plotting
            t = msg_time
            idx = self.data_ptr % self.max_points
            self.data_time[idx] = t
            self.data_angle[idx] = data['encoder']
            self.data_velocity[idx] = data['velocity']
            self.data_current[idx] = data['current']
            self.data_ptr += 1
            self.data_count = min(self.data_count + 1, self.max_points)
            self.last_data_time = t

    def update_plots(self):
        self.plot_window = self.plot_window_spin.value()
        if self.data_count == 0:
            return
        # Handle ring buffer for time and data arrays
        if self.data_count < self.max_points:
            t_arr = self.data_time[:self.data_count]
            angle_arr = self.data_angle[:self.data_count]
            vel_arr = self.data_velocity[:self.data_count]
            cur_arr = self.data_current[:self.data_count]
            t_now = t_arr[-1]
        else:
            idx = self.data_ptr % self.max_points
            t_arr = np.concatenate((self.data_time[idx:], self.data_time[:idx]))
            angle_arr = np.concatenate((self.data_angle[idx:], self.data_angle[:idx]))
            vel_arr = np.concatenate((self.data_velocity[idx:], self.data_velocity[:idx]))
            cur_arr = np.concatenate((self.data_current[idx:], self.data_current[:idx]))
            t_now = t_arr[-1]
        # Use plot_time_zero for x axis
        if self.plot_time_zero is not None:
            t_plot = t_arr - self.plot_time_zero
        else:
            t_plot = t_arr
        mask = (t_arr > t_now - self.plot_window)
        if not np.any(mask):
            for curve in self.plot_curves:
                curve.setData([], [])
            return
        self.plot_curves[0].setData(t_plot[mask], angle_arr[mask])
        self.plot_curves[1].setData(t_plot[mask], vel_arr[mask])
        self.plot_curves[2].setData(t_plot[mask], cur_arr[mask])
        for pw in self.plot_widgets:
            pw.setLabel('bottom', 'Time (s)')

    def closeEvent(self, event):
        if self.serial_thread.running:
            self.serial_thread.stop()
        event.accept()

    # Update parameters when user changes them
    def _update_params(self):
        if self.connected:
            freq = self.freq_spin.value()
            send_interval = 1.0 / freq
            mode = self.mode_combo.currentData()
            current = self.current_spin.value()
            device_id = self.id_spin.value()
            self.serial_thread.update_params(send_interval, mode, current, device_id)

    # Connect parameter changes to update
    def showEvent(self, event):
        self.freq_spin.valueChanged.connect(self._update_params)
        self.mode_combo.currentIndexChanged.connect(self._update_params)
        self.current_spin.valueChanged.connect(self._update_params)
        self.plot_window_spin.valueChanged.connect(self.update_plots)
        self.id_spin.valueChanged.connect(self._update_params)
        super().showEvent(event)

    def show_serial_error(self, msg):
        QtWidgets.QMessageBox.critical(self, 'Serial Error', msg)

    def toggle_sending(self):
        if not self.connected:
            QtWidgets.QMessageBox.warning(self, 'Warning', 'Please connect to serial port first!')
            self.start_btn.setChecked(False)
            return
        if self.start_btn.isChecked():
            self.start_btn.setText('Stop Sending')
            self.serial_thread.enable_sending(True)
        else:
            self.start_btn.setText('Start Sending')
            self.serial_thread.enable_sending(False)

    def append_hex_log(self, direction, frame_bytes):
        # direction: 'TX' or 'RX'
        hex_str = ' '.join(f'{b:02X}' for b in frame_bytes)
        self.hex_log.append(f'<b>{direction}:</b> {hex_str}')
        self.hex_log.moveCursor(QTextCursor.End)

    def start_recording(self):
        self.recording = True
        self.record_data = []
        self.record_start_time = None
        self.record_start_btn.setEnabled(False)
        self.record_stop_btn.setEnabled(True)
        self.record_status.setText('Recording...')
    def stop_recording(self):
        self.recording = False
        self.record_start_btn.setEnabled(True)
        self.record_stop_btn.setEnabled(False)
        self.record_status.setText('Not recording')
        if not self.record_data:
            QtWidgets.QMessageBox.information(self, 'No Data', 'No data to save!')
            return
        # Ask user for file path
        path, _ = QFileDialog.getSaveFileName(self, 'Save CSV', '', 'CSV Files (*.csv)')
        if not path:
            return
        # Write CSV
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'timestamp', 'encoder', 'velocity', 'current', 'mode', 'collision', 'crc_recv', 'crc_calc', 'raw_frame_hex'])
            writer.writeheader()
            for row in self.record_data:
                writer.writerow(row)
        QtWidgets.QMessageBox.information(self, 'Saved', f'Data saved to {path}')

    def clear_plots(self):
        # Clear all plot data and reset time
        self.data_time[:] = 0
        self.data_angle[:] = 0
        self.data_velocity[:] = 0
        self.data_current[:] = 0
        self.data_ptr = 0
        self.data_count = 0
        self.plot_time_zero = None
        for curve in self.plot_curves:
            curve.setData([], [])

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_()) 