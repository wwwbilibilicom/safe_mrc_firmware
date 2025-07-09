import warnings
warnings.filterwarnings("ignore", category=UserWarning, module='pyqtgraph')
import os
os.environ['QT_LOGGING_RULES'] = '*.debug=false;qt.qpa.*=false'
import sys
import time
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from PyQt5.QtGui import QTextCursor
import csv
from PyQt5.QtWidgets import QFileDialog, QLabel
from safemrc_comm import SerialThread

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
        ports = self.serial_thread.get_available_ports() # Assuming SerialThread has this method
        for p in ports:
            self.port_combo.addItem(p)

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