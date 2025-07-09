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
from PyQt5.QtWidgets import QFileDialog, QLabel, QComboBox, QPushButton, QHBoxLayout, QVBoxLayout, QGroupBox, QSpinBox, QDoubleSpinBox
from safemrc_comm import SafeMRCThread
from torque_sensor import TorqueSensorThread

# -----------------------------
# Main Application Window
# -----------------------------
class MainWindow(QtWidgets.QMainWindow):
    MODES = {0: 'FREE', 1: 'FIX_LIMIT', 2: 'ADAPTATION', 3: 'DEBUG'}
    COMMON_BAUDRATES = [9600, 19200, 38400, 57600, 115200]
    def __init__(self):
        super().__init__()
        self.setWindowTitle('SafeMRC & Torque Sensor Host UI')
        self.resize(1400, 800)
        self._setup_ui()
        self._init_data_buffers()
        # SafeMRC线程
        self.safemrc_thread = SafeMRCThread()
        self.safemrc_thread.data_received.connect(self.on_safemrc_data)
        self.safemrc_thread.status_changed.connect(self.on_safemrc_status)
        self.safemrc_thread.error_signal.connect(lambda msg: self.show_error('SafeMRC', msg))
        # Torque线程
        self.torque_thread = TorqueSensorThread()
        self.torque_thread.data_received.connect(self.on_torque_data)
        self.torque_thread.status_changed.connect(self.on_torque_status)
        self.torque_thread.error_signal.connect(lambda msg: self.show_error('TorqueSensor', msg))
        # 状态
        self.safemrc_connected = False
        self.torque_connected = False
        self.latest_safemrc = None
        self.latest_torque = None
        # 录制
        self.safemrc_recording = False
        self.torque_recording = False
        self.safemrc_record_data = []
        self.torque_record_data = []

    def _setup_ui(self):
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        # --- SafeMRC区域 ---
        safemrc_group = QGroupBox('SafeMRC')
        safemrc_layout = QHBoxLayout()
        self.safemrc_port_combo = QComboBox()
        self.safemrc_refresh_btn = QPushButton('Refresh')
        self.safemrc_refresh_btn.clicked.connect(self.refresh_safemrc_ports)
        self.refresh_safemrc_ports()
        self.safemrc_baud_combo = QComboBox()
        for b in self.COMMON_BAUDRATES:
            self.safemrc_baud_combo.addItem(str(b))
        self.safemrc_baud_combo.setEditable(True)
        self.safemrc_baud_combo.setCurrentText('4000000')
        self.safemrc_connect_btn = QPushButton('Connect')
        self.safemrc_disconnect_btn = QPushButton('Disconnect')
        self.safemrc_disconnect_btn.setEnabled(False)
        self.safemrc_connect_btn.clicked.connect(self.connect_safemrc)
        self.safemrc_disconnect_btn.clicked.connect(self.disconnect_safemrc)
        self.safemrc_freq_spin = QSpinBox()
        self.safemrc_freq_spin.setRange(1, 1000)
        self.safemrc_freq_spin.setValue(20)
        safemrc_layout.addWidget(QLabel('Port:'))
        safemrc_layout.addWidget(self.safemrc_port_combo)
        safemrc_layout.addWidget(self.safemrc_refresh_btn)
        safemrc_layout.addWidget(QLabel('Baudrate:'))
        safemrc_layout.addWidget(self.safemrc_baud_combo)
        safemrc_layout.addWidget(QLabel('Freq(Hz):'))
        safemrc_layout.addWidget(self.safemrc_freq_spin)
        safemrc_layout.addWidget(self.safemrc_connect_btn)
        safemrc_layout.addWidget(self.safemrc_disconnect_btn)
        safemrc_layout.addStretch()
        safemrc_group.setLayout(safemrc_layout)
        # --- Torque区域 ---
        torque_group = QGroupBox('Torque Sensor')
        torque_layout = QHBoxLayout()
        self.torque_port_combo = QComboBox()
        self.torque_refresh_btn = QPushButton('Refresh')
        self.torque_refresh_btn.clicked.connect(self.refresh_torque_ports)
        self.refresh_torque_ports()
        self.torque_baud_combo = QComboBox()
        for b in self.COMMON_BAUDRATES:
            self.torque_baud_combo.addItem(str(b))
        self.torque_baud_combo.setEditable(True)
        self.torque_baud_combo.setCurrentText('115200')
        self.torque_connect_btn = QPushButton('Connect')
        self.torque_disconnect_btn = QPushButton('Disconnect')
        self.torque_disconnect_btn.setEnabled(False)
        self.torque_connect_btn.clicked.connect(self.connect_torque)
        self.torque_disconnect_btn.clicked.connect(self.disconnect_torque)
        self.torque_freq_spin = QSpinBox()
        self.torque_freq_spin.setRange(1, 1000)
        self.torque_freq_spin.setValue(20)
        torque_layout.addWidget(QLabel('Port:'))
        torque_layout.addWidget(self.torque_port_combo)
        torque_layout.addWidget(self.torque_refresh_btn)
        torque_layout.addWidget(QLabel('Baudrate:'))
        torque_layout.addWidget(self.torque_baud_combo)
        torque_layout.addWidget(QLabel('Freq(Hz):'))
        torque_layout.addWidget(self.torque_freq_spin)
        torque_layout.addWidget(self.torque_connect_btn)
        torque_layout.addWidget(self.torque_disconnect_btn)
        torque_layout.addStretch()
        torque_group.setLayout(torque_layout)
        # --- 布局 ---
        top_row = QHBoxLayout()
        top_row.addWidget(safemrc_group)
        top_row.addWidget(torque_group)
        layout.addLayout(top_row)
        # --- SafeMRC绘图 ---
        self.safemrc_plot_widgets = []
        self.safemrc_plot_curves = []
        self.safemrc_plot_titles = ['Encoder Angle (rad)', 'Encoder Velocity (rad/s)', 'Current (A)']
        safemrc_plot_layout = QHBoxLayout()
        for i in range(3):
            pw = pg.PlotWidget()
            pw.setTitle(self.safemrc_plot_titles[i])
            pw.showGrid(x=True, y=True)
            curve = pw.plot([], [], pen=pg.mkPen('b', width=2))
            self.safemrc_plot_widgets.append(pw)
            self.safemrc_plot_curves.append(curve)
            safemrc_plot_layout.addWidget(pw)
        layout.addLayout(safemrc_plot_layout)
        # --- Torque绘图 ---
        self.torque_plot_widget = pg.PlotWidget()
        self.torque_plot_widget.setTitle('Torque (Nm)')
        self.torque_plot_widget.showGrid(x=True, y=True)
        self.torque_plot_curve = self.torque_plot_widget.plot([], [], pen=pg.mkPen('r', width=2))
        layout.addWidget(self.torque_plot_widget)
        # --- SafeMRC录制 ---
        safemrc_record_row = QHBoxLayout()
        self.safemrc_record_start_btn = QPushButton('Start SafeMRC Recording')
        self.safemrc_record_stop_btn = QPushButton('Stop SafeMRC Recording')
        self.safemrc_record_stop_btn.setEnabled(False)
        self.safemrc_record_status = QLabel('Not recording')
        self.safemrc_record_start_btn.clicked.connect(self.start_safemrc_recording)
        self.safemrc_record_stop_btn.clicked.connect(self.stop_safemrc_recording)
        safemrc_record_row.addWidget(self.safemrc_record_start_btn)
        safemrc_record_row.addWidget(self.safemrc_record_stop_btn)
        safemrc_record_row.addWidget(self.safemrc_record_status)
        safemrc_record_row.addStretch()
        layout.addLayout(safemrc_record_row)
        # --- Torque录制 ---
        torque_record_row = QHBoxLayout()
        self.torque_record_start_btn = QPushButton('Start Torque Recording')
        self.torque_record_stop_btn = QPushButton('Stop Torque Recording')
        self.torque_record_stop_btn.setEnabled(False)
        self.torque_record_status = QLabel('Not recording')
        self.torque_record_start_btn.clicked.connect(self.start_torque_recording)
        self.torque_record_stop_btn.clicked.connect(self.stop_torque_recording)
        torque_record_row.addWidget(self.torque_record_start_btn)
        torque_record_row.addWidget(self.torque_record_stop_btn)
        torque_record_row.addWidget(self.torque_record_status)
        torque_record_row.addStretch()
        layout.addLayout(torque_record_row)
        self.setCentralWidget(central)
        # --- 定时器刷新绘图 ---
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(30)

    def _init_data_buffers(self):
        self.max_points = 2000
        # SafeMRC
        self.safemrc_time = np.zeros(self.max_points)
        self.safemrc_angle = np.zeros(self.max_points)
        self.safemrc_velocity = np.zeros(self.max_points)
        self.safemrc_current = np.zeros(self.max_points)
        self.safemrc_ptr = 0
        self.safemrc_count = 0
        # Torque
        self.torque_time = np.zeros(self.max_points)
        self.torque_value = np.zeros(self.max_points)
        self.torque_ptr = 0
        self.torque_count = 0

    # --- SafeMRC串口相关 ---
    def refresh_safemrc_ports(self):
        self.safemrc_port_combo.clear()
        ports = SafeMRCThread.get_available_ports()
        for p in ports:
            self.safemrc_port_combo.addItem(p)
    def connect_safemrc(self):
        port = self.safemrc_port_combo.currentText()
        try:
            baudrate = int(self.safemrc_baud_combo.currentText())
        except ValueError:
            baudrate = 4000000
        freq = self.safemrc_freq_spin.value()
        send_interval = 1.0 / freq
        self.safemrc_thread.configure(port, baudrate, send_interval, 0, 0.0, 1)
        self.safemrc_thread.start()
        self.safemrc_connect_btn.setEnabled(False)
        self.safemrc_disconnect_btn.setEnabled(True)
    def disconnect_safemrc(self):
        self.safemrc_thread.stop()
        self.safemrc_connect_btn.setEnabled(True)
        self.safemrc_disconnect_btn.setEnabled(False)
    def on_safemrc_status(self, ok):
        self.safemrc_connected = ok
    def on_safemrc_data(self, data):
        t = data.get('timestamp', time.perf_counter())
        idx = self.safemrc_ptr % self.max_points
        self.safemrc_time[idx] = t
        self.safemrc_angle[idx] = data.get('encoder', 0.0)
        self.safemrc_velocity[idx] = data.get('velocity', 0.0)
        self.safemrc_current[idx] = data.get('current', 0.0)
        self.safemrc_ptr += 1
        self.safemrc_count = min(self.safemrc_count + 1, self.max_points)
        self.latest_safemrc = data
        if self.safemrc_recording:
            self.safemrc_record_data.append({
                'timestamp': t,
                'encoder': data.get('encoder', 0.0),
                'velocity': data.get('velocity', 0.0),
                'current': data.get('current', 0.0),
                'mode': data.get('mode', 0),
                'collision': data.get('collision', 0),
                'crc_recv': data.get('crc_recv', 0),
                'crc_calc': data.get('crc_calc', 0),
                'raw_frame_hex': ' '.join(f'{b:02X}' for b in data.get('raw_frame', b''))
            })
    # --- Torque串口相关 ---
    def refresh_torque_ports(self):
        self.torque_port_combo.clear()
        ports = TorqueSensorThread.get_available_ports()
        for p in ports:
            self.torque_port_combo.addItem(p)
    def connect_torque(self):
        port = self.torque_port_combo.currentText()
        try:
            baudrate = int(self.torque_baud_combo.currentText())
        except ValueError:
            baudrate = 115200
        freq = self.torque_freq_spin.value()
        interval = 1.0 / freq
        self.torque_thread.configure(port, baudrate, interval)
        self.torque_thread.start()
        self.torque_connect_btn.setEnabled(False)
        self.torque_disconnect_btn.setEnabled(True)
    def disconnect_torque(self):
        self.torque_thread.stop()
        self.torque_connect_btn.setEnabled(True)
        self.torque_disconnect_btn.setEnabled(False)
    def on_torque_status(self, ok):
        self.torque_connected = ok
    def on_torque_data(self, data):
        t = data.get('timestamp', time.perf_counter())
        idx = self.torque_ptr % self.max_points
        self.torque_time[idx] = t
        self.torque_value[idx] = data.get('torque', 0.0)
        self.torque_ptr += 1
        self.torque_count = min(self.torque_count + 1, self.max_points)
        self.latest_torque = data
        if self.torque_recording:
            self.torque_record_data.append({
                'timestamp': t,
                'torque': data.get('torque', 0.0)
            })
    # --- 录制 ---
    def start_safemrc_recording(self):
        self.safemrc_recording = True
        self.safemrc_record_data = []
        self.safemrc_record_start_btn.setEnabled(False)
        self.safemrc_record_stop_btn.setEnabled(True)
        self.safemrc_record_status.setText('Recording...')
    def stop_safemrc_recording(self):
        self.safemrc_recording = False
        self.safemrc_record_start_btn.setEnabled(True)
        self.safemrc_record_stop_btn.setEnabled(False)
        self.safemrc_record_status.setText('Not recording')
        if not self.safemrc_record_data:
            QtWidgets.QMessageBox.information(self, 'No Data', 'No SafeMRC data to save!')
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Save SafeMRC CSV', '', 'CSV Files (*.csv)')
        if not path:
            return
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'timestamp', 'encoder', 'velocity', 'current', 'mode', 'collision', 'crc_recv', 'crc_calc', 'raw_frame_hex'])
            writer.writeheader()
            for row in self.safemrc_record_data:
                writer.writerow(row)
        QtWidgets.QMessageBox.information(self, 'Saved', f'SafeMRC data saved to {path}')
    def start_torque_recording(self):
        self.torque_recording = True
        self.torque_record_data = []
        self.torque_record_start_btn.setEnabled(False)
        self.torque_record_stop_btn.setEnabled(True)
        self.torque_record_status.setText('Recording...')
    def stop_torque_recording(self):
        self.torque_recording = False
        self.torque_record_start_btn.setEnabled(True)
        self.torque_record_stop_btn.setEnabled(False)
        self.torque_record_status.setText('Not recording')
        if not self.torque_record_data:
            QtWidgets.QMessageBox.information(self, 'No Data', 'No torque data to save!')
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Save Torque CSV', '', 'CSV Files (*.csv)')
        if not path:
            return
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['timestamp', 'torque'])
            writer.writeheader()
            for row in self.torque_record_data:
                writer.writerow(row)
        QtWidgets.QMessageBox.information(self, 'Saved', f'Torque data saved to {path}')
    # --- 绘图 ---
    def update_plots(self):
        # SafeMRC
        if self.safemrc_count > 0:
            if self.safemrc_count < self.max_points:
                t_arr = self.safemrc_time[:self.safemrc_count]
                angle_arr = self.safemrc_angle[:self.safemrc_count]
                vel_arr = self.safemrc_velocity[:self.safemrc_count]
                cur_arr = self.safemrc_current[:self.safemrc_count]
            else:
                idx = self.safemrc_ptr % self.max_points
                t_arr = np.concatenate((self.safemrc_time[idx:], self.safemrc_time[:idx]))
                angle_arr = np.concatenate((self.safemrc_angle[idx:], self.safemrc_angle[:idx]))
                vel_arr = np.concatenate((self.safemrc_velocity[idx:], self.safemrc_velocity[:idx]))
                cur_arr = np.concatenate((self.safemrc_current[idx:], self.safemrc_current[:idx]))
            t0 = t_arr[0]
            t_plot = t_arr - t0
            self.safemrc_plot_curves[0].setData(t_plot, angle_arr)
            self.safemrc_plot_curves[1].setData(t_plot, vel_arr)
            self.safemrc_plot_curves[2].setData(t_plot, cur_arr)
            for pw in self.safemrc_plot_widgets:
                pw.setLabel('bottom', 'Time (s)')
        # Torque
        if self.torque_count > 0:
            if self.torque_count < self.max_points:
                t_arr = self.torque_time[:self.torque_count]
                torque_arr = self.torque_value[:self.torque_count]
            else:
                idx = self.torque_ptr % self.max_points
                t_arr = np.concatenate((self.torque_time[idx:], self.torque_time[:idx]))
                torque_arr = np.concatenate((self.torque_value[idx:], self.torque_value[:idx]))
            t0 = t_arr[0]
            t_plot = t_arr - t0
            self.torque_plot_curve.setData(t_plot, torque_arr)
            self.torque_plot_widget.setLabel('bottom', 'Time (s)')
    def show_error(self, who, msg):
        QtWidgets.QMessageBox.critical(self, f'{who} Error', msg)
    def closeEvent(self, event):
        self.safemrc_thread.stop()
        self.torque_thread.stop()
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_()) 