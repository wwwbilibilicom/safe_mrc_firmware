"""
SafeMRC上位机主程序，提供GUI界面和数据处理功能
"""
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module='pyqtgraph')
import os
os.environ['QT_LOGGING_RULES'] = '*.debug=false;qt.qpa.*=false'
import sys
import time
import numpy as np
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QLabel, QMessageBox, QFileDialog, QApplication

# 导入自定义模块
from config import (
    APP_NAME, DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT,
    SAFEMRC_MODES, DEFAULT_DEVICE_ID, DEFAULT_SEND_FREQ, DEFAULT_CURRENT, DEFAULT_MODE,
    SAFEMRC_BAUDRATE, DEFAULT_TORQUE_BAUDRATE, DEFAULT_TORQUE_FREQ,
    PLOT_UPDATE_INTERVAL, PLOT_TITLES
)
from error_handler import ErrorHandler, ErrorSeverity, logger
from safemrc_comm import SerialThread
from torque_sensor_thread import TorqueSensorThread
from ui_components import (
    HexLogWidget, PlotWidget, SerialPortSelector,
    FeedbackDisplay, MultiPlotWidget
)
from data_manager import DataRecorder, PlotDataManager

class MainWindow(QtWidgets.QMainWindow):
    """SafeMRC主界面窗口"""
    
    def __init__(self):
        """初始化主界面窗口"""
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT)
        
        # 初始化数据管理器
        self.safemrc_data_manager = PlotDataManager()
        self.torque_data_manager = PlotDataManager()
        self.data_recorder = DataRecorder(self)
        
        # 初始化通信线程
        self.serial_thread = SerialThread()
        self.torque_thread = TorqueSensorThread()
        
        # 初始化状态变量
        self.connected = False
        self.torque_connected = False
        self.last_data_time = time.time()
        
        # 设置UI
        self._setup_ui()
        
        # 连接信号
        self._connect_signals()
        
        # 启动定时器
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        # 使用精确定时器，并降低优先级
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.start(PLOT_UPDATE_INTERVAL)
        
        logger.info("应用程序已启动")
        
    def _setup_ui(self):
        """设置UI界面"""
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        
        # ===== SafeMRC控制区域 =====
        self._setup_safemrc_controls(layout)
        
        # ===== 十六进制日志显示区域 =====
        self.hex_log = HexLogWidget()
        layout.addWidget(self.hex_log)
        
        # ===== SafeMRC参数控制区域 =====
        self._setup_safemrc_params(layout)
        
        # ===== 反馈数据显示区域 =====
        self.feedback_display = FeedbackDisplay("反馈数据")
        for name in ['CRC (recv)', 'CRC (calc)', 'Current (A)', 'Mode', 
                    'Encoder Angle (rad)', 'Encoder Velocity (rad/s)', 'Collision']:
            self.feedback_display.add_field(name)
        layout.addWidget(self.feedback_display)
        
        # ===== 扭矩传感器控制区域 =====
        self._setup_torque_controls(layout)
        
        # ===== 绘图控制区域 =====
        self._setup_plot_controls(layout)
        
        # ===== SafeMRC绘图区域 =====
        self.plot_widget = MultiPlotWidget(titles=PLOT_TITLES[:3])
        layout.addWidget(self.plot_widget)
        
        # ===== 扭矩传感器绘图区域 =====
        self.torque_plot = PlotWidget(PLOT_TITLES[3], 'r')
        torque_plot_row = QtWidgets.QHBoxLayout()
        torque_plot_row.addWidget(self.torque_plot.get_widget())
        layout.addLayout(torque_plot_row)
        
        # ===== 数据记录控制区域 =====
        self._setup_record_controls(layout)
        
        self.setCentralWidget(central)
        
        # 刷新串口列表
        QtCore.QTimer.singleShot(100, lambda: self.port_selector.refresh_ports(force=True))
        QtCore.QTimer.singleShot(100, lambda: self.torque_port_selector.refresh_ports(force=True))
        
    def _setup_safemrc_controls(self, layout):
        """设置SafeMRC控制区域"""
        top_row = QtWidgets.QHBoxLayout()
        
        # 连接/断开按钮
        self.connect_btn = QtWidgets.QPushButton('连接')
        self.disconnect_btn = QtWidgets.QPushButton('断开')
        self.start_btn = QtWidgets.QPushButton('开始发送')
        
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.start_btn.clicked.connect(self.toggle_sending)
        
        self.disconnect_btn.setEnabled(False)
        self.start_btn.setEnabled(False)
        self.start_btn.setCheckable(True)
        
        top_row.addWidget(self.connect_btn)
        top_row.addWidget(self.disconnect_btn)
        top_row.addWidget(self.start_btn)
        
        # 串口选择
        self.port_selector = SerialPortSelector(label_text='串口:')
        # 连接刷新按钮信号
        self.port_selector.refresh_btn.clicked.connect(lambda: self.port_selector.refresh_ports(force=True))
        top_row.addWidget(self.port_selector)
        
        top_row.addStretch()
        layout.addLayout(top_row)
        
    def _setup_safemrc_params(self, layout):
        """设置SafeMRC参数控制区域"""
        ctrl_row = QtWidgets.QHBoxLayout()
        
        # MRC ID
        ctrl_row.addWidget(QtWidgets.QLabel('MRC ID:'))
        self.id_spin = QtWidgets.QSpinBox()
        self.id_spin.setRange(1, 255)
        self.id_spin.setValue(DEFAULT_DEVICE_ID)
        ctrl_row.addWidget(self.id_spin)
        
        # 模式
        ctrl_row.addWidget(QtWidgets.QLabel('模式:'))
        self.mode_combo = QtWidgets.QComboBox()
        for k, v in SAFEMRC_MODES.items():
            self.mode_combo.addItem(v, k)
        ctrl_row.addWidget(self.mode_combo)
        
        # 发送频率
        ctrl_row.addWidget(QtWidgets.QLabel('发送频率 (Hz):'))
        self.freq_spin = QtWidgets.QSpinBox()
        self.freq_spin.setRange(1, 1000)
        self.freq_spin.setValue(DEFAULT_SEND_FREQ)
        ctrl_row.addWidget(self.freq_spin)
        
        # 电流
        ctrl_row.addWidget(QtWidgets.QLabel('电流 (A):'))
        self.current_spin = QtWidgets.QDoubleSpinBox()
        self.current_spin.setRange(-5.0, 5.0)
        self.current_spin.setDecimals(3)
        self.current_spin.setSingleStep(0.01)
        self.current_spin.setValue(DEFAULT_CURRENT)
        ctrl_row.addWidget(self.current_spin)
        
        ctrl_row.addStretch()
        layout.addLayout(ctrl_row)
        
    def _setup_torque_controls(self, layout):
        """设置扭矩传感器控制区域"""
        torque_row = QtWidgets.QHBoxLayout()
        
        # 串口选择
        self.torque_port_selector = SerialPortSelector(label_text='扭矩传感器串口:')
        # 连接刷新按钮信号
        self.torque_port_selector.refresh_btn.clicked.connect(lambda: self.torque_port_selector.refresh_ports(force=True))
        torque_row.addWidget(self.torque_port_selector)
        
        # 波特率
        torque_row.addWidget(QtWidgets.QLabel('波特率:'))
        self.torque_baud_combo = QtWidgets.QComboBox()
        for b in [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]:
            self.torque_baud_combo.addItem(str(b))
        self.torque_baud_combo.setEditable(True)
        self.torque_baud_combo.setCurrentText(str(DEFAULT_TORQUE_BAUDRATE))
        torque_row.addWidget(self.torque_baud_combo)
        
        # 采样频率
        torque_row.addWidget(QtWidgets.QLabel('采样频率 (Hz):'))
        self.torque_freq_spin = QtWidgets.QSpinBox()
        self.torque_freq_spin.setRange(1, 5000)
        self.torque_freq_spin.setValue(DEFAULT_TORQUE_FREQ)
        torque_row.addWidget(self.torque_freq_spin)
        
        # 连接/断开按钮
        self.torque_connect_btn = QtWidgets.QPushButton('连接扭矩传感器')
        self.torque_disconnect_btn = QtWidgets.QPushButton('断开扭矩传感器')
        self.torque_connect_btn.clicked.connect(self.connect_torque)
        self.torque_disconnect_btn.clicked.connect(self.disconnect_torque)
        self.torque_disconnect_btn.setEnabled(False)
        torque_row.addWidget(self.torque_connect_btn)
        torque_row.addWidget(self.torque_disconnect_btn)
        
        # 状态显示
        self.torque_status_label = QtWidgets.QLabel('未连接')
        torque_row.addWidget(self.torque_status_label)
        
        # 采样控制按钮
        self.torque_start_btn = QtWidgets.QPushButton('开始采样')
        self.torque_stop_btn = QtWidgets.QPushButton('停止采样')
        self.torque_start_btn.setEnabled(False)
        self.torque_stop_btn.setEnabled(False)
        self.torque_start_btn.clicked.connect(self.start_torque_sampling)
        self.torque_stop_btn.clicked.connect(self.stop_torque_sampling)
        torque_row.addWidget(self.torque_start_btn)
        torque_row.addWidget(self.torque_stop_btn)
        
        torque_row.addStretch()
        layout.addLayout(torque_row)
        
    def _setup_plot_controls(self, layout):
        """设置绘图控制区域"""
        plot_ctrl_row = QtWidgets.QHBoxLayout()
        
        # 时间窗口
        plot_ctrl_row.addWidget(QtWidgets.QLabel('绘图窗口 (秒):'))
        self.plot_window_spin = QtWidgets.QSpinBox()
        self.plot_window_spin.setRange(1, 30)
        self.plot_window_spin.setValue(1)
        plot_ctrl_row.addWidget(self.plot_window_spin)
        
        # 清空按钮
        self.clear_plots_btn = QtWidgets.QPushButton('清空图表')
        self.clear_plots_btn.clicked.connect(self.clear_plots)
        plot_ctrl_row.addWidget(self.clear_plots_btn)
        
        plot_ctrl_row.addStretch()
        layout.addLayout(plot_ctrl_row)
        
    def _setup_record_controls(self, layout):
        """设置数据记录控制区域"""
        record_row = QtWidgets.QHBoxLayout()
        
        self.record_start_btn = QtWidgets.QPushButton('开始记录')
        self.record_stop_btn = QtWidgets.QPushButton('停止记录')
        self.record_status = QLabel('未记录')
        
        self.record_start_btn.clicked.connect(self.start_recording)
        self.record_stop_btn.clicked.connect(self.stop_recording)
        self.record_stop_btn.setEnabled(False)
        
        record_row.addWidget(self.record_start_btn)
        record_row.addWidget(self.record_stop_btn)
        record_row.addWidget(self.record_status)
        record_row.addStretch()
        
        layout.addLayout(record_row)
        
    def _connect_signals(self):
        """连接信号和槽"""
        # SafeMRC线程信号
        self.serial_thread.data_received.connect(self.on_data_received)
        self.serial_thread.status_changed.connect(self.on_status_changed)
        self.serial_thread.error_signal.connect(self.show_serial_error)
        
        # 扭矩传感器线程信号
        self.torque_thread.data_received.connect(self.on_torque_data)
        self.torque_thread.status_changed.connect(self.on_torque_status)
        self.torque_thread.error_signal.connect(self.on_torque_error)
        
        # 数据记录器信号
        self.data_recorder.recording_started.connect(self.on_recording_started)
        self.data_recorder.recording_stopped.connect(self.on_recording_stopped)
        
        # UI控件信号
        self.freq_spin.valueChanged.connect(self._update_params)
        self.mode_combo.currentIndexChanged.connect(self._update_params)
        self.current_spin.valueChanged.connect(self._update_params)
        self.id_spin.valueChanged.connect(self._update_params)
        self.plot_window_spin.valueChanged.connect(self.update_plots)
        
    def connect_serial(self):
        """连接SafeMRC设备"""
        port = self.port_selector.get_current_port()
        if not port:
            ErrorHandler.show_error_dialog(
                self, "警告", "未选择串口！", ErrorSeverity.WARNING)
            return
            
        freq = self.freq_spin.value()
        send_interval = 1.0 / freq
        mode = self.mode_combo.currentData()
        current = self.current_spin.value()
        device_id = self.id_spin.value()
        
        try:
            if self.serial_thread.isRunning():
                self.serial_thread.stop()
                
            self.serial_thread = SerialThread()
            self.serial_thread.data_received.connect(self.on_data_received)
            self.serial_thread.status_changed.connect(self.on_status_changed)
            self.serial_thread.error_signal.connect(self.show_serial_error)
            
            self.serial_thread.configure(
                port, SAFEMRC_BAUDRATE, send_interval, mode, current, device_id)
            self.serial_thread.start()
            
            # 更新UI状态
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.start_btn.setEnabled(True)
            self.port_selector.set_enabled(False)
            
            # 确保所有参数控件可用（未开始发送时）
            self.freq_spin.setEnabled(True)
            self.mode_combo.setEnabled(True)
            self.current_spin.setEnabled(True)
            self.id_spin.setEnabled(True)
            
            logger.info(f"已连接到SafeMRC设备: {port}")
            
        except Exception as e:
            ErrorHandler.handle_exception(
                self, e, "连接SafeMRC设备失败", ErrorSeverity.ERROR)
            
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.start_btn.setEnabled(False)
            
    def disconnect_serial(self):
        """断开SafeMRC串口连接"""
        if not self.connected:
            return
            
        # 停止发送
        if self.start_btn.isChecked():
            self.start_btn.setChecked(False)
            self.start_btn.setText('开始发送')
            self.serial_thread.enable_sending(False)
        
        # 先禁用所有控件，防止在停止线程过程中用户操作
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(False)
        self.start_btn.setEnabled(False)
        self.port_selector.set_enabled(False)
        self.freq_spin.setEnabled(False)
        self.mode_combo.setEnabled(False)
        self.current_spin.setEnabled(False)
        self.id_spin.setEnabled(False)
        
        # 停止线程
        QApplication.processEvents()  # 处理所有待处理的事件
        self.serial_thread.stop()
        
        # 更新UI状态
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.start_btn.setEnabled(False)
        self.port_selector.set_enabled(True)
        self.connected = False
        
        # 确保参数控件可用
        self.freq_spin.setEnabled(True)
        self.mode_combo.setEnabled(True)
        self.current_spin.setEnabled(True)
        self.id_spin.setEnabled(True)
        
        # 清空图表
        self.safemrc_data_manager.clear()
        self.plot_widget.clear()
        self.feedback_display.clear()
        
        logger.info("已断开SafeMRC设备连接")
        
    def connect_torque(self):
        """连接扭矩传感器"""
        port = self.torque_port_selector.get_current_port()
        if not port:
            ErrorHandler.show_error_dialog(
                self, "警告", "未选择扭矩传感器串口！", ErrorSeverity.WARNING)
            return
            
        try:
            baud = int(self.torque_baud_combo.currentText())
            freq = self.torque_freq_spin.value()
            
            if self.torque_thread.isRunning():
                self.torque_thread.stop()
                
            self.torque_thread = TorqueSensorThread()
            self.torque_thread.data_received.connect(self.on_torque_data)
            self.torque_thread.status_changed.connect(self.on_torque_status)
            self.torque_thread.error_signal.connect(self.on_torque_error)
            
            self.torque_thread.configure(port, baud, freq)
            
            self.torque_connect_btn.setEnabled(False)
            self.torque_disconnect_btn.setEnabled(True)
            self.torque_start_btn.setEnabled(True)
            self.torque_stop_btn.setEnabled(False)
            self.torque_connected = True
            self.torque_status_label.setText('已连接')
            self.torque_port_selector.set_enabled(False)
            
            logger.info(f"已连接到扭矩传感器: {port}, {baud}波特率, {freq}Hz")
            
        except Exception as e:
            ErrorHandler.handle_exception(
                self, e, "连接扭矩传感器失败", ErrorSeverity.ERROR)
            
    def disconnect_torque(self):
        """断开扭矩传感器连接"""
        self.stop_torque_sampling()
        
        self.torque_connect_btn.setEnabled(True)
        self.torque_disconnect_btn.setEnabled(False)
        self.torque_start_btn.setEnabled(False)
        self.torque_stop_btn.setEnabled(False)
        self.torque_connected = False
        self.torque_status_label.setText('未连接')
        self.torque_port_selector.set_enabled(True)
        
        logger.info("已断开扭矩传感器连接")
        
    def on_status_changed(self, ok):
        """处理SafeMRC状态变化"""
        self.connected = ok
        
        if ok:
            # 连接成功，确保参数控件可用（未开始发送时）
            if not self.start_btn.isChecked():
                # 所有参数控件可用
                self.freq_spin.setEnabled(True)
                self.mode_combo.setEnabled(True)
                self.current_spin.setEnabled(True)
                self.id_spin.setEnabled(True)
            else:
                # 正在发送，只禁用频率控件
                self.freq_spin.setEnabled(False)
                self.mode_combo.setEnabled(True)
                self.current_spin.setEnabled(True)
                self.id_spin.setEnabled(True)
        else:
            # 连接断开，禁用控制按钮，启用连接按钮和端口选择
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.start_btn.setEnabled(False)
            self.port_selector.set_enabled(True)
            
            # 重置发送按钮状态
            self.start_btn.setChecked(False)
            self.start_btn.setText('开始发送')
            
            # 确保参数控件可用
            self.freq_spin.setEnabled(True)
            self.mode_combo.setEnabled(True)
            self.current_spin.setEnabled(True)
            self.id_spin.setEnabled(True)
            
    def on_torque_status(self, ok):
        """处理扭矩传感器状态变化"""
        # 采集线程结束时自动切换按钮状态
        if not ok:
            self.torque_stop_btn.setEnabled(False)
            self.torque_start_btn.setEnabled(True)
            
        self.torque_connected = ok
        self.torque_status_label.setText('已连接' if ok else '未连接')
        
        if not ok:
            self.torque_plot.clear()
            self.torque_data_manager.clear()
            
    def on_torque_error(self, msg):
        """处理扭矩传感器错误"""
        ErrorHandler.show_error_dialog(self, "扭矩传感器错误", msg, ErrorSeverity.ERROR)
        
    def on_data_received(self, data):
        """处理SafeMRC数据接收"""
        # 处理批数据
        if 'batch' in data:
            batch = data['batch']
            # 只显示最后一个数据点的信息
            if batch:
                last_data = batch[-1]
                self._update_ui_with_data(last_data)
                
            # 批量添加到数据管理器
            msg_time = time.perf_counter()
            for item in batch:
                if 'encoder' in item and 'velocity' in item and 'current' in item:
                    item_time = item.get('timestamp', msg_time)
                    self.safemrc_data_manager.add_data_point(item_time, {
                        'angle': item['encoder'],
                        'velocity': item['velocity'],
                        'current': item['current']
                    })
                    
                    # 添加数据到记录器
                    self.data_recorder.add_data(item)
            return
            
        # 原有的单数据点处理逻辑
        # 显示TX/RX十六进制帧
        if 'raw_frame' in data and data.get('direction') == 'RX':
            self.hex_log.append_hex_log('RX', data['raw_frame'])
        elif 'raw_frame' in data and data.get('direction') == 'TX':
            self.hex_log.append_hex_log('TX', data['raw_frame'])
            
        # 只处理RX方向的反馈数据
        if data.get('direction') == 'RX' and 'crc_recv' in data:
            # 获取时间戳
            msg_time = data.get('timestamp', time.perf_counter())
            self.last_data_time = msg_time
            
            # 更新反馈显示
            self.feedback_display.update_field('CRC (recv)', f"0x{data['crc_recv']:04X}")
            self.feedback_display.update_field('CRC (calc)', f"0x{data['crc_calc']:04X}")
            self.feedback_display.update_field('Current (A)', f"{data['current']:.3f}")
            self.feedback_display.update_field('Mode', SAFEMRC_MODES.get(data['mode'], str(data['mode'])))
            self.feedback_display.update_field('Encoder Angle (rad)', f"{data['encoder']:.6f}")
            self.feedback_display.update_field('Encoder Velocity (rad/s)', f"{data['velocity']:.6f}")
            self.feedback_display.update_field('Collision', '是' if data['collision'] else '否')
            
            # 添加数据到数据管理器
            self.safemrc_data_manager.add_data_point(msg_time, {
                'angle': data['encoder'],
                'velocity': data['velocity'],
                'current': data['current']
            })
            
            # 添加数据到记录器
            self.data_recorder.add_data(data)
            
    def _update_ui_with_data(self, data):
        """更新UI显示，从on_data_received中提取出来的逻辑"""
        # 显示TX/RX十六进制帧
        if 'raw_frame' in data and data.get('direction') == 'RX':
            self.hex_log.append_hex_log('RX', data['raw_frame'])
        elif 'raw_frame' in data and data.get('direction') == 'TX':
            self.hex_log.append_hex_log('TX', data['raw_frame'])
            
        # 只处理RX方向的反馈数据
        if data.get('direction') == 'RX' and 'crc_recv' in data:
            # 获取时间戳
            msg_time = data.get('timestamp', time.perf_counter())
            self.last_data_time = msg_time
            
            # 更新反馈显示
            self.feedback_display.update_field('CRC (recv)', f"0x{data['crc_recv']:04X}")
            self.feedback_display.update_field('CRC (calc)', f"0x{data['crc_calc']:04X}")
            self.feedback_display.update_field('Current (A)', f"{data['current']:.3f}")
            self.feedback_display.update_field('Mode', SAFEMRC_MODES.get(data['mode'], str(data['mode'])))
            self.feedback_display.update_field('Encoder Angle (rad)', f"{data['encoder']:.6f}")
            self.feedback_display.update_field('Encoder Velocity (rad/s)', f"{data['velocity']:.6f}")
            self.feedback_display.update_field('Collision', '是' if data['collision'] else '否')
            
    def on_torque_data(self, timestamp, torque):
        """处理扭矩传感器数据接收"""
        # 只有采集线程在运行时才更新数据
        if not self.torque_thread.isRunning() or not self.torque_connected:
            return
            
        # 添加数据到数据管理器
        self.torque_data_manager.add_data_point(timestamp, {'torque': torque})
        
        # 添加数据到记录器
        self.data_recorder.add_torque_data(timestamp, torque)
            
    def update_plots(self):
        """更新图表显示"""
        window = self.plot_window_spin.value()
        
        # 根据当前发送频率动态调整绘图更新间隔
        if hasattr(self, 'freq_spin'):
            freq = self.freq_spin.value()
            if freq > 100:
                # 高频模式下降低绘图刷新率
                if self.timer.interval() < 100:
                    self.timer.setInterval(100)
            else:
                # 低频模式下可以提高绘图刷新率
                if self.timer.interval() > 30:
                    self.timer.setInterval(30)
        
        # 更新SafeMRC图表
        t_angle, y_angle = self.safemrc_data_manager.get_plot_data('angle', window)
        t_vel, y_vel = self.safemrc_data_manager.get_plot_data('velocity', window)
        t_curr, y_curr = self.safemrc_data_manager.get_plot_data('current', window)
        
        if len(t_angle) > 0:
            self.plot_widget.set_data(0, t_angle, y_angle)
            self.plot_widget.set_data(1, t_vel, y_vel)
            self.plot_widget.set_data(2, t_curr, y_curr)
            
        # 更新扭矩传感器图表
        t_torque, y_torque = self.torque_data_manager.get_plot_data('torque', window)
        if len(t_torque) > 0:
            self.torque_plot.set_data(t_torque, y_torque)
            
    def closeEvent(self, event):
        """窗口关闭事件处理"""
        # 停止所有线程
        if self.serial_thread.running:
            self.serial_thread.stop()
            
        if self.torque_thread.running:
            self.torque_thread.stop()
            
        logger.info("应用程序已关闭")
        event.accept()
        
    def _update_params(self):
        """更新SafeMRC参数"""
        try:
            # 获取参数
            mode = self.mode_combo.currentData()
            current = self.current_spin.value()
            device_id = self.id_spin.value()
            
            # 如果正在发送，则只更新模式、电流和ID
            if self.start_btn.isChecked():
                logger.info(f"正在更新SafeMRC参数: 模式={mode}, 电流={current}A, ID={device_id}")
                # 保持原来的发送间隔不变
                send_interval = self.serial_thread.send_interval
                self.serial_thread.update_params(send_interval, mode, current, device_id)
                return
                
            # 未发送时，更新所有参数包括频率
            freq = self.freq_spin.value()
            send_interval = 1.0 / freq
            
            logger.info(f"正在更新SafeMRC参数: 频率={freq}Hz, 模式={mode}, 电流={current}A, ID={device_id}")
            
            # 更新通信线程参数
            self.serial_thread.update_params(send_interval, mode, current, device_id)
        except Exception as e:
            ErrorHandler.log_exception(e, "更新SafeMRC参数失败")
            
    def show_serial_error(self, msg):
        """显示串口错误"""
        ErrorHandler.show_error_dialog(self, "串口错误", msg, ErrorSeverity.ERROR)
        
    def toggle_sending(self):
        """切换发送状态"""
        if not self.connected:
            ErrorHandler.show_error_dialog(
                self, "警告", "请先连接串口！", ErrorSeverity.WARNING)
            self.start_btn.setChecked(False)
            return
            
        if self.start_btn.isChecked():
            # 开始发送前先更新一次参数
            try:
                # 临时禁用所有控件，防止在更新参数过程中用户操作
                self.freq_spin.setEnabled(False)
                self.mode_combo.setEnabled(False)
                self.current_spin.setEnabled(False)
                self.id_spin.setEnabled(False)
                
                # 处理所有待处理的事件
                QApplication.processEvents()
                
                self._update_params()
                
                # 开始发送
                self.start_btn.setText('停止发送')
                self.serial_thread.enable_sending(True)
                
                # 只禁用发送频率控件，允许修改模式、ID和电流
                self.freq_spin.setEnabled(False)
                # 保持这些控件可用
                self.mode_combo.setEnabled(True)
                self.current_spin.setEnabled(True)
                self.id_spin.setEnabled(True)
                
                logger.info("已开始发送命令，发送频率已锁定，但可以修改模式、电流和设备ID")
                
            except Exception as e:
                ErrorHandler.log_exception(e, "启动发送前更新参数失败")
                
                # 确保所有控件可用
                self.freq_spin.setEnabled(True)
                self.mode_combo.setEnabled(True)
                self.current_spin.setEnabled(True)
                self.id_spin.setEnabled(True)
                return
        else:
            # 停止发送
            self.start_btn.setText('开始发送')
            self.serial_thread.enable_sending(False)
            
            # 重新启用所有参数控件
            self.freq_spin.setEnabled(True)
            self.mode_combo.setEnabled(True)
            self.current_spin.setEnabled(True)
            self.id_spin.setEnabled(True)
            
            logger.info("已停止发送命令，所有参数可以修改")
            
    def start_torque_sampling(self):
        """开始扭矩传感器采样"""
        if not self.torque_connected:
            return
            
        if not self.torque_thread.isRunning():
            self.torque_thread._stop_event = False
            self.torque_thread.start()
            
        self.torque_start_btn.setEnabled(False)
        self.torque_stop_btn.setEnabled(True)
        
        logger.info("扭矩传感器采样已开始")
        
    def stop_torque_sampling(self):
        """停止扭矩传感器采样"""
        if self.torque_thread.isRunning():
            self.torque_thread.stop()
            
        self.torque_start_btn.setEnabled(True)
        self.torque_stop_btn.setEnabled(False)
        
        logger.info("扭矩传感器采样已停止")
        
    def start_recording(self):
        """开始记录数据"""
        self.data_recorder.start_recording(has_torque=self.torque_connected)
        
    def on_recording_started(self):
        """处理记录开始事件"""
        self.record_start_btn.setEnabled(False)
        self.record_stop_btn.setEnabled(True)
        self.record_status.setText('正在记录...')
        
    def stop_recording(self):
        """停止记录数据"""
        path, _ = QFileDialog.getSaveFileName(self, '保存CSV', '', 'CSV文件 (*.csv)')
        if not path:
            self.data_recorder.recording = False
            self.record_start_btn.setEnabled(True)
            self.record_stop_btn.setEnabled(False)
            self.record_status.setText('未记录')
            return
            
        self.data_recorder.stop_recording(path)
        
    def on_recording_stopped(self, file_path):
        """处理记录停止事件"""
        self.record_start_btn.setEnabled(True)
        self.record_stop_btn.setEnabled(False)
        self.record_status.setText('未记录')
        
        if file_path:
            ErrorHandler.show_error_dialog(
                self, "已保存", f"数据已保存到 {file_path}", ErrorSeverity.INFO)
        else:
            ErrorHandler.show_error_dialog(
                self, "保存失败", "保存数据失败", ErrorSeverity.WARNING)
            
    def clear_plots(self):
        """清空图表"""
        self.safemrc_data_manager.clear()
        self.torque_data_manager.clear()
        self.plot_widget.clear()
        self.torque_plot.clear()
        
        logger.info("图表已清空")


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_()) 