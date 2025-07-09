"""
UI组件模块，提供可重用的UI组件类
"""
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from PyQt5.QtGui import QTextCursor
from config import PLOT_TITLES

class HexLogWidget(QtWidgets.QTextEdit):
    """十六进制日志显示组件"""
    
    def __init__(self, parent=None, max_height=80):
        """初始化十六进制日志显示组件"""
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumHeight(max_height)
        
    def append_hex_log(self, direction, frame_bytes):
        """
        添加十六进制日志
        
        Args:
            direction: 方向，'TX'或'RX'
            frame_bytes: 帧数据字节
        """
        hex_str = ' '.join(f'{b:02X}' for b in frame_bytes)
        self.append(f'<b>{direction}:</b> {hex_str}')
        self.moveCursor(QTextCursor.End)


class PlotWidget:
    """绘图组件，封装pyqtgraph绘图功能"""
    
    def __init__(self, title, color='b', width=2, parent=None):
        """
        初始化绘图组件
        
        Args:
            title: 图表标题
            color: 曲线颜色，默认为蓝色
            width: 曲线宽度，默认为2
            parent: 父组件
        """
        self.plot_widget = pg.PlotWidget(parent)
        self.plot_widget.setTitle(title)
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('bottom', 'Time (s)')
        self.curve = self.plot_widget.plot([], [], pen=pg.mkPen(color, width=width))
        
    def set_data(self, x, y):
        """
        设置绘图数据
        
        Args:
            x: x轴数据
            y: y轴数据
        """
        self.curve.setData(x, y)
        
    def clear(self):
        """清空绘图数据"""
        self.curve.setData([], [])
        
    def get_widget(self):
        """获取绘图控件"""
        return self.plot_widget


class SerialPortSelector(QtWidgets.QWidget):
    """串口选择组件"""
    
    port_changed = QtCore.pyqtSignal(str)
    
    def __init__(self, parent=None, label_text="Serial Port:"):
        """初始化串口选择组件"""
        super().__init__(parent)
        
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 添加标签
        self.label = QtWidgets.QLabel(label_text)
        layout.addWidget(self.label)
        
        # 添加串口下拉框
        self.port_combo = QtWidgets.QComboBox()
        layout.addWidget(self.port_combo)
        
        # 添加刷新按钮
        self.refresh_btn = QtWidgets.QPushButton('刷新')
        self.refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(self.refresh_btn)
        
        # 连接信号
        self.port_combo.currentTextChanged.connect(self.port_changed)
        
    def refresh_ports(self, ports=None):
        """
        刷新可用串口列表
        
        Args:
            ports: 可用串口列表，如果为None则自动获取
        """
        current = self.port_combo.currentText()
        self.port_combo.clear()
        
        if ports is None:
            # 导入这里以避免循环导入
            from safemrc_comm import SerialThread
            ports = SerialThread.get_available_ports()
            
        for p in ports:
            self.port_combo.addItem(p)
            
        # 尝试恢复之前的选择
        index = self.port_combo.findText(current)
        if index >= 0:
            self.port_combo.setCurrentIndex(index)
            
    def get_current_port(self):
        """获取当前选择的串口"""
        return self.port_combo.currentText()
        
    def set_enabled(self, enabled):
        """设置组件是否启用"""
        self.port_combo.setEnabled(enabled)
        self.refresh_btn.setEnabled(enabled)


class FeedbackDisplay(QtWidgets.QGroupBox):
    """反馈数据显示组件"""
    
    def __init__(self, title="反馈数据", parent=None):
        """初始化反馈数据显示组件"""
        super().__init__(title, parent)
        
        self.layout = QtWidgets.QGridLayout(self)
        self.labels = {}
        
    def add_field(self, name, row=None):
        """
        添加显示字段
        
        Args:
            name: 字段名称
            row: 行号，如果为None则添加到最后
        """
        label = QtWidgets.QLabel('--')
        
        if row is None:
            row = self.layout.rowCount()
            
        self.layout.addWidget(QtWidgets.QLabel(name+':'), row, 0)
        self.layout.addWidget(label, row, 1)
        self.labels[name] = label
        
    def update_field(self, name, value):
        """
        更新字段值
        
        Args:
            name: 字段名称
            value: 字段值
        """
        if name in self.labels:
            self.labels[name].setText(str(value))
            
    def clear(self):
        """清空所有字段值"""
        for label in self.labels.values():
            label.setText('--')


class MultiPlotWidget(QtWidgets.QWidget):
    """多图表显示组件"""
    
    def __init__(self, parent=None, titles=None, colors=None):
        """
        初始化多图表显示组件
        
        Args:
            parent: 父组件
            titles: 图表标题列表
            colors: 曲线颜色列表
        """
        super().__init__(parent)
        
        if titles is None:
            titles = PLOT_TITLES[:3]  # 默认使用前三个标题
            
        if colors is None:
            colors = ['b', 'g', 'r']  # 默认颜色：蓝、绿、红
            
        # 创建布局
        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建绘图组件
        self.plot_widgets = []
        self.plot_curves = []
        
        for i, title in enumerate(titles):
            color = colors[i % len(colors)]
            plot = PlotWidget(title, color)
            self.plot_widgets.append(plot)
            self.plot_curves.append(plot.curve)
            self.layout.addWidget(plot.get_widget())
            
    def set_data(self, index, x, y):
        """
        设置指定图表的数据
        
        Args:
            index: 图表索引
            x: x轴数据
            y: y轴数据
        """
        if 0 <= index < len(self.plot_widgets):
            self.plot_widgets[index].set_data(x, y)
            
    def clear(self):
        """清空所有图表数据"""
        for plot in self.plot_widgets:
            plot.clear()
            
    def set_window(self, seconds):
        """
        设置显示窗口大小
        
        Args:
            seconds: 窗口大小（秒）
        """
        for plot in self.plot_widgets:
            plot.get_widget().setXRange(-seconds, 0) 