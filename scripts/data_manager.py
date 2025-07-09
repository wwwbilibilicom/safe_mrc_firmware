"""
数据管理模块，提供数据处理和管理功能
"""
import numpy as np
import csv
import time
from PyQt5 import QtCore
from error_handler import ErrorHandler, ErrorSeverity, logger
from config import MAX_PLOT_POINTS

class RingBuffer:
    """环形缓冲区，用于存储固定大小的数据"""
    
    def __init__(self, max_size, dtype=float):
        """
        初始化环形缓冲区
        
        Args:
            max_size: 缓冲区最大大小
            dtype: 数据类型，默认为float
        """
        self.max_size = max_size
        self.data = np.zeros(max_size, dtype=dtype)
        self.ptr = 0
        self.count = 0
        
    def append(self, value):
        """
        添加数据到缓冲区
        
        Args:
            value: 要添加的数据
        """
        self.data[self.ptr] = value
        self.ptr = (self.ptr + 1) % self.max_size
        self.count = min(self.count + 1, self.max_size)
        
    def get_array(self):
        """
        获取缓冲区中的数据数组
        
        Returns:
            numpy.ndarray: 缓冲区数据数组
        """
        if self.count < self.max_size:
            return self.data[:self.count]
        else:
            return np.concatenate((self.data[self.ptr:], self.data[:self.ptr]))
            
    def clear(self):
        """清空缓冲区"""
        self.ptr = 0
        self.count = 0
        self.data.fill(0)


class DataRecorder(QtCore.QObject):
    """数据记录器，用于记录数据到CSV文件"""
    
    recording_started = QtCore.pyqtSignal()
    recording_stopped = QtCore.pyqtSignal(str)  # 发送保存的文件路径
    
    def __init__(self, parent=None):
        """初始化数据记录器"""
        super().__init__(parent)
        self.recording = False
        self.record_data = []
        self.record_start_time = None
        self.file_path = None
        self.has_torque = False
        
    def start_recording(self, has_torque=False):
        """
        开始记录数据
        
        Args:
            has_torque: 是否包含扭矩数据
        """
        if self.recording:
            return
            
        self.recording = True
        self.record_data = []
        self.record_start_time = None
        self.has_torque = has_torque
        self.recording_started.emit()
        logger.info("数据记录已开始")
        
    def add_data(self, data):
        """
        添加SafeMRC数据
        
        Args:
            data: SafeMRC数据字典
        """
        if not self.recording:
            return
            
        # 设置记录开始时间
        if self.record_start_time is None:
            self.record_start_time = data.get('timestamp', time.perf_counter())
            
        # 计算相对时间戳
        timestamp = data.get('timestamp', time.perf_counter()) - self.record_start_time
        
        # 创建记录行
        record_row = {
            'timestamp': timestamp,
            'encoder': data.get('encoder'),
            'velocity': data.get('velocity'),
            'current': data.get('current'),
            'mode': data.get('mode'),
            'collision': data.get('collision'),
            'crc_recv': data.get('crc_recv'),
            'crc_calc': data.get('crc_calc')
        }
        
        # 添加原始帧数据
        if 'raw_frame' in data:
            record_row['raw_frame_hex'] = ' '.join(f'{b:02X}' for b in data['raw_frame'])
            
        # 添加到记录列表
        self.record_data.append(record_row)
        
    def add_torque_data(self, timestamp, torque):
        """
        添加扭矩数据
        
        Args:
            timestamp: 时间戳
            torque: 扭矩值
        """
        if not self.recording or not self.has_torque:
            return
            
        # 设置记录开始时间
        if self.record_start_time is None:
            self.record_start_time = timestamp
            
        # 计算相对时间戳
        rel_timestamp = timestamp - self.record_start_time
        
        # 查找最近的记录行
        if self.record_data:
            latest = self.record_data[-1]
            if abs(latest['timestamp'] - rel_timestamp) < 0.01:
                # 时间戳相近，直接添加到最近的记录行
                latest['torque'] = torque
                return
                
        # 创建新的只包含扭矩的记录行
        record_row = {
            'timestamp': rel_timestamp,
            'torque': torque
        }
        self.record_data.append(record_row)
        
    def stop_recording(self, file_path):
        """
        停止记录并保存数据
        
        Args:
            file_path: 保存文件路径
            
        Returns:
            bool: 是否保存成功
        """
        if not self.recording:
            return False
            
        self.recording = False
        self.file_path = file_path
        
        if not self.record_data:
            logger.warning("没有数据可保存")
            self.recording_stopped.emit("")
            return False
            
        try:
            # 确定字段名称
            if self.has_torque:
                fieldnames = [
                    'timestamp', 'encoder', 'velocity', 'current', 
                    'mode', 'collision', 'crc_recv', 'crc_calc', 
                    'raw_frame_hex', 'torque'
                ]
            else:
                fieldnames = [
                    'timestamp', 'encoder', 'velocity', 'current', 
                    'mode', 'collision', 'crc_recv', 'crc_calc', 
                    'raw_frame_hex'
                ]
                
            # 写入CSV文件
            with open(file_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                
                for row in self.record_data:
                    # 确保所有字段都有值
                    for field in fieldnames:
                        if field not in row:
                            row[field] = None
                            
                    writer.writerow(row)
                    
            logger.info(f"数据已保存到: {file_path}")
            self.recording_stopped.emit(file_path)
            return True
            
        except Exception as e:
            ErrorHandler.log_exception(e, f"保存数据到 {file_path} 失败")
            self.recording_stopped.emit("")
            return False
            
    def is_recording(self):
        """
        检查是否正在记录
        
        Returns:
            bool: 是否正在记录
        """
        return self.recording
        
    def get_record_count(self):
        """
        获取记录数量
        
        Returns:
            int: 记录数量
        """
        return len(self.record_data)


class PlotDataManager:
    """绘图数据管理器，管理绘图数据的存储和处理"""
    
    def __init__(self, max_points=MAX_PLOT_POINTS):
        """
        初始化绘图数据管理器
        
        Args:
            max_points: 最大数据点数量
        """
        self.max_points = max_points
        self.time_buffer = RingBuffer(max_points)
        self.data_buffers = {}
        self.plot_time_zero = None
        
    def add_data_channel(self, name):
        """
        添加数据通道
        
        Args:
            name: 通道名称
        """
        if name not in self.data_buffers:
            self.data_buffers[name] = RingBuffer(self.max_points)
            
    def add_data_point(self, timestamp, data_dict):
        """
        添加数据点
        
        Args:
            timestamp: 时间戳
            data_dict: 数据字典，键为通道名称，值为数据值
        """
        # 设置绘图时间零点
        if self.plot_time_zero is None:
            self.plot_time_zero = timestamp
            
        # 添加时间戳
        self.time_buffer.append(timestamp)
        
        # 添加各通道数据
        for name, value in data_dict.items():
            if name not in self.data_buffers:
                self.add_data_channel(name)
                
            self.data_buffers[name].append(value)
            
    def get_plot_data(self, channel, window=None):
        """
        获取指定通道的绘图数据
        
        Args:
            channel: 通道名称
            window: 时间窗口大小（秒），如果为None则返回所有数据
            
        Returns:
            tuple: (x, y) 时间和数据数组
        """
        if channel not in self.data_buffers:
            return [], []
            
        # 获取时间和数据数组
        t_arr = self.time_buffer.get_array()
        y_arr = self.data_buffers[channel].get_array()
        
        if len(t_arr) == 0:
            return [], []
            
        # 转换为相对时间
        if self.plot_time_zero is not None:
            t_arr = t_arr - self.plot_time_zero
            
        # 应用时间窗口
        if window is not None and len(t_arr) > 0:
            t_now = t_arr[-1]
            mask = t_arr > (t_now - window)
            t_arr = t_arr[mask]
            y_arr = y_arr[mask]
            
        return t_arr, y_arr
        
    def clear(self):
        """清空所有数据"""
        self.time_buffer.clear()
        for buffer in self.data_buffers.values():
            buffer.clear()
        self.plot_time_zero = None 