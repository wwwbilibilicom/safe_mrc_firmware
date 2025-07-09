"""
SafeMRC通信模块，提供与SafeMRC设备的通信功能
"""
import threading
import time
import serial
import serial.tools.list_ports
import sys
import os
from PyQt5 import QtCore
from error_handler import ErrorHandler, ErrorSeverity, logger
from config import SAFEMRC_HEADER, RX_POLL_INTERVAL

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
    """
    计算CRC-CCITT校验值
    
    Args:
        data: 需要计算CRC的字节数据
        crc: 初始CRC值，默认为0xFFFF
        
    Returns:
        int: 计算得到的CRC校验值
    """
    for b in data:
        crc = ((crc >> 8) ^ CRC_CCITT_TABLE[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc

class SerialThread(QtCore.QThread):
    """
    SafeMRC串口通信线程类
    
    使用Qt的事件驱动模型实现与SafeMRC设备的异步通信，
    避免阻塞主线程，提高UI响应性能
    """
    # 信号定义
    data_received = QtCore.pyqtSignal(dict)
    status_changed = QtCore.pyqtSignal(bool)
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        """初始化SafeMRC通信线程"""
        super().__init__()
        
        # 通信参数
        self.ser = None
        self.running = False
        self.send_interval = 0.05  # default 20Hz
        self.mode = 0
        self.current = 0.0
        self.device_id = 1
        
        # 线程同步
        self.lock = QtCore.QMutex()
        self._stop_event = threading.Event()
        self._sending = False
        
        # 数据缓冲
        self._last_cmd = b''
        self._last_send_time = 0
        self._buffer = b''
        
        # 批处理数据
        self._data_buffer = []
        self._last_emit_time = 0
        self._emit_interval = 0.05  # 50毫秒发送一次数据
        
        # 发送定时器
        self._tx_timer = QtCore.QTimer()
        self._tx_timer.moveToThread(self)
        self._tx_timer.timeout.connect(self._send_command)

        # 接收定时器
        self._rx_timer = QtCore.QTimer()
        self._rx_timer.moveToThread(self)
        self._rx_timer.timeout.connect(self._poll_read)

    def configure(self, port, baudrate, send_interval, mode, current, device_id):
        """
        配置SafeMRC通信参数
        
        Args:
            port: 串口名称
            baudrate: 波特率
            send_interval: 发送间隔 (秒)
            mode: 工作模式
            current: 电流值 (A)
            device_id: 设备ID
        """
        with QtCore.QMutexLocker(self.lock):
            self.port = port
            self.baudrate = baudrate
            self.send_interval = send_interval
            self.mode = mode
            self.current = current
            self.device_id = device_id
            
        logger.info(f"SafeMRC配置: 端口={port}, 波特率={baudrate}, "
                   f"频率={1/send_interval:.1f}Hz, 模式={mode}, "
                   f"电流={current}A, ID={device_id}")

    def run(self):
        """线程主函数，启动事件循环和定时器"""
        try:
            # 打开串口
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            self.status_changed.emit(True)
            logger.info(f"SafeMRC串口已打开: {self.port}")
        except Exception as e:
            error_msg = f"无法打开串口 {self.port} (波特率: {self.baudrate})"
            ErrorHandler.log_exception(e, error_msg)
            self.error_signal.emit(f"{error_msg}:\n{str(e)}")
            self.status_changed.emit(False)
            return
        
        # 启动定时器
        interval_ms = int(self.send_interval * 1000)
        self._tx_timer.start(interval_ms)
        self._rx_timer.start(RX_POLL_INTERVAL)
        logger.info(f"SafeMRC通信定时器已启动: 发送间隔={interval_ms}ms, 接收轮询={RX_POLL_INTERVAL}ms")

        # 进入Qt事件循环
        self.exec_()

        # 退出时清理资源
        self._cleanup()
        
    # 槽函数：更新定时器间隔
    @QtCore.pyqtSlot(int)
    def _update_timer_interval(self, interval_ms):
        """
        更新发送定时器间隔（必须在定时器所在线程中调用）
        
        Args:
            interval_ms: 定时器间隔（毫秒）
        """
        if self._tx_timer.isActive():
            self._tx_timer.setInterval(interval_ms)
            logger.info(f"SafeMRC发送间隔已更新: {interval_ms}ms ({1000/interval_ms:.1f}Hz)")
            
    # 槽函数：重置定时器
    @QtCore.pyqtSlot(int)
    def _reset_timer(self, interval_ms):
        """
        完全重置定时器（停止并重新启动）
        
        Args:
            interval_ms: 新的定时器间隔（毫秒）
        """
        # 安全地停止定时器
        if self._tx_timer.isActive():
            self._tx_timer.stop()
            
        # 重新启动定时器
        self._tx_timer.start(interval_ms)
        logger.info(f"SafeMRC发送定时器已重置: {interval_ms}ms ({1000/interval_ms:.1f}Hz)")

    def _cleanup(self):
        """清理资源并通知状态变化"""
        # 停止定时器
        if hasattr(self, '_tx_timer') and self._tx_timer.isActive():
            self._tx_timer.stop()
            
        if hasattr(self, '_rx_timer') and self._rx_timer.isActive():
            self._rx_timer.stop()
            
        # 关闭串口
        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
                    logger.info("SafeMRC串口已关闭")
            except Exception as e:
                ErrorHandler.log_exception(e, "关闭SafeMRC串口时出错", ErrorSeverity.WARNING)
                
        # 更新状态
        self.running = False
        self.status_changed.emit(False)

    def _send_command(self):
        """定时器触发的发送命令函数"""
        if not self._sending or not self.ser or not self.ser.is_open:
            return
        
        try:
            with QtCore.QMutexLocker(self.lock):
                cmd = self.pack_cmd(self.mode, self.current)
                self._last_cmd = cmd
            
            self.ser.write(cmd)
            self._last_send_time = time.perf_counter()
            
            # 发送TX信号
            self.data_received.emit({'raw_frame': cmd, 'direction': 'TX'})
        except Exception as e:
            ErrorHandler.log_exception(e, "发送SafeMRC命令失败", ErrorSeverity.WARNING)

    def _process_buffer(self):
        """处理接收缓冲区数据"""
        # 帧长度检查
        FRAME_LENGTH = 17  # SafeMRC帧固定长度
        
        while len(self._buffer) >= FRAME_LENGTH:
            # 查找帧头
            idx = self._buffer.find(SAFEMRC_HEADER)
            if idx == -1:
                # 没找到帧头，清空缓冲区
                self._buffer = b''
                break
            
            if len(self._buffer) - idx < FRAME_LENGTH:
                # 帧不完整，保留剩余数据等待下次处理
                self._buffer = self._buffer[idx:]
                break
            
            # 提取完整帧
            frame = self._buffer[idx:idx+FRAME_LENGTH]
            self._buffer = self._buffer[idx+FRAME_LENGTH:]  # 移除已处理的帧
            
            # 解析帧数据
            data = self.parse_feedback(frame)
            if data:
                # 添加元数据
                data['raw_frame'] = frame
                data['direction'] = 'RX'
                data['timestamp'] = time.perf_counter()  # 高精度时间戳
                
                # 添加到批处理缓冲区
                self._data_buffer.append(data)
                
                # 检查是否需要发送
                current_time = time.perf_counter()
                if current_time - self._last_emit_time >= self._emit_interval:
                    self._emit_data_buffer()
                    self._last_emit_time = current_time

    def _poll_read(self):
        """定时器触发的数据接收函数"""
        if not self.ser or not self.ser.is_open:
            return
            
        try:
            if self.ser.in_waiting:
                new_data = self.ser.read(self.ser.in_waiting)
                if new_data:
                    self._buffer += new_data
                    self._process_buffer()
        except Exception as e:
            ErrorHandler.log_exception(e, "读取SafeMRC数据失败", ErrorSeverity.WARNING)

    def stop(self):
        """安全停止线程"""
        logger.info("正在停止SafeMRC通信线程...")
        self._stop_event.set()
        # 立即停止发送
        self._sending = False
        
        # 发送剩余的数据缓冲区
        self._emit_data_buffer()
        
        # 退出事件循环
        QtCore.QMetaObject.invokeMethod(self, 'quit')
        # 等待线程结束
        if not self.wait(1000):
            logger.warning("SafeMRC通信线程未能在1秒内停止")
        else:
            logger.info("SafeMRC通信线程已停止")

    def update_params(self, send_interval, mode, current, device_id):
        """
        更新通信参数
        
        Args:
            send_interval: 发送间隔 (秒)
            mode: 工作模式
            current: 电流值 (A)
            device_id: 设备ID
        """
        with QtCore.QMutexLocker(self.lock):
            # 更新参数
            old_interval = self.send_interval
            self.send_interval = send_interval
            self.mode = mode
            self.current = current
            self.device_id = device_id
            
            # 如果发送间隔变化了，需要更新定时器
            if old_interval != send_interval:
                interval_ms = round(send_interval * 1000)
                
                # 使用invokeMethod安全地跨线程重置定时器
                # 这里使用_reset_timer而不是_update_timer_interval
                QtCore.QMetaObject.invokeMethod(
                    self, 
                    "_reset_timer", 
                    QtCore.Qt.QueuedConnection,
                    QtCore.Q_ARG(int, interval_ms)
                )
                logger.info(f"已请求重置发送定时器: {interval_ms}ms ({1000/interval_ms:.1f}Hz)")

    def pack_cmd(self, mode, current):
        """
        打包SafeMRC命令帧
        
        Args:
            mode: 工作模式
            current: 电流值 (A)
            
        Returns:
            bytes: 打包后的命令帧
        """
        # Command protocol: 0xFE 0xEE id(1) mode(1) current(int32, 4 bytes) CRC(2)
        head = SAFEMRC_HEADER
        id_byte = self.device_id.to_bytes(1, 'little')
        mode_byte = mode.to_bytes(1, 'little')
        cur = int(current * 1000)  # 转换为mA
        cur_bytes = cur.to_bytes(4, 'little', signed=True)  # int32_t, little-endian
        payload = head + id_byte + mode_byte + cur_bytes
        crc = crc_ccitt(payload)
        crc_bytes = crc.to_bytes(2, 'little')  # little-endian for CRC, to match embedded
        return payload + crc_bytes

    def parse_feedback(self, frame):
        """
        解析SafeMRC反馈帧
        
        Args:
            frame: 接收到的帧数据
            
        Returns:
            dict: 解析后的数据字典，解析失败时返回None
        """
        # Feedback: 0xFE 0xEE id(1) mode(1) collision(1) encoder(4) velocity(4) current(2) CRC(2)
        try:
            # 帧头检查
            if len(frame) < 17 or frame[0] != 0xFE or frame[1] != 0xEE:
                return None
                
            # 提取字段
            id_ = frame[2]
            mode = frame[3]
            collision = frame[4]
            encoder = int.from_bytes(frame[5:9], 'little', signed=True) / 65535.0
            velocity = int.from_bytes(frame[9:13], 'little', signed=True) / 1000.0
            current = int.from_bytes(frame[13:15], 'little', signed=True) / 1000.0
            
            # CRC校验
            crc_recv = int.from_bytes(frame[15:17], 'little')
            crc_calc = crc_ccitt(frame[:15])
            
            # 返回解析结果
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
        except Exception as e:
            ErrorHandler.log_exception(e, "解析SafeMRC反馈帧失败", ErrorSeverity.WARNING)
            return None

    def enable_sending(self, enable):
        """
        启用或禁用命令发送
        
        Args:
            enable: 是否启用发送
        """
        with QtCore.QMutexLocker(self.lock):
            was_sending = self._sending
            self._sending = enable
            if not was_sending and enable:
                logger.info("SafeMRC命令发送已启用")
            elif was_sending and not enable:
                logger.info("SafeMRC命令发送已禁用")
    
    def _emit_data_buffer(self):
        """发送批处理数据缓冲区中的数据"""
        if not self._data_buffer:
            return
            
        # 克隆缓冲区并发送
        buffer_to_send = self._data_buffer.copy()
        self._data_buffer = []
        
        # 批量发送
        self.data_received.emit({'batch': buffer_to_send})
    
    @staticmethod
    def get_available_ports():
        """
        获取可用串口列表
        
        Returns:
            list: 可用串口名称列表
        """
        try:
            logger.info("开始获取可用串口列表...")
            
            # 检查操作系统类型
            if sys.platform.startswith('win'):
                logger.info(f"当前操作系统: Windows ({sys.platform})")
                # Windows系统下，尝试直接枚举COM端口
                ports = []
                # 首先尝试使用serial.tools.list_ports.comports()
                comports = list(serial.tools.list_ports.comports())
                logger.info(f"通过pyserial找到 {len(comports)} 个串口")
                
                if comports:
                    for port in comports:
                        ports.append(port.device)
                        logger.info(f"找到串口: {port.device} ({port.description})")
                else:
                    # 如果没有找到，尝试手动枚举常见COM端口
                    logger.info("未找到串口，尝试手动枚举COM端口...")
                    for i in range(1, 21):  # 检查COM1到COM20
                        port_name = f"COM{i}"
                        try:
                            s = serial.Serial(port_name)
                            s.close()
                            ports.append(port_name)
                            logger.info(f"手动检测到可用串口: {port_name}")
                        except (OSError, serial.SerialException):
                            pass
                
                if not ports:
                    logger.warning("未找到任何可用串口")
                return ports
            else:
                # 其他操作系统使用标准方法
                logger.info(f"当前操作系统: {sys.platform}")
                comports = list(serial.tools.list_ports.comports())
                ports = [p.device for p in comports]
                logger.info(f"找到 {len(ports)} 个串口: {', '.join(ports)}")
                return ports
        except Exception as e:
            logger.error(f"获取可用串口列表失败: {str(e)}")
            return [] 