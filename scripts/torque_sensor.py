"""
扭矩传感器基础类，提供与扭矩传感器通信的基本功能
"""
import serial
import time
import threading
from error_handler import ErrorHandler, ErrorSeverity, logger
from config import TORQUE_COMMAND, TORQUE_CONVERSION_FACTOR

class TorqueSensor:
    """扭矩传感器类，实现与RS-485扭矩传感器的通信"""
    
    def __init__(self):
        """初始化扭矩传感器对象"""
        self.serial = None
        self.initialized = False
        self.lock = threading.Lock()
        self.port_name = None
        self.baudrate = None
        
    def __enter__(self):
        """上下文管理器入口"""
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出，确保资源释放"""
        self.close()
        return False  # 不抑制异常
        
    def initialize(self, port_name, baudrate):
        """
        初始化扭矩传感器连接
        
        Args:
            port_name: 串口名称
            baudrate: 波特率
            
        Returns:
            bool: 初始化是否成功
        """
        try:
            # 如果已经初始化，先关闭之前的连接
            if self.initialized:
                self.close()
                
            self.port_name = port_name
            self.baudrate = baudrate
            
            self.serial = serial.Serial()
            self.serial.port = port_name
            self.serial.baudrate = baudrate
            self.serial.timeout = 0.5  # 500ms timeout
            self.serial.open()
            
            if not self.serial.is_open:
                logger.error(f"无法打开RS485端口: {port_name}")
                return False
                
            # 清空缓冲区
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.1)
            
            self.initialized = True
            logger.info(f"扭矩传感器在端口 {port_name} 上初始化成功")
            return True
            
        except serial.SerialException as e:
            ErrorHandler.log_exception(e, f"初始化扭矩传感器失败 (端口:{port_name}, 波特率:{baudrate})")
            return False

    def read_torque(self):
        """
        读取当前扭矩值
        
        Returns:
            float: 扭矩值 (Nm)，失败时返回0.0
        """
        if not self.initialized or not self.serial or not self.serial.is_open:
            logger.warning("扭矩传感器未初始化或端口已关闭")
            return 0.0
            
        try:
            with self.lock:
                # 清空接收缓冲区
                self.serial.reset_input_buffer()
                
                # 发送读取命令
                bytes_written = self.serial.write(TORQUE_COMMAND)
                if bytes_written != len(TORQUE_COMMAND):
                    logger.warning(f"命令写入不完整: 已写入 {bytes_written}/{len(TORQUE_COMMAND)} 字节")
                    return 0.0
                    
                # 等待响应
                time.sleep(0.01)  # 缩短等待时间，提高性能
                
                # 检查可用数据
                available = self.serial.in_waiting
                if available < 8:  # 最小响应长度
                    logger.debug(f"可用数据不足: {available} 字节")
                    return 0.0
                    
                # 读取响应
                buffer = self.serial.read(available)
                
                # 解析响应
                return self._parse_response(buffer)
                
        except serial.SerialException as e:
            ErrorHandler.log_exception(e, "读取扭矩传感器时出错", ErrorSeverity.WARNING)
            return 0.0

    def _parse_response(self, buffer):
        """
        解析传感器响应数据
        
        Args:
            buffer: 接收到的字节数据
            
        Returns:
            float: 解析出的扭矩值 (Nm)，失败时返回0.0
        """
        # 查找帧头并解析数据
        for i in range(len(buffer) - 6):
            if buffer[i] == 0x01 and buffer[i + 1] == 0x03:
                # 提取数据字段
                data_high = buffer[i + 5]
                data_low = buffer[i + 6]
                u_data = (data_high << 8) | data_low
                
                # 转换为扭矩值
                return self.hex2dec(u_data) / TORQUE_CONVERSION_FACTOR
                
        logger.debug("响应中未找到有效帧头")
        return 0.0

    def is_initialized(self):
        """
        检查传感器是否已初始化
        
        Returns:
            bool: 是否已初始化
        """
        return self.initialized and self.serial and self.serial.is_open

    def close(self):
        """关闭传感器连接并释放资源"""
        if self.serial:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except Exception as e:
                ErrorHandler.log_exception(e, "关闭扭矩传感器连接时出错", ErrorSeverity.WARNING)
                
        self.initialized = False
        logger.info("扭矩传感器连接已关闭")

    @staticmethod
    def hex2dec(hex_data):
        """
        将16位无符号整数转换为有符号整数
        
        Args:
            hex_data: 16位无符号整数
            
        Returns:
            int: 转换后的有符号整数
        """
        # 确保输入是16位整数
        hex_data &= 0xFFFF
        
        # 转换为有符号整数
        if hex_data & 0x8000:
            # 负数
            return -((~hex_data & 0xFFFF) + 1)
        else:
            # 正数
            return hex_data 