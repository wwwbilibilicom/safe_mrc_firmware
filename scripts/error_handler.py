"""
错误处理模块，提供统一的异常处理机制
"""
import logging
import traceback
from enum import Enum
from PyQt5 import QtWidgets

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("safemrc_app.log"),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger("SafeMRC")

class ErrorSeverity(Enum):
    """错误严重程度枚举"""
    INFO = 0
    WARNING = 1
    ERROR = 2
    CRITICAL = 3

class ErrorHandler:
    """统一错误处理类"""
    
    @staticmethod
    def log_exception(e, context="", severity=ErrorSeverity.ERROR):
        """记录异常到日志"""
        error_msg = f"{context}: {str(e)}"
        if severity == ErrorSeverity.INFO:
            logger.info(error_msg)
        elif severity == ErrorSeverity.WARNING:
            logger.warning(error_msg)
        elif severity == ErrorSeverity.ERROR:
            logger.error(error_msg)
            logger.error(traceback.format_exc())
        elif severity == ErrorSeverity.CRITICAL:
            logger.critical(error_msg)
            logger.critical(traceback.format_exc())
    
    @staticmethod
    def show_error_dialog(parent, title, message, severity=ErrorSeverity.ERROR):
        """显示错误对话框"""
        if severity == ErrorSeverity.INFO:
            QtWidgets.QMessageBox.information(parent, title, message)
        elif severity == ErrorSeverity.WARNING:
            QtWidgets.QMessageBox.warning(parent, title, message)
        elif severity == ErrorSeverity.ERROR:
            QtWidgets.QMessageBox.critical(parent, title, message)
        elif severity == ErrorSeverity.CRITICAL:
            QtWidgets.QMessageBox.critical(parent, title, message)
    
    @staticmethod
    def handle_exception(parent, e, context="", severity=ErrorSeverity.ERROR, show_dialog=True):
        """处理异常：记录日志并可选显示对话框"""
        ErrorHandler.log_exception(e, context, severity)
        if show_dialog:
            # 对用户友好的错误信息，隐藏技术细节
            user_msg = f"{context}出现错误。\n\n"
            if severity == ErrorSeverity.CRITICAL:
                user_msg += f"错误详情: {str(e)}\n\n请联系技术支持。"
            else:
                user_msg += "请检查设备连接和设置后重试。"
            
            ErrorHandler.show_error_dialog(parent, "操作错误", user_msg, severity)
        
        return False  # 返回False表示处理失败 