#!/usr/bin/env python
"""
SafeMRC应用程序启动脚本
"""
import sys
import os
import argparse
from PyQt5 import QtWidgets

def main():
    """主函数，解析命令行参数并启动应用程序"""
    parser = argparse.ArgumentParser(description='SafeMRC上位机应用程序')
    parser.add_argument('--legacy', action='store_true', 
                      help='使用原始版本的应用程序')
    parser.add_argument('--debug', action='store_true',
                      help='启用调试模式')
    args = parser.parse_args()
    
    # 设置环境变量
    if args.debug:
        os.environ['SAFEMRC_DEBUG'] = '1'
    
    # 创建应用程序
    app = QtWidgets.QApplication(sys.argv)
    
    # 根据参数选择启动哪个版本
    if args.legacy:
        from main import MainWindow
        print("启动原始版本的应用程序...")
    else:
        from main_refactored import MainWindow
        print("启动重构版本的应用程序...")
    
    # 创建并显示主窗口
    win = MainWindow()
    win.show()
    
    # 运行应用程序
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 