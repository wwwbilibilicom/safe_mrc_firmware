#!/usr/bin/env python
"""
测试运行脚本，用于运行所有测试
"""
import unittest
import sys
import os

def run_tests():
    """运行所有测试"""
    # 获取测试目录
    test_dir = os.path.join(os.path.dirname(__file__), 'tests')
    
    # 如果测试目录不存在，创建它
    if not os.path.exists(test_dir):
        os.makedirs(test_dir)
        print(f"创建测试目录: {test_dir}")
    
    # 发现所有测试
    loader = unittest.TestLoader()
    suite = loader.discover(test_dir)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 返回测试结果
    return result.wasSuccessful()

if __name__ == '__main__':
    # 运行测试
    success = run_tests()
    
    # 根据测试结果设置退出码
    sys.exit(0 if success else 1) 