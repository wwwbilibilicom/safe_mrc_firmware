#!/usr/bin/env python
"""
数据管理器单元测试
"""
import sys
import os
import unittest
import numpy as np
import time

# 添加父目录到路径，以便导入模块
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from data_manager import RingBuffer, PlotDataManager

class TestRingBuffer(unittest.TestCase):
    """环形缓冲区测试类"""
    
    def test_init(self):
        """测试初始化"""
        rb = RingBuffer(10)
        self.assertEqual(rb.max_size, 10)
        self.assertEqual(rb.ptr, 0)
        self.assertEqual(rb.count, 0)
        self.assertEqual(len(rb.data), 10)
    
    def test_append(self):
        """测试添加数据"""
        rb = RingBuffer(5)
        
        # 添加3个元素
        rb.append(1.0)
        rb.append(2.0)
        rb.append(3.0)
        
        self.assertEqual(rb.count, 3)
        self.assertEqual(rb.ptr, 3)
        
        # 获取数组
        arr = rb.get_array()
        np.testing.assert_array_equal(arr, np.array([1.0, 2.0, 3.0]))
    
    def test_overflow(self):
        """测试缓冲区溢出"""
        rb = RingBuffer(3)
        
        # 添加5个元素，超过缓冲区大小
        rb.append(1.0)
        rb.append(2.0)
        rb.append(3.0)
        rb.append(4.0)
        rb.append(5.0)
        
        self.assertEqual(rb.count, 3)
        self.assertEqual(rb.ptr, 2)  # 指针应该回绕到2
        
        # 获取数组，应该是最后3个元素
        arr = rb.get_array()
        np.testing.assert_array_equal(arr, np.array([3.0, 4.0, 5.0]))
    
    def test_clear(self):
        """测试清空缓冲区"""
        rb = RingBuffer(5)
        
        # 添加数据
        rb.append(1.0)
        rb.append(2.0)
        
        # 清空缓冲区
        rb.clear()
        
        self.assertEqual(rb.count, 0)
        self.assertEqual(rb.ptr, 0)
        
        # 获取数组，应该为空
        arr = rb.get_array()
        self.assertEqual(len(arr), 0)


class TestPlotDataManager(unittest.TestCase):
    """绘图数据管理器测试类"""
    
    def test_init(self):
        """测试初始化"""
        pdm = PlotDataManager(100)
        self.assertEqual(pdm.max_points, 100)
        self.assertIsNone(pdm.plot_time_zero)
        self.assertEqual(len(pdm.data_buffers), 0)
    
    def test_add_data_channel(self):
        """测试添加数据通道"""
        pdm = PlotDataManager(100)
        
        # 添加通道
        pdm.add_data_channel('test')
        
        self.assertIn('test', pdm.data_buffers)
        self.assertEqual(pdm.data_buffers['test'].max_size, 100)
    
    def test_add_data_point(self):
        """测试添加数据点"""
        pdm = PlotDataManager(100)
        
        # 添加数据点
        timestamp = time.time()
        pdm.add_data_point(timestamp, {'ch1': 1.0, 'ch2': 2.0})
        
        # 检查时间零点
        self.assertEqual(pdm.plot_time_zero, timestamp)
        
        # 检查通道
        self.assertIn('ch1', pdm.data_buffers)
        self.assertIn('ch2', pdm.data_buffers)
        
        # 检查数据
        t_arr, y_arr = pdm.get_plot_data('ch1')
        self.assertEqual(len(t_arr), 1)
        self.assertEqual(len(y_arr), 1)
        self.assertEqual(t_arr[0], 0.0)  # 相对时间应该是0
        self.assertEqual(y_arr[0], 1.0)
    
    def test_get_plot_data_with_window(self):
        """测试获取带时间窗口的绘图数据"""
        pdm = PlotDataManager(100)
        
        # 添加数据点
        t0 = time.time()
        pdm.add_data_point(t0, {'ch1': 1.0})
        pdm.add_data_point(t0 + 1.0, {'ch1': 2.0})
        pdm.add_data_point(t0 + 2.0, {'ch1': 3.0})
        pdm.add_data_point(t0 + 3.0, {'ch1': 4.0})
        
        # 获取最近2秒的数据
        t_arr, y_arr = pdm.get_plot_data('ch1', window=2.0)
        
        # 应该只有最后2个数据点
        self.assertEqual(len(t_arr), 2)
        self.assertEqual(len(y_arr), 2)
        np.testing.assert_array_almost_equal(y_arr, np.array([3.0, 4.0]))
    
    def test_clear(self):
        """测试清空数据"""
        pdm = PlotDataManager(100)
        
        # 添加数据点
        timestamp = time.time()
        pdm.add_data_point(timestamp, {'ch1': 1.0})
        
        # 清空数据
        pdm.clear()
        
        # 检查时间零点
        self.assertIsNone(pdm.plot_time_zero)
        
        # 检查数据
        t_arr, y_arr = pdm.get_plot_data('ch1')
        self.assertEqual(len(t_arr), 0)
        self.assertEqual(len(y_arr), 0)


if __name__ == '__main__':
    unittest.main() 