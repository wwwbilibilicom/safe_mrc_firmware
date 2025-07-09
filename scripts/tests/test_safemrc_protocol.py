#!/usr/bin/env python
"""
SafeMRC通信协议单元测试
"""
import sys
import os
import unittest

# 添加父目录到路径，以便导入模块
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from safemrc_comm import SerialThread, crc_ccitt
from config import SAFEMRC_HEADER

class TestSafeMRCProtocol(unittest.TestCase):
    """SafeMRC通信协议测试类"""
    
    def test_crc_ccitt(self):
        """测试CRC-CCITT计算"""
        # 测试空数据
        self.assertEqual(crc_ccitt(b''), 0xFFFF)
        
        # 测试已知数据
        test_data = b'\x01\x02\x03\x04'
        expected_crc = 0xD9B3  # 预先计算的CRC值
        self.assertEqual(crc_ccitt(test_data), expected_crc)
        
        # 测试实际协议数据
        protocol_data = SAFEMRC_HEADER + b'\x01\x00\x00\x00\x00\x00'
        crc = crc_ccitt(protocol_data)
        # 确保CRC不为0xFFFF（初始值）
        self.assertNotEqual(crc, 0xFFFF)
    
    def test_pack_cmd(self):
        """测试命令打包"""
        # 创建SerialThread实例
        thread = SerialThread()
        thread.device_id = 1
        
        # 测试模式0，电流0.0
        cmd = thread.pack_cmd(0, 0.0)
        
        # 检查帧头
        self.assertEqual(cmd[0:2], SAFEMRC_HEADER)
        
        # 检查设备ID
        self.assertEqual(cmd[2], 1)
        
        # 检查模式
        self.assertEqual(cmd[3], 0)
        
        # 检查电流（应该是0）
        current_bytes = cmd[4:8]
        current_value = int.from_bytes(current_bytes, 'little', signed=True)
        self.assertEqual(current_value, 0)
        
        # 检查CRC
        crc_bytes = cmd[8:10]
        crc_value = int.from_bytes(crc_bytes, 'little')
        expected_crc = crc_ccitt(cmd[0:8])
        self.assertEqual(crc_value, expected_crc)
        
        # 测试不同的模式和电流
        cmd = thread.pack_cmd(2, 1.5)
        
        # 检查模式
        self.assertEqual(cmd[3], 2)
        
        # 检查电流（应该是1500 mA）
        current_bytes = cmd[4:8]
        current_value = int.from_bytes(current_bytes, 'little', signed=True)
        self.assertEqual(current_value, 1500)
    
    def test_parse_feedback(self):
        """测试反馈解析"""
        # 创建SerialThread实例
        thread = SerialThread()
        
        # 创建一个有效的反馈帧
        frame = bytearray(17)
        frame[0:2] = SAFEMRC_HEADER  # 帧头
        frame[2] = 1  # 设备ID
        frame[3] = 2  # 模式
        frame[4] = 0  # 碰撞标志
        
        # 编码器角度：1.5 rad
        encoder_value = int(1.5 * 65535)
        frame[5:9] = encoder_value.to_bytes(4, 'little', signed=True)
        
        # 编码器速度：2.5 rad/s
        velocity_value = int(2.5 * 1000)
        frame[9:13] = velocity_value.to_bytes(4, 'little', signed=True)
        
        # 电流：0.75 A
        current_value = int(0.75 * 1000)
        frame[13:15] = current_value.to_bytes(2, 'little', signed=True)
        
        # 计算CRC
        crc = crc_ccitt(frame[0:15])
        frame[15:17] = crc.to_bytes(2, 'little')
        
        # 解析反馈
        data = thread.parse_feedback(frame)
        
        # 检查解析结果
        self.assertIsNotNone(data)
        self.assertEqual(data['id'], 1)
        self.assertEqual(data['mode'], 2)
        self.assertEqual(data['collision'], 0)
        self.assertAlmostEqual(data['encoder'], 1.5, places=2)
        self.assertAlmostEqual(data['velocity'], 2.5, places=2)
        self.assertAlmostEqual(data['current'], 0.75, places=2)
        self.assertEqual(data['crc_recv'], crc)
        self.assertEqual(data['crc_calc'], crc)
        
        # 测试无效帧头
        invalid_frame = bytearray(frame)
        invalid_frame[0] = 0x00  # 修改帧头
        data = thread.parse_feedback(invalid_frame)
        self.assertIsNone(data)
        
        # 测试CRC错误
        invalid_frame = bytearray(frame)
        invalid_frame[15] = 0x00  # 修改CRC
        data = thread.parse_feedback(invalid_frame)
        self.assertNotEqual(data['crc_recv'], data['crc_calc'])


if __name__ == '__main__':
    unittest.main() 