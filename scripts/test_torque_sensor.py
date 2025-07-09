import sys
import time
from torque_sensor import TorqueSensor

def main():
    # 端口和波特率可根据实际情况修改
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM3'  # Windows下常用COM3
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
    sensor = TorqueSensor()
    if not sensor.initialize(port, baudrate):
        print('Failed to initialize torque sensor!')
        return
    print('Reading torque value...')
    torque = sensor.read_torque()
    print(f'Torque: {torque:.4f} Nm')
    sensor.close()

if __name__ == '__main__':
    main() 