#!/usr/bin/env python3
# BEGIN ALL
# BEGIN SHEBANG
#!/usr/bin/env python3
# END SHEBANG

"""
pip install pyserial
"""

# BEGIN IMPORT
import numpy as np
import math
import rospy
import serial
import struct
from geometry_msgs.msg import Twist
import tf
import threading
import time
# END IMPORT
 
# BEGIN STD_MSGS
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# END STD_MSGS

class RobotController:
    
    def __init__(self, port, baud_rate):
        # init Ros Node : robot_controller
        rospy.init_node('robot_controller', anonymous=True)
        # Try to open serial
        try:
            self.serial_device = serial.Serial(port, baud_rate, timeout=1)
        except serial.SerialException as e:
            rospy.logerr("Failed to open serial port: %s" % str(e))
            exit()
        
        # Subscribe "cmd_vel" topic
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.Velocity_Call_Back)
        # 启动线程接收数据
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.start()
        
        rospy.spin()
        
    def Check_Checksum(data):
        # Calc Check Byte sum
        return sum(data) & 0xFF
    
    def Velocity_Call_Back (self, data):
        print(data)

        # 车辆参数
        L = 0.25  # 车长 m
        W = 0.25  # 车宽 m
        rad = 0.04 # rad of whell m
        R = math.sqrt(L**2 + W**2)  # 到中心点的距离 m

        # 输入参数
        vx = data.linear.x  # x方向速度 m/s
        vy = data.linear.y   # y方向速度 m/s
        omega = data.angular.z   # 偏航轴角速度 rad/s

        wheel_speeds = np.zeros(4)

        wheel_speeds[0] = vx - vy + (W + L) * omega
        wheel_speeds[1] = vx + vy - (W + L) * omega
        wheel_speeds[2] = vx + vy + (W + L) * omega
        wheel_speeds[3] = vx - vy - (W + L) * omega

        # wheel_speeds /= rad

        # 示例用法
        device_address = "RF"  # 地盘电机地址
        command = "V"  # 示例命令字
        data = 100  # 示例数据，int类型
        
        rospy.loginfo("Wheel Speeds in rpm: {}".format(wheel_speeds))

        self.send_data("RF", "V", wheel_speeds[0])
        self.send_data("LF", "V", wheel_speeds[1])
        self.send_data("RB", "V", wheel_speeds[2])
        self.send_data("LB", "V", wheel_speeds[3])


    def pack_message(self, device_address_o, command_o, data):
        """
        打包消息为指定的协议格式。
        
        参数:
        - device_address_o: str, 设备地址的名称
        - command_o: str, 命令字的名称
        - data: int, 命令的具体参数
        
        返回:
        - 打包好的消息, bytes类型
        """
        
        command_map = {
            'R': 0x01, 'S': 0x02,
            'V': 0x03, 'A': 0x04,
            'Q': 0x05
        }

        address_map = {
            'RF': 0x01, 'LF': 0x02, 'RB': 0x03, 'LB': 0x04,
            'CY': 0x05,
            'ML': 0x06, 'MR': 0x07,
            'SL': 0x08, 'SR': 0x09
        }

        # 查找设备地址和命令码
        device_address = address_map.get(device_address_o, 0x00)
        command = command_map.get(command_o, 0x00)

        if (device_address == 0x00 or command == 0x00):
            rospy.loginfo("err : device_address == 0x00 or command == 0x00")
            rospy.loginfo("Unknow Code")

        # 数据处理
        data = int(data)
        data &= 0xFFFF
        # 数据打包，使用大端模式
        message = struct.pack('>BBBH', 0xFF, 0xFF, device_address, command) + struct.pack('>H', data)
        # 计算校验和
        checksum = sum(message) & 0xFF
        # 添加校验和
        message += struct.pack('>B', checksum)

        # print(message)
        
        return message

    def send_data(self, dev, cmd, data):
        # Send Data to Device.
        serial_msg = self.pack_message(dev, cmd, data)
        rospy.loginfo(serial_msg)
        self.serial_device.write(serial_msg)

    def close(self) :
        # close Serial 
        if self.serial_device.is_open:
            rospy.loginfo("Serial Device Close ...")
            self.serial_device.close()
        else :
            rospy.loginfo("Serial Has Already Closed ...")
    
    def receive_data(self):
        # 在这个线程中接收来自串行设备的数据
        # 并根据需要处理这些数据（例如更新里程计信息）
        if rospy.is_shutdown() and self.serial_device.in_waiting:
            data = self.serial_device.read_all()
            print("get:", data)

if __name__ == "__main__":
    try :
        rc = RobotController("/dev/serial", 115200)
        



    except rospy.ROSInterruptException:
        pass


# END ALL
    