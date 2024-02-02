#!/usr/bin/env python
# BEGIN ALL
# BEGIN SHEBANG
#!/usr/bin/env python
# END SHEBANG

"""
pip install pyserial
"""

# BEGIN IMPORT
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
        # self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        
        # 启动线程接收数据
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.start()
    
    def Check_Checksum(data):
        # Calc Check Byte sum
        return sum(data) & 0xFF

    def Calc_Checksum(self, address, command, data_length, data):
        """计算校验和"""
        return (address + command + data_length + sum(data)) & 0xFF
    
    def Data_Package (self, address, command, data):
        """打包数据"""
        data_length = len(data)
        checksum = self.Calc_Checksum(address, command, data_length, data)
        # 使用struct打包数据
        packed_data = struct.pack('BBB', address, command, data_length) + bytes(data) + struct.pack('B', checksum)
        # 添加帧头
        frame_header = struct.pack('H', 0xFFFF)
        print ("Serial Sent: ", frame_header + packed_data)
        return frame_header + packed_data
    
    
    def send_data(self, address, command, data):
        # Send Data to Device
        packed_data = self.Data_Package(address, command, data)
        self.serial_device.write(packed_data)

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





if __name__ == "__main__1":
    rospy.spin()  # 进入ROS事件循环
    try :
        rospy.init_node('main')
        print("Create main node")

        # 实例化RobotController
        controller = RobotController(port='/dev/serial', baud_rate=115200)
        rospy.spin()  # 进入ROS事件循环
        
        # BEGIN PUB
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # END PUB
        
        # BEGIN LOOP
        
    except rospy.ROSInterruptException:
        pass


# END ALL

if __name__ == '__main__':
    try:
        # 创建RobotController实例，指定串口和波特率
        communicator = RobotController('/dev/serial', 115200)
        # 设置设备地址、命令字和数据
        address = 0x01
        command = 0x02
        data = [0x10, 0x20, 0x30]
        # 发送数据
        communicator.send_data(address, command, data)
    finally:
        # 关闭串口
        communicator.close()