#!/usr/bin/env python3
# BEGIN ALL
# BEGIN SHEBANG
#!/usr/bin/env python3
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

if __name__ == "__main__":
    try :
        rospy.init_node('main')
        print("Create main node")

        # BEGIN PUB
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # END PUB
        
        rate = rospy.Rate(10)
        # BEGIN LOOP
        while not rospy.is_shutdown():
            
            T = Twist()
            T.linear.x = 1
            T.linear.y = 1
            T.angular.z = 0

            cmd_vel.publish(T)

            rate.sleep()
           
    except rospy.ROSInterruptException:
        pass


# END ALL
