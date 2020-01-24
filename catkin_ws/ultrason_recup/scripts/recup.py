#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
from std_msgs.msg import String

def talker():
     
    pub = rospy.Publisher('arduino', String, queue_size=25)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = serial.Serial('/dev/ttyACM0', 9600)
	
        rospy.loginfo(hello_str.readline())
        pub.publish(hello_str.readline())
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

