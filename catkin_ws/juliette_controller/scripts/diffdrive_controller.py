#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Declare the length between the two wheels
L = 0.244

def cmd_vel_callback(msg):
		# Get targeted v and w from message
		v = msg.linear.x 
		w = msg.angular.z

		# Compute tangential velocity for each wheel 
		vl = (v - (w * L) / 2) 
		vr = (v + (w * L) / 2)
		
		# Publish the results
		lwheel_tangent_vel_target_pub.publish(vl)
		rwheel_tangent_vel_target_pub.publish(vr)

if __name__ == '__main__':
	rospy.init_node('diffdrive_controller')
	
	# Publisher for tangential velocities
	lwheel_tangent_vel_target_pub = rospy.Publisher('lwheel_tangent_vel_target', Float32, queue_size=10)
	rwheel_tangent_vel_target_pub = rospy.Publisher('rwheel_tangent_vel_target', Float32, queue_size=10)
	
	# Subscriber to command topic
	rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
	
	rospy.spin()
