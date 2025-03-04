#!/usr/bin/python

import rospy
import roslib
from smbus import SMBus

# Messages
from std_msgs.msg import Float32

# Issue commands to the juliette motors to achieve the target velocity
# Use a PID that compares the error based on encoder readings
class ControlsToMotors:
	def __init__(self):
		rospy.init_node('juliette_controller')
		self.rate = rospy.get_param('~rate', 10)
		self.Kp = rospy.get_param('~Kp', 1.0)
		self.Ki = rospy.get_param('~Ki', 1.0)
		self.Kd = rospy.get_param('~Kd', 1.0)

		# Radian/s min and max velocities
		self.motor_max_angular_vel = rospy.get_param('~motor_max_angular_vel',6.0)
		self.motor_min_angular_vel = rospy.get_param('~motor_min_angular_vel',1.0)
		# Corresponding motor commands
		self.motor_cmd_max = rospy.get_param('~motor_cmd_max',180)
		self.motor_cmd_min = rospy.get_param('~motor_cmd_min',30)
		
		# Constants for SMBus
		self.i2c_addr = 0x7 # bus address
		self.i2c_bus = SMBus(1) # indicates /dev/ic2-1
		self.break_cmd = 0
		self.left_motor_cmd = 2
		self.right_motor_cmd = 1
		self.forward_msg = 1
		self.backward_msg = 0

		self.R = rospy.get_param('~robot_wheel_radius', 0.063)
		self.pid_on = rospy.get_param('~pid_on',False)
		self.juliette_on = rospy.get_param('~juliette_on',True)

		# (Optional) Publish the computed angular velocity targets
		self.lwheel_angular_vel_target_pub = rospy.Publisher('lwheel_angular_vel_target', Float32, queue_size=10)
		self.rwheel_angular_vel_target_pub = rospy.Publisher('rwheel_angular_vel_target', Float32, queue_size=10)
		
		# (Optional) Publish the computed angular velocity control command
		self.lwheel_angular_vel_control_pub = rospy.Publisher('lwheel_angular_vel_control', Float32, queue_size=10)
		self.rwheel_angular_vel_control_pub = rospy.Publisher('rwheel_angular_vel_control', Float32, queue_size=10)

		# (Optional) Publish the computed angular velocity motor command
		self.lwheel_angular_vel_motor_pub = rospy.Publisher('lwheel_angular_vel_motor', Float32, queue_size=10)
		self.rwheel_angular_vel_motor_pub = rospy.Publisher('rwheel_angular_vel_motor', Float32, queue_size=10)

		# Read in encoders for PID control
		self.lwheel_angular_vel_enc_sub = rospy.Subscriber('lwheel_angular_vel_enc', Float32, self.lwheel_angular_vel_enc_callback)    
		self.rwheel_angular_vel_enc_sub = rospy.Subscriber('rwheel_angular_vel_enc', Float32, self.rwheel_angular_vel_enc_callback)    

		# Read in tangential velocity targets
		self.lwheel_tangent_vel_target_sub = rospy.Subscriber('lwheel_tangent_vel_target', Float32, self.lwheel_tangent_vel_target_callback)
		self.rwheel_tangent_vel_target_sub = rospy.Subscriber('rwheel_tangent_vel_target', Float32, self.rwheel_tangent_vel_target_callback)


		# Tangential velocity target
		self.lwheel_tangent_vel_target = 0;
		self.rwheel_tangent_vel_target = 0;

		# Angular velocity target
		self.lwheel_angular_vel_target = 0
		self.rwheel_angular_vel_target = 0
		
		# Angular velocity encoder readings
		self.lwheel_angular_vel_enc = 0
		self.rwheel_angular_vel_enc = 0

		# PID control variables
		self.lwheel_pid = {}
		self.rwheel_pid = {}

	# ==================================================
	# Read in tangential velocity targets
	# ==================================================
	def lwheel_tangent_vel_target_callback(self, msg):
		self.lwheel_tangent_vel_target = msg.data

	def rwheel_tangent_vel_target_callback(self, msg):
		self.rwheel_tangent_vel_target = msg.data
		
	# ==================================================
	# Read in encoder readings for PID
	# ==================================================
	def lwheel_angular_vel_enc_callback(self, msg):
		self.lwheel_angular_vel_enc = msg.data

	def rwheel_angular_vel_enc_callback(self, msg):
		self.rwheel_angular_vel_enc = msg.data

	# ==================================================
	# Update motor commands
	# ==================================================

	# Compute angular velocity target
	def tangentvel_2_angularvel(self,tangent_vel):
		# v = wr
		# v - tangential velocity (m/s)
		# w - angular velocity (rad/s)
		# r - radius of wheel (m)
		angular_vel = tangent_vel / self.R;
		return angular_vel


	# PID control
	def pid_control(self,wheel_pid,target,state):


		# Initialize pid dictionary
		if len(wheel_pid) == 0:
			wheel_pid.update({'time_prev':rospy.Time.now(), 'derivative':0, 'integral':[0]*10, 'error_prev':0,'error_curr':0})

		wheel_pid['time_curr'] = rospy.Time.now()

		# PID control
		wheel_pid['dt'] = (wheel_pid['time_curr'] - wheel_pid['time_prev']).to_sec()
		if wheel_pid['dt'] == 0: return 0

		wheel_pid['error_curr'] = target - state
		wheel_pid['integral'] = wheel_pid['integral'][1:] + [(wheel_pid['error_curr']*wheel_pid['dt'])]
		wheel_pid['derivative'] = (wheel_pid['error_curr'] - wheel_pid['error_prev'])/wheel_pid['dt']

		wheel_pid['error_prev'] = wheel_pid['error_curr']
		control_signal = (self.Kp*wheel_pid['error_curr'] + self.Ki*sum(wheel_pid['integral']) + self.Kd*wheel_pid['derivative'])
		target_new = target + control_signal

		# Ensure control_signal does not flip sign of target velocity
		if target > 0 and target_new < 0: target_new = target;
		if target < 0 and target_new > 0: target_new = target;

		if (target == 0): # Not moving
			target_new = 0
			return target_new

		wheel_pid['time_prev'] = wheel_pid['time_curr']
		return target_new

	# Mapping angular velocity targets to motor commands
	# Note: motor commands are ints between 0 - 255
	# We also assume motor commands are issues between motor_min_angular_vel and motor_max_angular_vel
	def angularvel_2_motorcmd(self, angular_vel_target):
		if angular_vel_target == 0: return 0;
		slope = (self.motor_cmd_max - self.motor_cmd_min) / (self.motor_max_angular_vel - self.motor_min_angular_vel)
		intercept = self.motor_cmd_max - slope * self.motor_max_angular_vel

		if angular_vel_target > 0: # positive angular velocity
			motor_cmd = slope * angular_vel_target + intercept
			if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
			if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
		
		else: # negative angular velocity
			motor_cmd = slope * abs(angular_vel_target) + intercept
			if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
			if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
			motor_cmd = -motor_cmd      

		return motor_cmd

	# Send motor command to robot
	# motor1 for left wheel. motor1(0, ?) tells wheel to move backwards. motor1(1, ?) tells wheel to move forwards
	# motor2 for right wheel.
	def motorcmd_2_robot(self, wheel='left', motor_command=0):
		if self.juliette_on:
			motor_command_raw = int(abs(motor_command))
			msg = []
			# Print direction into message
			if motor_command >= 0:
				msg.append(self.forward_msg)
			elif motor_command < 0:
				msg.append(self.backward_msg)
			
			# Print motor command value into message
			msg.append(motor_command_raw)			
			
			# Send message to the specific motor
			if wheel == 'left':
				try:
					self.i2c_bus.write_i2c_block_data(self.i2c_addr, self.left_motor_cmd, msg)
				except:
					rospy.loginfo("Unable to send the following command to the Trex")
				finally:
					rospy.loginfo("l: %s", msg)
			elif wheel == 'right':
				try:
					self.i2c_bus.write_i2c_block_data(self.i2c_addr, self.right_motor_cmd, msg)
				except: 
					rospy.loginfo("Unable to send the following command to the Trex")
				finally:
					rospy.loginfo("r: %s", msg);
			
	def lwheel_update(self):
		# Compute target angular velocity
		self.lwheel_angular_vel_target = self.tangentvel_2_angularvel(self.lwheel_tangent_vel_target)
		self.lwheel_angular_vel_target_pub.publish(self.lwheel_angular_vel_target)
		
		# If we want to adjust target angular velocity using PID controller to incorporate encoder readings
		if self.pid_on: 
			self.lwheel_angular_vel_target = self.pid_control(self.lwheel_pid, self.lwheel_angular_vel_target,self.lwheel_angular_vel_enc)
		self.lwheel_angular_vel_control_pub.publish(self.lwheel_angular_vel_target)

		# Compute motor command
		lwheel_motor_cmd = self.angularvel_2_motorcmd(self.lwheel_angular_vel_target)
		self.lwheel_angular_vel_motor_pub.publish(lwheel_motor_cmd)    

		# Send motor command
		self.motorcmd_2_robot('left',lwheel_motor_cmd)

	def rwheel_update(self):
		# Compute target angular velocity
		self.rwheel_angular_vel_target = self.tangentvel_2_angularvel(self.rwheel_tangent_vel_target)
		self.rwheel_angular_vel_target_pub.publish(self.rwheel_angular_vel_target)
		
		# If we want to adjust target angular velocity using PID controller to incorporate encoder readings
		if self.pid_on: 
			self.rwheel_angular_vel_target = self.pid_control(self.rwheel_pid, self.rwheel_angular_vel_target,self.rwheel_angular_vel_enc)
		self.rwheel_angular_vel_control_pub.publish(self.rwheel_angular_vel_target)

		# Compute motor command
		rwheel_motor_cmd = self.angularvel_2_motorcmd(self.rwheel_angular_vel_target)
		self.rwheel_angular_vel_motor_pub.publish(rwheel_motor_cmd)    

		# Send motor command
		self.motorcmd_2_robot('right',rwheel_motor_cmd)

	# When given no commands for some time, do not move
	def spin(self):
		rospy.loginfo("Start juliette_controller")
		rate = rospy.Rate(self.rate)
		
		rospy.on_shutdown(self.shutdown)

		while not rospy.is_shutdown():
			self.rwheel_update()
			self.lwheel_update()
			rate.sleep()
		rospy.spin();

	def shutdown(self):
		rospy.loginfo("Stop juliette_controller")
		# Shut down juliette
		msg = []
		try:
			self.i2c_bus.write_i2c_block_data(self.i2c_addr, self.break_cmd, msg)
		except:
			rospy.loginfo("Unable to send the following command to the Trex")
		finally:
			rospy.loginfo("l&r: Shutdown")
		
		# Stop message
		self.lwheel_angular_vel_target_pub.publish(0)
		self.rwheel_angular_vel_target_pub.publish(0)
		self.lwheel_angular_vel_control_pub.publish(0)
		self.rwheel_angular_vel_control_pub.publish(0)
		self.lwheel_angular_vel_motor_pub.publish(0)
		self.rwheel_angular_vel_motor_pub.publish(0)
		
		rospy.sleep(1)        

def main():
	controls_to_motors = ControlsToMotors();
	controls_to_motors.spin()

if __name__ == '__main__':
	main();
