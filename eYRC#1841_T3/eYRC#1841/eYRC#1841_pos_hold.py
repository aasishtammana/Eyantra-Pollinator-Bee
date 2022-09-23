#!/usr/bin/env python

#The required packages are imported here
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy
import time
import sys

class DroneFly():
	"""docstring for DroneFly"""
	def __init__(self):
		
		rospy.init_node('pluto_fly', disable_signals = True)#listener

		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
		
		# To publish errors
		self.alterr = rospy.Publisher('/alt_error', Float64, queue_size=10)
		self.rollerr=rospy.Publisher('/roll_error',Float64,queue_size=10)
		self.pitcherr=rospy.Publisher('/pitch_error',Float64,queue_size=10)
		self.yawerr=rospy.Publisher('/yaw_error',Float64,queue_size=10)
		
		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		
		# To tune the drone during runtime
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
		#rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)
		
		self.cmd = PlutoMsg()
		
		# Variables used in PID formula
		self.error_throt=0#current
		self.prev_throt=0#previous
		self.error_roll=0#current
		self.prev_roll=0#previous
		self.error_pitch=0#current
		self.prev_pitch=0#previous
		self.error_yaw=0#current
		self.prev_yaw=0#previous

		self.Iterm_throt=0
		self.Iterm_roll=0
		self.Iterm_pitch=0
		self.Iterm_yaw=0
		self.num_t=0
		self.num_r=0
		self.num_p=0
		
		#Other variables
		#self.waypoints=[(6.0,4,21.0),(6.0,4,21.0)]
		#self.waypoints=[(2,-2,28),(2,-2,28)]
		self.waypoints=[(0,0,20),(0,0,20)]
		self.wp=0
		self.next_waypoint_time=10.0
		self.last_waypoint_time=0.0
		self.last_waypoint=False
		
		# Position to hold.
		#self.wp_yaw=0.3
		self.next_waypoint(self.wp)
		
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0
		
		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0
		self.drone_yaw=0.0
		
		#PID constants for Throttle
		self.kp_throt = 10.0 #11.0
		self.ki_throt = 0.01
		self.kd_throt = 30.0 #81.0
		
		#PID constants for Roll
		self.kp_roll = 6 #7
		self.ki_roll = 0#0.01  #0.0
		self.kd_roll = 120#120
		#PID constants for Pitch
		self.kp_pitch = 7 #7
		self.ki_pitch = 0#0.1  #0.0
		self.kd_pitch = 120 #120
		
		#PID constants for Yaw
		self.kp_yaw = 16.0
		self.ki_yaw = 0.0
		self.kd_yaw = 180.0

		
		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0
		
		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.05

		rospy.sleep(.1)
		
	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)
		
	def position_hold(self):

		rospy.sleep(2)
		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)
		
		while True:

			self.calc_pid()
			
			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
		 	throt_value = int(1550 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1650,1450)
			
			pitch_value = int(1500 - self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 - self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
			
			self.pluto_cmd.publish(self.cmd)
			
			self.waypoint_timer = time.time()
			cur_waypoint_time = self.waypoint_timer - self.last_waypoint_time
			if(cur_waypoint_time >= self.next_waypoint_time and self.last_waypoint==False):
				self.next_waypoint(self.wp)
				self.wp=self.wp+1
				self.last_waypoint_time = self.waypoint_timer
	
	def calc_pid(self):
		self.seconds = time.time()
		current_time = self.seconds - self.last_time
		if(current_time >= self.loop_time):
			self.pid_roll()
			self.pid_pitch()
			self.pid_throt()
			#self.pid_yaw()
			self.publish_plot_data()			
			self.last_time = self.seconds
		
	def pid_throt(self):
		#Compute Throttle PID here
		self.error_throt=self.wp_z-self.drone_z
		if(self.num_t>30):
			self.Iterm_throt=0
			self.num_t=0
		else:
			self.Iterm_throt=(self.Iterm_throt+self.error_throt)*self.ki_throt
		self.correct_throt = (self.kp_throt*self.error_throt)+self.Iterm_throt + (self.kd_throt*(self.error_throt-self.prev_throt))
		self.prev_throt=self.error_throt
		self.num_t=self.num_t+1;
		
	def pid_roll(self):
		#Compute Roll PID here
		self.error_roll=self.wp_y-self.drone_y
		if(self.num_r>30):
			self.Iterm_roll=0
			self.num_r=0
		else:
			self.Iterm_roll=(self.Iterm_roll+self.error_roll)*self.ki_roll
		self.correct_roll = (self.kp_roll*self.error_roll)+self.Iterm_roll + (self.kd_roll*(self.error_roll-self.prev_roll))
		self.prev_roll=self.error_roll
		self.num_r=self.num_r+1;


	def pid_pitch(self):
		#Compute Pitch PID here
		self.error_pitch=self.wp_x-self.drone_x
		if(self.num_p>30):
			self.Iterm_pitch=0
			self.num_p=0
		else:
			self.Iterm_pitch=(self.Iterm_pitch+self.error_pitch)*self.ki_pitch
		self.correct_pitch = (self.kp_pitch*self.error_pitch)+self.Iterm_pitch + (self.kd_pitch*(self.error_pitch-self.prev_pitch))
		self.prev_pitch=self.error_pitch
		self.num_p=self.num_p+1;
		
	
		
	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value
			
	def set_pid_alt(self,pid_val):
		
		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

		self.kp_throt = pid_val.Kp
		self.ki_throt = pid_val.Ki
		self.kd_throt = pid_val.Kd

	def set_pid_roll(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

		self.kp_roll = pid_val.Kp
		self.ki_roll = pid_val.Ki
		self.kd_roll = pid_val.Kd
		
	def set_pid_pitch(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

		self.kp_pitch = pid_val.Kp
		self.ki_pitch = pid_val.Ki
		self.kd_pitch = pid_val.Kd
	
		
	def publish_plot_data(self):
		self.alterr.publish(self.error_throt)
		self.rollerr.publish(self.error_roll)
		self.pitcherr.publish(self.error_pitch)
		self.yawerr.publish(self.error_yaw)
		
		
	def get_pose(self,pose):

		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z
		
	
	def next_waypoint(self,n):
		if(n==1):
			self.last_waypoint=True
		else:
			w=self.waypoints[n]
			self.wp_x = w[0]
			self.wp_y = w[1]
			self.wp_z = w[2]
		

if __name__ == '__main__':
	try:	
		while not rospy.is_shutdown():
			temp = DroneFly()
			temp.position_hold()
		rospy.spin()
	except KeyboardInterrupt:
		print "disarm"
		temp = DroneFly()
		temp.disarm()
		sys.exit()
