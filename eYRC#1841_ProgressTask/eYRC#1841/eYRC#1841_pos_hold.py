'''
* Team Id : eYRC#1841
* Author List : Aakash Hegde, Abhijit BN
* Filename: task_3_pos_hold.py
* Theme: Pollinator Bee
* Functions: arm(), disarm(), position_hold(), calc_pid(), get_iterm(int), pid_throt(), pid_roll(), pid_pitch(), limit(int, int, int),
		set_pid_alt(), set_pid_roll(), set_pid_pitch(), publist_plot_data(), get_pose(int), next_waypoint(int)
* Global Variables: -
'''

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

		self.iterm_throt=0
		self.iterm_roll=0
		self.iterm_pitch=0
		self.iterm_yaw=0
		
		#Waypoints the drone has to traverse to perform the task.
		self.waypoints=[(7.12,-0.52,17.0),(2.8,4.15,18),(2.8,4.15,19.2),(2.8,4.15,16.5),(0,0,21)] 
			
		self.wp=0	#Initial waypoint number i.e. first waypoint	
		self.next_waypoint_time=6.0	#interval at which waypoints are changed.
		self.last_waypoint_time=0.0
		self.last_waypoint=False	#boolean to indicate if last waypoint is reached.
		
		#Array to hold i-terms
		self.throt_array=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
		self.roll_array=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
		self.pitch_array=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
		
		
		#Positon of current data in the iterm array i.e. a pointer.
		self.roll_tcount=0;	
		self.roll_dcount=0;
		self.roll_pcount=0;
		
		#Counters that are used to de-noise the error curve. Error is generated due to noise in whycon detection.
		self.throt_counter=0;
		self.roll_counter=0;
		self.pitch_counter=0;
		self.yaw_counter=0;
		
		#Move to first waypoint
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
		self.kp_throt = 10.0
		self.ki_throt = 0.5
		self.kd_throt = 30.0
		
		#PID constants for Roll
		self.kp_roll = 4.0
		self.ki_roll = 0.35
		self.kd_roll = 180.0
		#PID constants for Pitch
		self.kp_pitch = 4.5
		self.ki_pitch = 0.25
		self.kd_pitch = 80.0
		
		#PID constants for Yaw
		self.kp_yaw = 16.0
		self.ki_yaw = 0.0
		self.kd_yaw = 180.0

		
		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0
		
		# Loop time for PID computation.
		self.last_time = 0.0
		self.loop_time = 0.05

		rospy.sleep(.1)
		
	'''	
	* Function Name: arm()
	* Input: -
	* Output: -
	* Logic: Initialises drone parameters to arm the drone, and publishes it
	* Example Call: self.arm()
	'''
	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)
	
	'''	
	* Function Name: disarm()
	* Input: -
	* Output: -
	* Logic: Initialises drone parameters to disarm the drone, and publishes it
	* Example Call: self.disarm()
	'''
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)
		
	
	'''	
	* Function Name: position_hold()
	* Input: -
	* Output: -
	* Logic: Responsible for stable flight of the drone. Calls all required functions for the same.
		 Handles publishing of data at regular intervals. Also responsible for changing waypoints.
	* Example Call: self.position_hold()
	'''	
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
			
		 	throt_value = int(1550 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1650,1350)
			
			pitch_value = int(1500 - self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1550, 1450)
															
			roll_value = int(1510 - self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1550,1450)
			
			self.pluto_cmd.publish(self.cmd)
			
			#get current time
			self.waypoint_timer = time.time()
			#get time since last waypoint was updated
			cur_waypoint_time = self.waypoint_timer - self.last_waypoint_time
			#if the time interval set has elapsed, move to next waypoint
			if(cur_waypoint_time >= self.next_waypoint_time and self.last_waypoint==False):
				self.next_waypoint(self.wp)
				self.wp=self.wp+1
				self.last_waypoint_time = self.waypoint_timer
	
	'''	
	* Function Name: calc_pid()
	* Input: -
	* Output: -
	* Logic: Obtains position of the drone at set intervals of time and calls functions to tune PID.
	* Example Call: self.calc_pid()
	'''
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
	
	'''	
	* Function Name: get_iterm(int)
	* Input: num - parameter that decides what axis has to be worked upon
	* Output: -
	* Logic: Generates the i-term (required for the integral component of PID) for the required axis. 
	* Example Call: self.get_iterm(2)
	'''		
	def get_iterm(self,num):
		if(num==0):
			self.throt_array[self.throt_counter]=self.error_throt
			self.throt_counter=self.throt_counter+1
			if(self.throt_counter>30):
				self.throt_counter=0
			self.iterm_throt=sum(self.throt_array)
		if(num==1):
			self.roll_array[self.roll_counter]=self.error_roll
			self.roll_counter=self.roll_counter+1
			if(self.roll_counter>30):
				self.roll_counter=0
			self.iterm_roll=sum(self.roll_array)
		if(num==2):
			self.pitch_array[self.pitch_counter]=self.error_pitch
			self.pitch_counter=self.pitch_counter+1
			if(self.pitch_counter>30):
				self.pitch_counter=0
			self.iterm_pitch=sum(self.pitch_array)
		if(num==3):
			self.yaw_array[self.yaw_counter]=self.error_yaw
			self.yaw_counter=self.yaw_counter+1
			if(self.yaw_counter>30):
				self.yaw_counter=0
			self.iterm_yaw=sum(self.yaw_array)
	
	'''	
	* Function Name: pid_throt(), pid_pitch(), pid_roll()
	* Input: -
	* Output: -
	* Logic: These functions calculate the components of the PID controller. 
		 Error from desired position is generated by using coordinates obtained using whycon marker.
		 The correction is applied as: correct_value = (Kp*error)+(Kd*slope(error))+(Ki*integral(error))
		 Due to noisy whycon performance, a stabilizing algorithm has been implemented.
	* Example Call: self.pid_throt()
	'''				
	def pid_throt(self):
		#Compute Throttle PID here
		self.error_throt=self.wp_z-self.drone_z
		self.get_iterm(0)
		self.correct_throt=(self.kp_throt*self.error_throt)+(self.iterm_throt*self.ki_throt)+(self.kd_throt*(self.error_throt-self.prev_throt))
		#De-noising by taking values at intervals instead of all values.
		if(self.roll_tcount==20):
			self.prev_throt=self.error_throt
			self.roll_tcount=0
		self.roll_tcount=self.roll_tcount+1
		
		
	def pid_roll(self):
		#Compute Roll PID here
		self.error_roll=self.wp_y-self.drone_y
		self.get_iterm(1)
		self.correct_roll=(self.kp_roll*self.error_roll)+(self.iterm_roll*self.ki_roll)+(self.kd_roll*(self.error_roll-self.prev_roll))
		#De-noising by taking values at intervals instead of all values.
		if(self.roll_dcount==10):
			self.prev_roll=self.error_roll
			self.roll_dcount=0
		self.roll_dcount=self.roll_dcount+1


	def pid_pitch(self):
		#Compute Pitch PID here
		self.error_pitch=self.wp_x-self.drone_x
		self.get_iterm(2)
		self.correct_pitch = (self.kp_pitch*self.error_pitch)+(self.iterm_pitch*self.ki_pitch)+(self.kd_pitch*(self.error_pitch-self.prev_pitch))
		#De-noising by taking values at intervals instead of all values.
		if(self.roll_pcount==10):
			self.prev_pitch=self.error_pitch
			self.roll_pcount=0
		self.roll_pcount=self.roll_pcount+1
		
		
	
	'''	
	* Function Name: limit(int, int, int)
	* Input: input_value - corrected drone parameter; max_value - of the parameter; min_value - of the paremeter
	* Output: adjusted value of drone parameter.
	* Logic: Returns the adjusted value of corected drone parameter.
	* Example Call: self.limit(roll_value, 1550,1450)
	'''
	def limit(self, input_value, max_value, min_value):

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value
	
	
	'''	
	* Function Name: set_pid_alt(int), set_pid_roll(int), set_pid_pitch(int)
	* Input: pid_val
	* Output: -
	* Logic: Subscriber function to get the Kp, Ki and Kd values set through the GUI
	* Example Call: -
	'''		
	def set_pid_alt(self,pid_val):

		self.kp_throt = pid_val.Kp
		self.ki_throt = pid_val.Ki
		self.kd_throt = pid_val.Kd

	def set_pid_roll(self,pid_val):

		self.kp_roll = pid_val.Kp
		self.ki_roll = pid_val.Ki
		self.kd_roll = pid_val.Kd
		
	def set_pid_pitch(self,pid_val):
	
		self.kp_pitch = pid_val.Kp
		self.ki_pitch = pid_val.Ki
		self.kd_pitch = pid_val.Kd
	
	
	'''	
	* Function Name: publish_plot_data()
	* Input: -
	* Output: -
	* Logic: Publishes error generated by taking difference between desired and current position of the drone.
	* Example Call: self.publish_plot_data()
	'''	
	def publish_plot_data(self):
		self.alterr.publish(self.error_throt)
		self.rollerr.publish(self.error_roll)
		self.pitcherr.publish(self.error_pitch)
		self.yawerr.publish(self.error_yaw)
		
	
	'''	
	* Function Name: get_pose()
	* Input: pose data
	* Output: -
	* Logic: This is the subscriber function to get the whycon poses.
		 The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
	* Example Call: -
	'''	
	def get_pose(self,pose):
		
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z
					
	
	'''	
	* Function Name: next_waypoint(int)
	* Input: n - number of waypoints to traverse.
	* Output: -
	* Logic: Sets the desired location of the drone. If the waypoint is the last one, the position is held.
	* Example Call: 
	'''
	def next_waypoint(self,n):
		if(n==5):
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
