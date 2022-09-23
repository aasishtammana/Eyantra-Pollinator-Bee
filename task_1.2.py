#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32

class WayPoint:
	
	def __init__(self):

		#image processing part
		rospy.init_node('ros_bridge')
		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()
		# Subscribe to whycon image_out
		self.image_sub = rospy.Subscriber('whycon/image_out', Image, self.image_callback)


		



		
	def image_callback(self,msg):

		# 'image' is now an opencv frame
		# You can run opencv operations on 'image'
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		print("Hello this is working")
		hsv_image=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

		def nothing(x):
			pass

		
		#this is for red colour
		lower_red=np.array([0,100,85])
		upper_red=np.array([0,255,255])
		red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
		red_res = cv2.bitwise_and(image,image, mask= red_mask)

		#this is for blue color
		lower_blue=np.array([120,100,85])
		upper_blue=np.array([120,255,255])
		blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
		blue_res = cv2.bitwise_and(image,image, mask= blue_mask)

		#this is for green color
		lower_green=np.array([60,100,85])
		upper_green=np.array([60,255,255])
		green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
		green_res = cv2.bitwise_and(image,image, mask= green_mask)


		green=np.uint8([[[0,218,0]]])
		hsv_green=cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
		print(hsv_green)

		#Output image
		cv2.imshow('rgbimage',image)
		#cv2.imshow('grayimage',gray_image)
		cv2.imshow('hsvimage',hsv_image)
		
		cv2.imshow('red_mask',red_mask)
		cv2.imshow('red_result',red_res)
		
		cv2.imshow('blue_mask',blue_mask)
		cv2.imshow('blue_result',blue_res)

		cv2.imshow('green_mask',green_mask)
		cv2.imshow('green_result',green_res)

		cv2.waitKey(0)
		cv2.destroyAllWindows()
		

		
if __name__ == '__main__':
	test = WayPoint()
	rospy.spin()

		