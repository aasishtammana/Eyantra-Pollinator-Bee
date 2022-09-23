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
		#self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)
		rospy.Subscriber('/whycon/image_out', Image, self.image_callback)
		
	def image_callback(self,msg):

		def display(window_name,image_name):
			cv2.imshow(window_name,image_name)

		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		hsv_image=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		#display("Input image",image)
		#display("HSV Image ",hsv_image)

		lower_red=np.array([170,100,100])#160,100,100
		upper_red=np.array([192,255,255])#192,255,255
			
		lower_green=np.array([40,100,100])#40,100,100
		upper_green=np.array([70,255,255])#70,255,255

		lower_blue=np.array([110,100,100])#110,100,100
		upper_blue=np.array([130,255,255])#130,255,255



		red=cv2.inRange(hsv_image, lower_red, upper_red)
		green=cv2.inRange(hsv_image,lower_green,upper_green)
		blue=cv2.inRange(hsv_image,lower_blue,upper_blue)
		#display("red_image",red)
		#display("blue_mask",blue)
		#display("green_mask",green)

		red_new = cv2.bitwise_and(image,image, mask = red)
		green_new = cv2.bitwise_and(image,image, mask = green)
		blue_new = cv2.bitwise_and(image,image, mask = blue)
		#display("red new",red_new)
		#display("blue new",blue_new)
		#display("green new",green_new)



		gray_blue = cv2.cvtColor(blue_new, cv2.COLOR_BGR2GRAY)
		#display("Grayscale blue",gray_blue)
		ret,thresh_blue = cv2.threshold(gray_blue,50,255,cv2.THRESH_BINARY)
		#display("Blue bin",thresh_blue)
		contour_image_blue,cnts_blue,hierarchy = cv2.findContours(thresh_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(contour_image_blue,cnts_blue,-1,(0,0,0),2)#the parameters are input image, output contour img,external colour, internal colour, thickness 
		contour_image_new,cnts_new,hierarchy = cv2.findContours(contour_image_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		#display("Blue contours",contour_image_new)
		cv2.drawContours(contour_image_new,cnts_new,-1,(0,0,0),2)#the parameters are input image, output contour img,external colour, internal colour, thickness 
		#display("Blue contours",contour_image_new)

		gray_red = cv2.cvtColor(red_new, cv2.COLOR_BGR2GRAY)
		#display("Grayscale red",gray_red)
		ret,thresh_red = cv2.threshold(gray_red,140,255,cv2.THRESH_BINARY)
		#display("red bin",thresh_red)
		contour_image_red,cnts_red,hierarchy = cv2.findContours(thresh_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		#display("red contours",contour_image_red)
		cv2.drawContours(contour_image_blue,cnts_red,-1,(100,100,100),2)#the parameters are input image, output contour img,external colour, internal colour, thickness 
		#display("red contours",contour_image_red)

		gray_green = cv2.cvtColor(green_new, cv2.COLOR_BGR2GRAY)
		#display("Grayscale green",gray_green)
		ret,thresh_green = cv2.threshold(gray_green,50,255,cv2.THRESH_BINARY)
		#display("green bin",thresh_green)
		contour_image_green,cnts_green,hierarchy = cv2.findContours(thresh_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		#display("green contours",contour_image_green)
		cv2.drawContours(contour_image_green,cnts_green,-1,(100,100,100),2)#the parameters are input image, output contour img,external colour, internal colour, thickness 
		#display("green contours",contour_image_green)


		for contour in cnts_red:
			area=cv2.contourArea(contour)
			#print("Red contour area :",area)
			if(area>1):
				x,y,w,h=cv2.boundingRect(contour)
				cv2.rectangle(image,(x-20,y-20),(x+w+25,y+h+20),(0,0,255),3)#to draw rectangle
		#display("rectangle_red",image)

		for contour in cnts_green:
			area=cv2.contourArea(contour)
			#print("greeen contour area =",area)
			if(area>60 and area<200):
				x,y,w,h=cv2.boundingRect(contour)
				cv2.rectangle(image,(x-10,y-10),(x+w+10,y+h+10),(0,255,0),3)#to draw rectangle
		#display("rectangle_green",image)

		for contour in cnts_blue:
			area=cv2.contourArea(contour)
			#print("Blue contour area :",area)
			if(area>0):
				x,y,w,h=cv2.boundingRect(contour)
				cv2.rectangle(image,(x-10,y-10),(x+w+10,y+h+10),(255,0,0),3)#to draw rectangle
		#display("rectangle_blue",image)
		display("image",image)		
		
		#cv2.updateWindow("image")	
		cv2.waitKey(3)
		#cv2.destroyAllWindows()		

		
if __name__ == '__main__':
	while not rospy.is_shutdown():
		test = WayPoint()
		rospy.spin()
