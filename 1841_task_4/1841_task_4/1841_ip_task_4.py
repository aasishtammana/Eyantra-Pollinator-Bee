#!/usr/bin/env python


''' * Team Id : eYRC#1841
* Author List : Aakash Hegde , Abhijith B N
* Filename: task_3_ip.py
* Theme: Pollinator Bee
* Functions:  image_callback,display
* Global Variables: countr
'''



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
		rospy.Subscriber('/whycon/image_out', Image, self.image_callback)
		
		self.red_flower = rospy.Publisher('/red_flower', Int32, queue_size=10)
		self.green_flower = rospy.Publisher('/green_flower', Int32, queue_size=10)
		self.blue_flower = rospy.Publisher('/blue_flower', Int32, queue_size=10)		


	''' * Function Name: image_callback
	* Input: msg (input image from ROS environment to opencv for image processing)
	* Output: image (final output image which contains all the red/blue/green contours of the regions of interest) 
	* Logic: Colour image processing of the HSV Image using upper and lower bounds.Conversion to grayscale image,
			Binary thresholding ,finding and drawing contours, area constraints to derive the Region of interest(ROI)
			followed by Enlosing the ROI within rectangles.
	* 
	* Example Call: rospy.Subscriber('/input_source',Image,self.image_callback)
	'''
	def image_callback(self,msg):



		global countr #global variable- to count the number of RED Led's detected in the frame
		global countg
		global countb


		''' * Function Name: display
		* Input: frame to be displayed, frame_name - which is the name of the window under which the frame has to be displayed
		* Output: diplays the frame 
		* Logic: uses imshow() from opencv library to show the frame.
		* Example Call: display(input_frame,"Input RGB Image")
		'''
		def display(window_name,image_name):
			cv2.imshow(window_name,image_name)

		#input image from ROS environment to Opencv for image processing 
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		hsv_image=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)#RGB image to HSV since easy for colour related filtering

		#lower and upper bounds for RED Colour in HSV Scale
		lower_red=np.array([170,80,100])#170 ,100
		upper_red=np.array([190,255,255])#192 ,200
		
		#lower and upper bounds for GREEN Colour in HSV Scale
		lower_green=np.array([40,100,100])
		upper_green=np.array([70,255,255])

		#lower and upper bounds for BLUE Colour in HSV Scale
		lower_blue=np.array([110,100,180])#110
		upper_blue=np.array([130,255,255])#130


		#filtering out the unwanted colours with respect to the bounds specified before to generate a mask image/frame
		red=cv2.inRange(hsv_image, lower_red, upper_red)
		green=cv2.inRange(hsv_image,lower_green,upper_green)
		blue=cv2.inRange(hsv_image,lower_blue,upper_blue)


		#earlier generate mask image is applied on top of the original HSV image to give our region of interest
		red_new = cv2.bitwise_and(image,image, mask = red)
		green_new = cv2.bitwise_and(image,image, mask = green)
		blue_new = cv2.bitwise_and(image,image, mask = blue)


		#Blue colour filtering,finding and drawing Contours
		gray_blue = cv2.cvtColor(blue_new, cv2.COLOR_BGR2GRAY)#convert to grayscale image
		ret,thresh_blue = cv2.threshold(gray_blue,50,255,cv2.THRESH_BINARY)#binary thresholding
		contour_image_blue,cnts_blue,hierarchy = cv2.findContours(thresh_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#finding contours
		cv2.drawContours(contour_image_blue,cnts_blue,-1,(0,0,0),2)#drawing contours
		contour_image_new,cnts_new,hierarchy = cv2.findContours(contour_image_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(contour_image_new,cnts_new,-1,(0,0,0),2) #finding and drawing contours done again to remove noise in the image

		#Red colour filtering,finding and drawing Contours
		gray_red = cv2.cvtColor(red_new, cv2.COLOR_BGR2GRAY)#convert to grayscale image
		ret,thresh_red = cv2.threshold(gray_red,140,255,cv2.THRESH_BINARY)#binary thresholding
		#display("red_new",red)
		contour_image_red,cnts_red,hierarchy = cv2.findContours(thresh_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#finding contours
		cv2.drawContours(contour_image_red,cnts_red,-1,(100,100,100),2)#drawing contours

		#Green colour filtering,finding and drawing Contours
		gray_green = cv2.cvtColor(green_new, cv2.COLOR_BGR2GRAY)#convert to grayscale image
		ret,thresh_green = cv2.threshold(gray_green,50,255,cv2.THRESH_BINARY)#binary thresholding
		contour_image_green,cnts_green,hierarchy = cv2.findContours(thresh_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#finding contours
		cv2.drawContours(contour_image_green,cnts_green,-1,(100,100,100),2)#drawing contours
 

		#enclose the region of interest in RED Coloured Rectangle
		xr=10000; yr=10000; cr=0;
		for contour in cnts_red:
			area=cv2.contourArea(contour)
			if(area>0):
				X,Y,W,H=cv2.boundingRect(contour)
				if((X-xr)>500 or (X-xr)<-500):
					cr=cr+1
					xr=X; yr=Y; w=20; h=20;
					cv2.rectangle(image,(xr-10,yr-10),(xr+w+10,yr+h+10),(0,0,255),3)#to draw Red rectangle
				else:
					cv2.rectangle(image,(xr-10,yr-10),(xr+w+10,yr+h+10),(0,0,255),3)#to draw Red rectangle
		
		#enclose the region of interest in GREEN Coloured Rectangle
		xg=10000; yg=10000; cg=0
		for contour in cnts_green:
			area=cv2.contourArea(contour)
			if(area>60):
				X,Y,W,H=cv2.boundingRect(contour)
				if((X-xg)>500 or (X-xg)<-500):
					cg=cg+1
					xg=X; yg=Y; w=20; h=20;
					cv2.rectangle(image,(xg-10,yg-10),(xg+w+10,yg+h+10),(0,255,0),3)#to draw Green rectangle
				else:
					cv2.rectangle(image,(xg-10,yg-10),(xg+w+10,yg+h+10),(0,255,0),3)#to draw Green rectangle
		
		#enclose the region of interest in GREEN BLUE Rectangle
		xb=10000; yb=10000; cb=0
		for contour in cnts_blue:
			area=cv2.contourArea(contour)
			if(area>4):
				X,Y,W,H=cv2.boundingRect(contour)
				if((X-xb)>200 or (X-xb)<-200):
					cb=cb+1;
					xb=X; yb=Y; w=20; h=20;
					cv2.rectangle(image,(xb-10,yb-10),(xb+w+10,yb+h+10),(255,0,0),3)#to draw Blue rectangle
				else:
					cv2.rectangle(image,(xb-10,yb-10),(xb+w+10,yb+h+10),(255,0,0),3)#to draw Blue rectangle
		display("image",image)	#this is the final image output	
		
		#print("R",cr," G",cg," B",cb)
		self.red_flower.publish(cr)
		self.green_flower.publish(cg)
		self.blue_flower.publish(cb)
		
		cv2.waitKey(3)
				

		
if __name__ == '__main__':
	while not rospy.is_shutdown():
		test = WayPoint()
		rospy.spin()
