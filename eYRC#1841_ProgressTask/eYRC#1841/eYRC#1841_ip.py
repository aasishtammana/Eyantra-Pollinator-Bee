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

countr=0

class WayPoint:
	
	def __init__(self):
		
		#image processing part
		rospy.init_node('ros_bridge')
		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()
		# Subscribe to whycon image_out
		rospy.Subscriber('/whycon/image_out', Image, self.image_callback)
		


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
		lower_red=np.array([170,100,100])
		upper_red=np.array([192,255,255])
		
		#lower and upper bounds for GREEN Colour in HSV Scale
		lower_green=np.array([40,100,100])
		upper_green=np.array([70,255,255])

		#lower and upper bounds for BLUE Colour in HSV Scale
		lower_blue=np.array([110,100,100])
		upper_blue=np.array([130,255,255])


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
		contour_image_red,cnts_red,hierarchy = cv2.findContours(thresh_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#finding contours
		cv2.drawContours(contour_image_red,cnts_red,-1,(100,100,100),2)#drawing contours

		#Green colour filtering,finding and drawing Contours
		gray_green = cv2.cvtColor(green_new, cv2.COLOR_BGR2GRAY)#convert to grayscale image
		ret,thresh_green = cv2.threshold(gray_green,50,255,cv2.THRESH_BINARY)#binary thresholding
		contour_image_green,cnts_green,hierarchy = cv2.findContours(thresh_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#finding contours
		cv2.drawContours(contour_image_green,cnts_green,-1,(100,100,100),2)#drawing contours
 

		#enclose the region of interest in RED Coloured Rectangle
		for contour in cnts_red:
			area=cv2.contourArea(contour)
			if(area>1):
				countr=countr+1;
				if(countr==1):
					print("\nPollination done! Pollinated 1 Red Daylily")
				x,y,w,h=cv2.boundingRect(contour)
				cv2.rectangle(image,(x-20,y-20),(x+w+25,y+h+20),(0,0,255),3)#to draw Red rectangle
		
		#enclose the region of interest in GREEN Coloured Rectangle
		for contour in cnts_green:
			area=cv2.contourArea(contour)
			if(area>60 and area<200):
				x,y,w,h=cv2.boundingRect(contour)
				cv2.rectangle(image,(x-10,y-10),(x+w+10,y+h+10),(0,255,0),3)#to draw Green rectangle
		
		#enclose the region of interest in GREEN BLUE Rectangle
		for contour in cnts_blue:
			area=cv2.contourArea(contour)
			
			if(area>0):
				x,y,w,h=cv2.boundingRect(contour)
				cv2.rectangle(image,(x-10,y-10),(x+w+10,y+h+10),(255,0,0),3)#to draw blue rectangle
	
		display("image",image)	#this is the final image output	
		
		
		cv2.waitKey(3)
				

		
if __name__ == '__main__':
	while not rospy.is_shutdown():
		test = WayPoint()
		rospy.spin()