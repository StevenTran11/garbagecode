#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HW7:
	def __init__(self):
		self.bridge = CvBridge()
		rospy.Subscriber("image", Image, self.callback)
		self.pub1 = rospy.Publisher("image_cropped", Image, queue_size=10)
		self.pub2 = rospy.Publisher("image_white", Image, queue_size=10)
		self.pub3 = rospy.Publisher("image_yellow", Image, queue_size=10)

	def callback(self, msg):
		# convert to a ROS image using the bridge
		cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		#cropping being done
		cv_cropped = cv_img[int(cv_img.shape[0]/2):cv_img.shape[0], 0:cv_img.shape[1]]
		#filtering
		cv_white = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)
		cv_yellow = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)
		#white filter
		sensitivity = 45
		lower_white = np.array([0,0,255-sensitivity])
		upper_white = np.array([255,sensitivity,255])
		mask1 = cv2.inRange(cv_white, lower_white, upper_white)
		#res1 = cv2.bitwise_and(cv_cropped,cv_cropped, mask= mask1)
		#yellow filter
		lower_yellow = np.array([22, 175, 0])
		upper_yellow = np.array([45, 255, 255])
		mask2 = cv2.inRange(cv_yellow, lower_yellow, upper_yellow)
		#res2 = cv2.bitwise_and(cv_cropped,cv_cropped, mask= mask2)
		#erode
		kernel = np.ones((3, 3), np.uint8)
		mask1 = cv2.erode(mask1, kernel, iterations=1)
		#mask2 = cv2.erode(mask2, kernel, iterations=1)
		#dilation
		mask1 = cv2.dilate(mask1, kernel, iterations=3)
		mask2 = cv2.dilate(mask2, kernel, iterations=3)
		#convert to Ros
		ros_cropped = self.bridge.cv2_to_imgmsg(cv_cropped, "bgr8")
		ros_white = self.bridge.cv2_to_imgmsg(mask1, "mono8")
		ros_yellow = self.bridge.cv2_to_imgmsg(mask2, "mono8")
		#publish image
		self.pub1.publish(ros_cropped)
		self.pub2.publish(ros_white)
		self.pub3.publish(ros_yellow)

if __name__ == '__main__':
	rospy.init_node('hw7')
	HW7()
	rospy.spin()
