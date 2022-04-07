#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HW8:
	def __init__(self):
		self.bridge = CvBridge()
		#subscribers
		sub1 = message_filters.Subscriber("image_cropped", Image)
		sub2 = message_filters.Subscriber("image_white", Image)
		sub3 = message_filters.Subscriber("image_yellow", Image)
		#publishers
		self.pub1 = rospy.Publisher("image_edges", Image, queue_size=10)
		self.pub2 = rospy.Publisher("image_lines_white", Image, queue_size=10)
		self.pub3 = rospy.Publisher("image_lines_yellow", Image, queue_size=10)
		ts = message_filters.TimeSynchronizer([sub1, sub2, sub3], 10)
		ts.registerCallback(self.callback)
		
	def callback(self, msg1, msg2, msg3):
		# convert to a ROS image using the bridge
		cv_img1 = self.bridge.imgmsg_to_cv2(msg1, "bgr8")
		cv_img2 = self.bridge.imgmsg_to_cv2(msg2, "mono8")
		cv_img3 = self.bridge.imgmsg_to_cv2(msg3, "mono8")
		#do canny
		t_lower = 50 # Lower Threshold
		t_upper = 200 # Upper threshold
		aperture_size = 3 # Aperture size
		canny = cv2.Canny(cv_img1,t_lower, t_upper, apertureSize = aperture_size)
		#bitwise and
		bit1 = cv2.bitwise_and(canny, cv_img2, mask = None)
		bit2 = cv2.bitwise_and(canny, cv_img3, mask = None)
		#hough transform
		lines1 = cv2.HoughLinesP(bit1, 1, np.pi/180, 1, None, minLineLength=3, maxLineGap=3)
		lines2 = cv2.HoughLinesP(bit2, 1, np.pi/180, 1, None, minLineLength=1, maxLineGap=3)
		output1 = np.copy(cv_img1)
		if lines1 is not None:
			for i in range(len(lines1)):
				l = lines1[i][0]
				cv2.line(output1, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
		output2 = np.copy(cv_img1)
		if lines2 is not None:
			for i in range(len(lines2)):
				l = lines2[i][0]
				cv2.line(output2, (l[0], l[1]), (l[2], l[3]), (255,0,0), 1, cv2.LINE_AA)
		#convert to Ros
		ros_canny = self.bridge.cv2_to_imgmsg(canny, "8UC1")
		ros1 = self.bridge.cv2_to_imgmsg(output1, "bgr8")
		ros2 = self.bridge.cv2_to_imgmsg(output2, "bgr8")
		#publish image
		self.pub1.publish(ros_canny)
		self.pub2.publish(ros1)
		self.pub3.publish(ros2)
		
if __name__ == '__main__':
	rospy.init_node('hw8')
	HW8()
	rospy.spin()
