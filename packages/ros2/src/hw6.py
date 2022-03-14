#!/usr/bin/env python3

import rospy
import math
from odometry_hw.msg import DistWheel
from odometry_hw.msg import Pose2D

class HW6:
	def __init__(self):
		rospy.Subscriber("dist_wheel", DistWheel, self.callback)
		self.pub = rospy.Publisher("pose", Pose2D, queue_size=10)
		self.x = 0
		self.y = 0
		self.theta = 0
		self.dx = 0
		self.dy = 0
		self.dtheta = 0
		self.ds = 0
		self.L = 0.05
		self.pub_msg = Pose2D()
		

	def callback(self, msg):
		self.ds = (msg.dist_wheel_left + msg.dist_wheel_right) / 2
		self.dtheta = (msg.dist_wheel_right - msg.dist_wheel_left) / (2 * self.L)
		self.dx = self.ds * math.cos(self.theta + (self.dtheta / 2))
		self.dy = self.ds * math.sin(self.theta + (self.dtheta / 2))
		self.x += self.dx
		self.y += self.dy
		self.theta += self.dtheta
		self.pub_msg.x = self.x
		self.pub_msg.y = self.y
		self.pub_msg.theta = self.theta
		self.pub.publish(self.pub_msg)

if __name__ == '__main__':
	rospy.init_node('hw6')
	HW6()
	rospy.spin()
