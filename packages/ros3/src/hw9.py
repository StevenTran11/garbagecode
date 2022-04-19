#!/usr/bin/env python3

import sys
import rospy
import time
import math
from std_msgs.msg import Float32

class HW9:
	def __init__(self):
		rospy.Subscriber("error", Float32, self.callback)
		self.pub = rospy.Publisher("control_input", Float32, queue_size=10)
		if rospy.has_param("controller_ready"):
			rospy.set_param("controller_ready", "true")
		if rospy.has_param("Kp"):
			self.kp = rospy.get_param("Kp")
		if rospy.has_param("Ki"):
			self.ki = rospy.get_param("Ki")
		if rospy.has_param("Kd"):
			self.kd = rospy.get_param("Kd")
		self.t = 0
		self.t_prev = 0
		self.err_prev = 0
		self.i = 0
		self.d = 0

	def callback(self, err):
		gain = Float32()
		p = self.kp * err.data
		if self.t != 0:
			self.i = self.i + self.ki * err.data
		if self.t != 0:
			self.d = self.kd * (err.data - self.err_prev)/(self.t - self.t_prev)
		self.err_prev = err.data
		self.t_prev = self.t
		self.t += 0.1
		gain = p + self.i + self.d
		self.pub.publish(gain)
		

if __name__ == '__main__':
	rospy.init_node('hw9')
	HW9()
	rospy.spin()
