#!/usr/bin/env python3

import rospy
from mystery_package.msg import UnitsLabelled

class Converter:
	def __init__(self):
		rospy.Subscriber("output2", UnitsLabelled, self.callback)
		self.pub_units = rospy.Publisher("output3", UnitsLabelled, queue_size=10)
		self.total = 0
		self.pub_msg = UnitsLabelled()
		self.pub_msg.units = ""

	def callback(self, msg):
		if rospy.has_param("/unit"):
			self.pub_msg.units = rospy.get_param("/unit")
			if self.pub_msg.units == "Meters":
				self.total = msg.value
			elif self.pub_msg.units == "Feet":
				self.total = msg.value * 3.28
			elif self.pub_msg.units == "Smoots":
				self.total = msg.value * 1.7018
			else:
				self.pub_msg.units = "Meters"
				self.total = msg.value
		else:
			self.pub_msg.units = "Meters"
			self.total = msg.value
		self.pub_msg.value = self.total
		self.pub_units.publish(self.pub_msg)

if __name__ == '__main__':
	rospy.init_node('converter')
	Converter()
	rospy.spin()
