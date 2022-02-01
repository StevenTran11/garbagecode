#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Float32

class Publisher:
	def __init__(self):
		self.pub = rospy.Publisher("input", Float32, queue_size=10)
		self.total = 0
		
	def callback(self):
		self.total = random.randint(0,10)
		self.pub.publish(self.total)
		
if __name__ == '__main__':
	try:
		rospy.init_node('publisher')
		p = Publisher()
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			p.callback()
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
		
