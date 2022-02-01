#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Float32

class Listener:
	def __init__(self):
		rospy.Subscriber("output1", Float32, self.callback)
		
	def callback(self, msg):
		rospy.loginfo("mystery_node published " + str(msg.data))
		
if __name__ == '__main__':
	rospy.init_node('listener')
	Listener()
	rospy.spin()
