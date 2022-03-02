#!/usr/bin/env python3

import rospy
import numpy as np
import math
from duckietown_msgs.msg import Vector2D

class simpleConverter:
	def __init__(self):
		rospy.Subscriber("input_coord", Vector2D, self.callback)
		self.robot = rospy.Publisher("robot_coord", Vector2D, queue_size=10)
		self.world = rospy.Publisher("world_coord", Vector2D, queue_size=10)
		self.ROB = Vector2D()
		self.WOR = Vector2D()
		self.sensorAngle = 180 * math.pi / 180
		self.robotAngle = 135 * math.pi / 180
		self.RPS = np.array([-1,0])
		self.RTS = np.array([[math.cos(self.sensorAngle),-math.sin(self.sensorAngle),self.RPS[0]], [math.sin(self.sensorAngle), math.cos(self.sensorAngle), self.RPS[1]], [0, 0, 1]])
		self.WPR = np.array([3,2])
		self.WTR = np.array([[math.cos(self.robotAngle), -math.sin(self.robotAngle),self.WPR[0]],[math.sin(self.robotAngle), -math.cos(self.robotAngle),self.WPR[1]], [0,0,1]])
		self.SP = np.array([[0],[0],[1]])
		self.WP = np.array([[0],[0],[1]])
		self.sensorPos = np.array([[0],[0],[1]])

	def callback(self, msg):
		self.sensorPos[0] = msg.x
		self.sensorPos[1] = msg.y
		self.SP = np.dot(self.RTS,self.sensorPos)
		self.ROB.x = self.SP[0]
		self.ROB.y = self.SP[1]
		self.robot.publish(self.ROB)
		self.WP = np.dot(self.WTR,self.SP)
		self.WOR.x = self.WP[0]
		self.WOR.y = self.WP[1]
		self.world.publish(self.WOR)
		
if __name__ == '__main__':
	rospy.init_node('SimpleConverter')
	simpleConverter()
	rospy.spin()

