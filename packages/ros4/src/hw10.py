#!/usr/bin/env python3

import rospy
import time
import actionlib
import example_action_server.msg
from example_service.srv import Fibonacci, FibonacciResponse


def service(x):
	rospy.wait_for_service('calc_fibonacci')
	try:
		srv = rospy.ServiceProxy('calc_fibonacci',Fibonacci)
		response = srv(x)
		return response
	except:
		print("Failed")	

def action(x):
	client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
	client.wait_for_server()
	goal = example_action_server.msg.FibonacciGoal(order=x)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()

if __name__ == '__main__':
	rospy.init_node('hw10')
	try:
		start_time = time.time()
		result = action(3)
		rospy.loginfo("Action Total Time: " + str(time.time() - start_time) + "s")
	except rospy.ROSInterruptException:
		print("program interrupted before completion", file=sys.stderr)
	try:
		start_time = time.time()
		result = action(15)
		rospy.loginfo("Action Total Time: " + str(time.time() - start_time) + "s")
	except rospy.ROSInterruptException:
		print("program interrupted before completion", file=sys.stderr)
	try:
		start_time = time.time()
		result = service(3)
		rospy.loginfo("Service Total Time: " + str(time.time() - start_time) + "s")
	except rospy.ROSInterruptException:
		print("program interrupted before completion", file=sys.stderr)
	try:
		start_time = time.time()
		result = service(15)
		rospy.loginfo("Service Total Time: " + str(time.time() - start_time) + "s")
	except rospy.ROSInterruptException:
		print("program interrupted before completion", file=sys.stderr)
