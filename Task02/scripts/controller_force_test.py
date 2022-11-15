#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
# [ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user
import numpy as np
from numpy import *

# Message type used for publishing force vectors
from geometry_msgs.msg import Wrench
# Message type used for receiving goals
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback
from std_msgs.msg import Float64


import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion  # Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

x_goals = []
y_goals = []
theta_goals = []

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None


##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):

	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal
	print('Clean-up !')
	cleanup()
	sys.exit(0)


def cleanup():
	print("uwu")
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP :
	# -> Not mandatory - but it is recommended to do some cleanup over here,
	# to make sure that your logic and the robot model behaves predictably in the next run.

	############################################
def task2_goals_Cb(msg):
	global x_goals, y_goals, theta_goals
	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x,
							orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion(orientation_list)[2]
		theta_goals.append(theta_goal)


def aruco_feedback_Cb(msg):
	print("uwuw")
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP :
	# -> Receive & store the feedback / coordinates found by aruco detection logic.
	# -> This feedback plays the same role as the 'Odometry' did in the previous task.

	############################################


def inverse_kinematics():
	print("uwuw")
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP :
	# -> Use the target velocity you calculated for the robot in previous task, and
	# Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
	# Publish the calculated efforts to actuate robot by applying force vectors on provided topics
	############################################

def main():
	
	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)
	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	# Use the below given topics to generate motion for the robot.
	right_wheel_pub = rospy.Publisher(
		'/right_wheel_force', Wrench, queue_size=10)
	wrench1 = Wrench()

	front_wheel_pub = rospy.Publisher(
		'/front_wheel_force', Wrench, queue_size=10)
	wrench2=Wrench()

	left_wheel_pub = rospy.Publisher(
		'/left_wheel_force', Wrench, queue_size=10)
	wrench3=Wrench()
	x=np.float64(0)
	y=Float64(0)
	z=np.float



	a = 0
	while True:
		a = a+1
		# A=np.matrix([[wrench1.force.x],
		# 			[wrench2.force.x],
		# 			[wrench3.force.x]])
		# print("WRENCH-----------",A)
		# A=[[x],
			# [y],
			# [z]
			# ]
		B=np.array([[-0.33,1,0],[-0.33,-(math.cos(math.pi/3)),-math.sin(math.pi/3)],[-0.33,-(math.cos(math.pi/3)),(math.sin(math.pi/3))]])
		C=np.array([[0],[50],[50]])
		# B=[[-0.33,0.58,0.33],[-0.33,0.58,0.33],[0.67,0,0.33]]

		A=np.dot(B,C)
		# wrench1.force.x=(-(math.sin(math.pi/3))*(math.cos(math.pi/3))*10+math.cos(2*(math.pi/3))*10+10)
		# wrench2.force.x=(-(math.sin(math.pi/3))*(math.cos(math.pi/3))*10+math.cos(2*(math.pi/3))*10+10)
		# wrench1.force.x=-10+10
		# wrench2.force.x=-10-math.cos(math.pi/3)*10-math.sin(math.pi/3)*10
		# wrench2.force.x=-10-math.cos(math.pi/3)*10-math.sin(math.pi/3)*10


		# print("DOT-------",A)
		print(A)
		ary=A.flatten()
		print(ary)
		print(len(ary))
				
		wrench1.force.x=ary[1]
		wrench2.force.x=ary[0]
		wrench3.force.x=ary[2]
		print("##########################################")

		# print(wrench1,wrench2,wrench3)
		# print(wrench1+"||"+wrench2)
		# wrench1=x
		# wrench2-y
		# wrench3=z
			# wrench1.force.x = 100
			# wrench1.force.y = -100
		right_wheel_pub.publish(wrench1)

		# wrench2.force.x = -100
		# wrench3.force.y = -100.1
		front_wheel_pub.publish(wrench2)
		# wrench3.force.x = 0

		left_wheel_pub.publish(wrench3)


		# wrench.torque.x = 0.001
		# right_wheel_pub.publish(wrench)
		# wrench.torque.y = 0.001
		# left_wheel_pub.publish(wrench)
		# wrench.torque.z = 0.001
		# front_wheel_pub.publish(wrench)

	rospy.Subscriber('detected_aruco', Pose2D, aruco_feedback_Cb)
	rospy.Subscriber('task2_goals', PoseArray, task2_goals_Cb)

	rate = rospy.Rate(100)

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP :
	# -> Make use of the logic you have developed in previous task to go-to-goal.
	# -> Extend your logic to handle the feedback that is in terms of pixels.
	# -> Tune your controller accordingly.
	# 	-> In this task you have to further implement (Inverse Kinematics!)
	#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
	#      given velocity of the chassis (Vx, Vy, W)
	#

	# while not rospy.is_shutdown():

		# Calculate Error from feedback

		# Change the frame by using Rotation Matrix (If you find it required)

		# Calculate the required velocity of bot for the next iteration(s)

		# Find the required force vectors for individual wheels from it.(Inverse Kinematics)

		# Apply appropriate force vectors

		# Modify the condition to Switch to Next goal (given position in pixels instead of meters)

		# rate.sleep()

	############################################
main()

# if __name__ == "__main__":
# 	try:
# 		main()
# 	except rospy.ROSInterruptException:
# 		pass
