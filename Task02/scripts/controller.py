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

# Team ID:		[ hb_1134]
# Author List:		[ kartik, krrish, harsh, manan ]
# Filename:		feedback.py
# Functions:
#			[ signal_handler, cleanup, task2_goals_Cb, aruco_feedback_Cb, inverse_kinematics, main ]
# Nodes:		controller_node


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

x_goals = []
y_goals = []
theta_goals = []

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None

wrench = Wrench()


##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Not mandatory - but it is recommended to do some cleanup over here,
	#	   to make sure that your logic and the robot model behaves predictably in the next run.
	wrench.force.x = 0
	right_wheel_pub.publish(wrench)
	front_wheel_pub.publish(wrench)
	left_wheel_pub.publish(wrench)
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
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def aruco_feedback_Cb(msg):
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Receive & store the feedback / coordinates found by aruco detection logic.
	#	-> This feedback plays the same role as the 'Odometry' did in the previous task.
	global hola_x, hola_y, hola_theta
	hola_x = msg.x
	hola_y = msg.y
	hola_theta = msg.theta

	############################################


def inverse_kinematics(vel):
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use the target velocity you calculated for the robot in previous task, and
	#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
	#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
	vel_front = -vel[2] + vel[0]
	vel_right = -vel[2] - vel[0]*math.cos(math.pi/3) - vel[1]*math.sin(math.pi/3)
	vel_left = -vel[2] - vel[0]*math.cos(math.pi/3) + vel[1]*math.sin(math.pi/3)

	return (vel_front,vel_right,vel_left)
	
	############################################


def main():

	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)

	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	#	Use the below given topics to generate motion for the robot.
	global right_wheel_pub,front_wheel_pub,left_wheel_pub
	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
	rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)
	rospy.wait_for_message("detected_aruco",Pose2D)
	rospy.wait_for_message("/task2_goals",PoseArray)
	
	rate = rospy.Rate(100)

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Make use of the logic you have developed in previous task to go-to-goal.
	#	-> Extend your logic to handle the feedback that is in terms of pixels.
	#	-> Tune your controller accordingly.
	# 	-> In this task you have to further implement (Inverse Kinematics!)
	#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
	#      given velocity of the chassis (Vx, Vy, W)

	#x_goals,y_goals,theta_goals = [350,50,50,350,250], [350,350,50,50,250], [0.785, 2.335, -2.335, -0.785, 0]

	x_d,y_d,theta_d = x_goals[0],y_goals[0],theta_goals[0]
	dist_tolerance,theta_tolerance = 5, 0.1
	kp,ka = 1.2,1.8
	count = 0

		
	while not rospy.is_shutdown():
		
		# Calculate Error from feedback
		# error in global frame
		if theta_d > 1.47:
			theta_d = theta_d - 1.47
		elif theta_d <= -1.47:
			theta_d = theta_d + 1.47
			
		e_g_theta = theta_d - hola_theta
		e_g_x = x_d - hola_x
		e_g_y = y_d - hola_y


		# Change the frame by using Rotation Matrix (If you find it required)

		# Calculate the required velocity of bot for the next iteration(s)
		e_b_theta = e_g_theta*ka
		e_b_x = (math.cos(hola_theta)*e_g_x + math.sin(hola_theta)*e_g_y)*kp
		e_b_y = (-math.sin(hola_theta)*e_g_x + math.cos(hola_theta)*e_g_y)*kp

		vel = (e_b_x,e_b_y,-e_b_theta)
		#vel = (0,0,-1)
		# Find the required force vectors for individual wheels from it.(Inverse Kinematics)
		
		# Apply appropriate force vectors
		wrench.force.x=inverse_kinematics(vel)[2]
		right_wheel_pub.publish(wrench)
		wrench.force.x=inverse_kinematics(vel)[0]
		front_wheel_pub.publish(wrench)
		wrench.force.x=inverse_kinematics(vel)[1]
		left_wheel_pub.publish(wrench)
		print(theta_d, hola_theta)
		# Modify the condition to Switch to Next goal (given position in pixels instead of meters)
		if abs(e_g_x) < dist_tolerance and abs(e_g_y) < dist_tolerance and abs(e_g_theta) < theta_tolerance:
			cleanup()
			try:
				count +=1 
				time.sleep(2)

				x_d,y_d,theta_d = x_goals[count],y_goals[count],theta_goals[count]
			except:
				print("goal reached")

		rate.sleep()

	############################################

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
