#!/usr/bin/env python3


# Importing the required libraries
import numpy as np

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Wrench

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry  # subscribing to /odom with msg type: Odometry

import math  # for finding sin(), cos(), square-root and square

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseArray


hola_x = 0
hola_y = 0
hola_theta = 0
x_goals, y_goals, theta_goals = [
	 -1, -1, 1, 0], [ 1, -1, -1, 0], [ 2.335, -2.335, -0.785, 0]

# The callback function for the position of the bot

# 
def odometryCb(msg):
	global hola_x, hola_y, hola_theta
	hola_x = msg.pose.pose.position.x
	hola_y = msg.pose.pose.position.y
	hola_theta = euler_from_quaternion(
		[msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
# def aruco_feedback_Cb(msg):
	# global hola_theta , hola_x,hola_y
	# hola_x = msg.pose.pose.position.x
	# hola_y = msg.pose.pose.position.y
	# hola_theta =msg.theta


# def task1_goals_Cb(msg):
#     global x_goals, y_goals, theta_goals

#     x_goals.clear()
#     y_goals.clear()
#     theta_goals.clear()

#     for waypoint_pose in msg.poses:
#         x_goals.append(waypoint_pose.position.x)
#         y_goals.append(waypoint_pose.position.y)

#         orientation_q = waypoint_pose.orientation
#         orientation_list = [orientation_q.x,
#                             orientation_q.y, orientation_q.z, orientation_q.w]
#         theta_goal = euler_from_quaternion(orientation_list)[2]
#         theta_goals.append(theta_goal)


def main():
	# rospy.Subscriber('/odom', Odometry, odometryCb)

	right_wheel_pub = rospy.Publisher(
		'/right_wheel_force', Wrench, queue_size=10)
	wrench1 = Wrench()

	front_wheel_pub = rospy.Publisher(
		'/front_wheel_force', Wrench, queue_size=10)
	wrench2=Wrench()

	left_wheel_pub = rospy.Publisher(
		'/left_wheel_force', Wrench, queue_size=10)
	wrench3=Wrench()

	# Initialze Node named controller
	rospy.init_node('controller')

	# Initialzing Publisher and Subscriber for cmd_vel and odometry
	# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('/odom', Odometry, odometryCb)
	# rospy.Subscriber('/detected_aruco', Pose2D, aruco_feedback_Cb)

	# rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)

	# Declare a Twist message for the velocity
	vel = Twist()

	# rospy.wait_for_message("/task1_goals", PoseArray)

	# For maintaining control loop rate.
	rate = rospy.Rate(100)

	# defining the goal vaariables
	x_d, y_d, theta_d = x_goals[0], y_goals[0], theta_goals[0]
	# x_d, y_d, theta_d = -1, -1, 1

	# initializing variable kp for the P-controller
	kp = 0.40

	# Control Loop
	while not rospy.is_shutdown():
		# print("odom: " + str(hola_x)+" "+str(hola_y)+" "+str(hola_theta))
		print("##################")
		print(hola_theta)
		print("##################")

		# Find error (in x, y and theta) in global frame
		dist_error = math.sqrt(
			math.pow((x_d - hola_x), 2) + math.pow((y_d - hola_y), 2))
		e_g_x = x_d - hola_x
		e_g_y = y_d - hola_y

		e_g_theta = round(theta_d - hola_theta, 2)
		e_b_theta = (theta_d) -(hola_theta)

		e_b_x = math.cos(e_b_theta)*e_g_x + math.sin(e_b_theta)*e_g_y
		e_b_y = -math.sin(e_b_theta)*e_g_x + math.cos(e_b_theta)*e_g_y
		B=np.array([[-1,1,0],[-1,-(math.cos(math.pi/3)),-math.sin(math.pi/3)],[-1,-(math.cos(math.pi/3)),(math.sin(math.pi/3))]])
		C=np.array([[e_b_theta*5],[e_b_x*5],[e_b_y*5]])
		A=np.dot(B,C)
		ary=A.flatten()	
		wrench1.force.x=ary[1]*30
		wrench2.force.x=ary[0]*30
		wrench3.force.x=ary[2]*30

		right_wheel_pub.publish(wrench1)
		front_wheel_pub.publish(wrench2)
		left_wheel_pub.publish(wrench3)
		if abs(dist_error) < 0.1:
			print("if one")
			if abs(e_b_theta) < 0.1:
				print("eazy")
				try:
					print("but don't cry")
					wrench1.force.x=0
					wrench2.force.x=0
					wrench3.force.x=0
			
					right_wheel_pub.publish(wrench1)
					front_wheel_pub.publish(wrench2)
					left_wheel_pub.publish(wrench3)
					count +=1 
					# rospy.sleep(5)

					x_d,y_d,theta_d = x_goals[count],y_goals[count],theta_goals[count]
				except:
					print("goal reached")

main()
		# x_error = round(x_d-hola_x, 2)
		# y_error = round(y_d-hola_y, 2)
# 
		# x = x_error+x_error*math.cos(theta_error)
		# y = y_error+y_error*math.sin(theta_error)

		
		