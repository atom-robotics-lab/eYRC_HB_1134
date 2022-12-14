#!/usr/bin/env python3
import numpy as np
from numpy import *
from geometry_msgs.msg import Pose2D

# Message type used for publishing force vectors
from geometry_msgs.msg import Wrench
import signal	
import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion,quaternion_from_euler

from geometry_msgs.msg import PoseArray
yaw=0

hola_x = 0
hola_y = 0
hola_theta =0

x_goals, y_goals, theta_goals = [],[],[]
def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	sys.exit(0)


# def task2_goals_Cb(msg):
	# global x_goals, y_goals, theta_goals
	# x_goals.clear()
	# y_goals.clear()
	# theta_goals.clear()
# 
	# for waypoint_pose in msg.poses:
		# x_goals.append(waypoint_pose.position.x)
		# y_goals.append(waypoint_pose.position.y)
# 
		# orientation_q = waypoint_pose.orientation
		# orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		# theta_goal = euler_from_quaternion (orientation_list)[2]
		# theta_goals.append(theta_goal)
# def odom_feedback(msg):
	# global hola_theta , hola_x,hola_y
	# hola_x = msg.pose.pose.position.x
	# hola_y = msg.pose.pose.position.y
# 
	# hola_theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.orientation.z, msg.pose.pose.orientation.w])[2]
	# print("$$$$$$$$$$$$",hola_theta)

def aruco_feedback_Cb(msg):
	global hola_x, hola_y, hola_theta
	hola_x = msg.x
	hola_y = msg.y
	hola_theta =msg.theta
	
	# global hola_theta , hola_x,hola_y,yaw
	# hola_x = msg.pose.pose.position.x
	# hola_y = msg.pose.pose.position.y
	# hola_theta = msg.pose.pose.orientation
	# (roll, pitch, yaw) = quaternion_from_euler(euler_from_quaternion([hola_theta.x,hola_theta.y,hola_theta.z,hola_theta.w]))


	# hola_theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.orientation.z, msg.pose.pose.orientation.w])[2]

	# print(hola_theta)

	# Write your code to take the msg and update the three variables

def main():
	
	# Initialze Node
	rospy.init_node('controller_node')
	signal.signal(signal.SIGINT, signal_handler)

	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"

	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	rospy.Subscriber('detected_aruco', Pose2D, aruco_feedback_Cb)
	# rospy.Subscriber('/odom', Odometry, aruco_feedback_Cb)

	right_wheel_pub = rospy.Publisher(
		'/right_wheel_force', Wrench, queue_size=10)
	wrench1 = Wrench()

	front_wheel_pub = rospy.Publisher(
		'/front_wheel_force', Wrench, queue_size=10)
	wrench2=Wrench()

	left_wheel_pub = rospy.Publisher(
		'/left_wheel_force', Wrench, queue_size=10)
	wrench3=Wrench()
	# hola_theta =msg.theta
	# hola_theta_odom = euler_from_quaternion(
		# [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.orientation.z, msg.pose.pose.orientation.w])[2]

	# rospy.Subscriber('/task2_goals', PoseArray, task2_goals_Cb)

	# initialising publisher and subscriber of cmd_vel and odom respectively

	# rospy.wait_for_message("/task2_goals",PoseArray)

	# Declare a Twist message
	# vel = Twist()
	# Initialise the required variables to 0
	# <This is explained below>

	# For maintaining control loop rate.
	rate = rospy.Rate(100)
	
	# Initialise variables that may be needed for the control loop
	# x_goals,y_goals,theta_goals =[50,350,50,250,250], [350,50,50,350,50], [0, 0, 0, 0, 0]
	# x_goals,y_goals,theta_goals = [350,  350, 50, 50,250 ], [350, 50,  350, 50,250], [3, -3,  3, -3, 0]
	x_goals,y_goals,theta_goals = [350,50,50,350,250], [350,350,50,50,250], [0.785, 2.335, -2.335, -0.785, 0]
	
	# x_goals,y_goals,theta_goals = [1,0,0,0,1], [0,1,0,0,0], [0, 0, 0, 3, -3]
	# x_goals, y_goals, theta_goals = [
	# 1, -1, -1, 1, 0], [1, 1, -1, -1, 0], [0.785, 2.335, -2.335, -0.785, 0]
	# x_goals, y_goals, theta_goals = [50,350,50,250,250], [350,50,50,350,50], [0, 0, 0, 0, 0]
	

	x_d, y_d, theta_d = x_goals[0],y_goals[0],theta_goals[0]

	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller
	kp = 2.4
	ka=	1.2
	# vel_x = 0
	# vel_y = 0
	# vel_z = 0
	count = 0
	# temp_vx= 0
	# temp_vy= 0
	
	# 
	# Control Loop goes here
	while not rospy.is_shutdown():
		global hola_theta_odom

		print(                x_d,            y_d,            theta_d)
		print("odom: " + str(hola_x)+" "+str(hola_y)+" "+str(hola_theta))
		# print(hola_theta_odom)
		dist_error = math.sqrt(math.pow((x_d - hola_x), 2) + math.pow((y_d - hola_y), 2))

		dist_tolerance = 1
		theta_tolerance = 0.100

		# Find error (in x, y and theta) in global frame
		# e_g_theta = abs(theta_d) - abs(yaw)
		e_g_x = x_d - hola_x
		e_g_y = y_d - hola_y
		e_b_theta = round((theta_d) -(hola_theta),3)
		# e_b_theta = 0


		# if (theta_d-abs(hola_theta))>=0.034:
			# e_b_theta = e_g_theta
		# else:
			# e_b_theta=0
		# print("#########",e_b_theta)
		e_g_theta=round(math.atan2(e_g_x,e_g_y),2)
		x = e_g_x+e_g_x*abs(math.cos(e_b_theta))
		y = e_g_y+e_g_y*abs(math.sin(e_b_theta))

		e_b_x = math.cos(e_g_theta)*e_g_x + math.sin(e_g_theta)*e_g_y
		e_b_y = -math.sin(e_g_theta)*e_g_x + math.cos(e_g_theta)*e_g_y
		# vel.linear.x = e_b_x*kp
		# vel.linear.y = e_b_y*kp
		# vel.angular.z = e_b_theta*ka
		# e_g_theta=theta_d-hola_theta
		print("#######################\n")
		print(e_g_theta,"\n")
		print("#########################\n")
		B=np.array([[-1,1,0],[-1,-(math.cos(math.pi/3)),-math.sin(math.pi/3)],[-1,-(math.cos(math.pi/3)),(math.sin(math.pi/3))]])
		C=np.array([[e_b_theta*5],[x*kp],[y*kp]])
		A=np.dot(B,C)
		ary=A.flatten()	
		wrench1.force.x=ary[2]
		wrench2.force.x=ary[0]
		wrench3.force.x=ary[1]

		right_wheel_pub.publish(wrench1)
		front_wheel_pub.publish(wrench2)
		left_wheel_pub.publish(wrench3)


		# publishing the velocity

		rate.sleep()
		print("dist error = " + str(dist_error))

		if abs(dist_error) < dist_tolerance:
			print("if one")
			if abs(e_b_theta) < theta_tolerance:
				print("eazy")
				try:
					print("but don't cry")
					# vel.linear.x = 0
					# vel.linear.y = 0
					# vel.angular.z = 0
					# pub.publish(vel)
					

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



		

			#print("moving towards goal:", x_d, y_d, theta_d)


		# to react to the error with velocities in x, y and theta.
		
		# Safety Check
		# if vel_x > 1.05:
			# vel_x = 1.04
		# elif vel_x < -1.05:
			# vel_x = -1.04
		# temp_vx = max(temp_vx,vel_x)
		# if vel_y > 1.05:
			# vel_y = 1.04
		# elif vel_y < -1.05:
			# vel_y = -1.04
		# temp_vy = max(temp_vy,vel_y)
		# print("\n------velocities after check------\n",vel_x,vel_y,"\n-----------------------------------\n")
		# make sure the velocities are within a range.
		# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
		# we may get away with skipping this step. But it will be very necessary in the long run.
		rate.sleep()
		# vel.linear.x = vel_x
		# vel.linear.y = vel_y
		# vel.angular.z = vel_z
		# pub.publish(vel)
		# rate.sleep()
	#			
		right_wheel_pub.publish(wrench1)
		front_wheel_pub.publish(wrench2)
		left_wheel_pub.publish(wrench3)

	#


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass