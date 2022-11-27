#!/usr/bin/env python3

import rospy
import numpy as np

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Wrench

# for finding sin() cos() 
import math

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion
import signal	

from geometry_msgs.msg import PoseArray

hola_x = 0
hola_y = 0
hola_theta = 0
x_goals, y_goals, theta_goals = [],[],[]
# def signal_handler(sig, frame):
	#   
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	# print('Clean-up !')
	# sys.exit(0)

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
# 

# def odometryCb(msg):
	# global hola_x, hola_y, hola_theta
	# hola_x = msg.pose.pose.position.x
	# hola_y = msg.pose.pose.position.y
	# hola_theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
	# Write your code to take the msg and update the three variables
def aruco_feedback_Cb(msg):
	global hola_x, hola_y, hola_theta
	hola_x = msg.x
	hola_y = msg.y
	hola_theta =msg.theta

def main():
	# Initialze Node
	rospy.init_node('controller')
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"

	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	# rospy.Subscriber('/odom', Odometry, odometryCb)
	# rospy.Subscriber('/task1_goals', PoseArray, task1_goals_Cb)
	# initialising publisher and subscriber of cmd_vel and odom respectively
	rospy.Subscriber('detected_aruco', Pose2D, aruco_feedback_Cb)
	# rospy.Subscriber('/task2_goals', PoseArray, task2_goals_Cb)

	# rospy.wait_for_message("/task2_goals",PoseArray)
	# Declare a Twist message
	# vel = Twist()
	right_wheel_pub = rospy.Publisher(
		'/right_wheel_force', Wrench, queue_size=10)
	wrench1 = Wrench()

	front_wheel_pub = rospy.Publisher(
		'/front_wheel_force', Wrench, queue_size=10)
	wrench2=Wrench()

	left_wheel_pub = rospy.Publisher(
		'/left_wheel_force', Wrench, queue_size=10)
	wrench3=Wrench()


	# Initialise the required variables to 0
	# <This is explained below>

	# For maintaining control loop rate.
	rate = rospy.Rate(100)
	# x_goals,y_goals,theta_goals = [350,50,50,350,250], [350,350,50,50,250], [0.785, 2.335, -2.335, -0.785, 0]
	x_goals,y_goals,theta_goals = [350,  350, 50, 50,250 ], [350, 50,  350, 50,250], [2, -2,  2, -2, 0]
	
	# Initialise variables that may be needed for the control loop
	# x_goals,y_goals,theta_goals = [50,350,50,250,250], [350,50,50,350,50], [0, 0, 0, 0, 0]
	# x_goals,y_goals,theta_goals = [350,250,250,250,350], [50,350,250,250,250], [0, 0, 0, 3, -3]
	#x_goals,y_goals,theta_goals = [1,0,0,0,1], [0,1,0,0,0], [0, 0, 0, 3, -3]
	x_d, y_d, theta_d = x_goals[0],y_goals[0],theta_goals[0]
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller
	kp = 3.0
	# vel_x = 0
	# vel_y = 0
	# vel_z = 0
	count = 0
	temp_vx= 0
	temp_vy= 0
	#
	# 
	# Control Loop goes here
	while not rospy.is_shutdown():
		print("eeeee")

		
		# Find error (in x, y and theta) in global frame
		x_error = (x_d-hola_x)
		y_error = (y_d-hola_y)
		# dist_error = math.sqrt(math.pow((x_d - hola_x),2) + math.pow((y_d - hola_y),2))
		theta_error = (theta_d-hola_theta)
		theta_tolerance = 0.10
		dist_tolerance = 3.0
		# print("errors", x_error,x_d,"-",hola_x,"|", y_error,y_d,"-",hola_y,"|", theta_error)
		# the /odom topic is giving pose of the robot in global frame
		# the desired pose is declared above and defined by you in global frame
		# therefore calculate error in global frame
		
		# (Calculate error in body frame)
		e_g_theta=round(math.atan2(x_error,y_error),2)
		print("----------","hola odom:", hola_x, hola_y, e_g_theta)


		# if np.cos(hola_theta)*(y_d - hola_y) -  np.sin(hola_theta)*(x_d - hola_x) > 0 :
			# theta_error = - theta_error 
		# if hola_x==371:
			# print("OOOOOOOOOOOOOOOOOOOOOOOOO")
			# wrench1.force.x=0
			# wrench2.force.x=0
			# wrench3.force.x=0
# 
			# right_wheel_pub.publish(wrench1)
			# front_wheel_pub.publish(wrench2)
			# left_wheel_pub.publish(wrench3)

		# x = x_error+x_error*abs(math.cos(theta_error))
		# y = y_error+y_error*abs(math.sin(theta_error))
		print("###############",theta_error,"##############")
		if abs(theta_error)<0.3:
			print("TTTTTTTTTTTTTTTTTTTTTTTTTTT")
			e_b_x = math.cos(theta_error)*x_error + math.sin(theta_error)*y_error
			e_b_y = -math.sin(theta_error)*x_error + math.cos(theta_error)*y_error
		elif abs(theta_error)>theta_tolerance:
			e_b_x = math.cos(e_g_theta)*x_error + math.sin(e_g_theta)*y_error
			e_b_y = -math.sin(e_g_theta)*x_error + math.cos(e_g_theta)*y_error
		# e_b_x = math.cos(theta_error)*x_error + math.sin(theta_error)*y_error
		# e_b_y = -math.sin(theta_error)*x_error + math.cos(theta_error)*y_error

		# But for Controller outputs robot velocity in robot_body frame, 
		# i.e. velocity are define is in x, y of the robot frame, 
		# Notice: the direction of z axis says the same in global and body frame
		# therefore the errors will have have to be calculated in body frame.
 
		# This is probably the crux of Task 1, figure this out and rest should be fine.
		
		# Finally implement a P dist_errorcontroller
		if  abs(x_error) < dist_tolerance and abs(y_error) < +dist_tolerance:
			while not abs(theta_error) < theta_tolerance:
				print("\n------------------------------------------------------------------\n")
				print("error:", theta_error)
				print("current theta:", hola_theta, "goal:",theta_d)
				print("\n------------------------------------------------------------------\n")
				B=np.array([[-1,1,0],[-1,-(math.cos(math.pi/3)),-math.sin(math.pi/3)],[-1,-(math.cos(math.pi/3)),(math.sin(math.pi/3))]])
				C=np.array([[theta_error*15],[0*kp],[0*kp]])
				A=np.dot(B,C)
				ary=A.flatten()	
				wrench1.force.x=ary[2]
				wrench2.force.x=ary[0]
				wrench3.force.x=ary[1]

				right_wheel_pub.publish(wrench1)
				front_wheel_pub.publish(wrench2)
				left_wheel_pub.publish(wrench3)

				theta_error = round(theta_d-hola_theta, 3)
				rate.sleep()

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

			print("\n------------------------------------------------------------------\n")
			print("goal reached")
			print("Sleep for 1 sec")
			print("max velocity x:", temp_vx)
			print("max velocity y:", temp_vy)
			print("\n------------------------------------------------------------------\n")
			rospy.sleep(1.5)
			temp_vx=0
			temp_vy=0
			if count < len(x_goals):
				count+=1
				try:
					x_d = x_goals[count]
					y_d = y_goals[count]
					theta_d = theta_goals[count]
				except:
					while True:
						print("Run Completed")
					

			# while not -0.5 < round(hola_theta, 2) < 0.5:
				# print("\n------------------------------------------------------------------\n")
				# print("current theta:", hola_theta, "goal: 0")
				# print("\n------------------------------------------------------------------\n")
				# if hola_theta > 0.5:
					# vel.linear.x = 0
					# vel.linear.y = 0
					# vel.angular.z = -round(hola_theta, 2)*3.5
				# if hola_theta < 0.5:
					# vel.linear.x = 0
					# vel.linear.y = 0
					# vel.angular.z = round(hola_theta, 2)*3.5
				# pub.publish(vel)
				# rate.sleep()

			# vel.linear.x = 0
			# vel.linear.y = 0
			# vel.angular.z = 0
			# pub.publish(vel) 
		else:
			# vel_x = x*kp
			# vel_y = y*kp
			# print("velocities:",vel_x,vel_y)
			# vel_z = 0

			B=np.array([[-1,1,0],[-1,-(math.cos(math.pi/3)),-math.sin(math.pi/3)],[-1,-(math.cos(math.pi/3)),(math.sin(math.pi/3))]])
			C=np.array([[0],[e_b_x*kp],[e_b_y*kp]])
			A=np.dot(B,C)
			ary=A.flatten()	
			wrench1.force.x=ary[2]
			wrench2.force.x=ary[0]
			wrench3.force.x=ary[1]
	
			right_wheel_pub.publish(wrench1)
			front_wheel_pub.publish(wrench2)
			left_wheel_pub.publish(wrench3)

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
		right_wheel_pub.publish(wrench1)
		front_wheel_pub.publish(wrench2)
		left_wheel_pub.publish(wrench3)

		# vel.linear.x = vel_x
		# vel.linear.y = vel_y
		# vel.angular.z = vel_z
		# pub.publish(vel)
		rate.sleep()
	#
	#
main()

# if __name__ == "__main__":
	# try:
		# main()
	# except rospy.ROSInterruptException:
		# pass