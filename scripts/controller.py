#!/usr/bin/env python3

import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

hola_x = 0
hola_y = 0
hola_theta = 0

def odometryCb(msg):
	global hola_x, hola_y, hola_theta
	hola_x = msg.pose.pose.position.x
	hola_y = msg.pose.pose.position.y
	hola_theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
	# Write your code to take the msg and update the three variables

def main():
	# Initialze Node
	rospy.init_node('controller')
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"

	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('/odom', Odometry, odometryCb)
	# initialising publisher and subscriber of cmd_vel and odom respectively

	# Declare a Twist message
	vel = Twist()
	# Initialise the required variables to 0
	# <This is explained below>

	# For maintaining control loop rate.
	rate = rospy.Rate(100)
	
	# Initialise variables that may be needed for the control loop
	pi = 3.141592653589793238
	x_goals = [1, 0, 0, 0, 1]
	y_goals = [0, 1, 0, 0, 0]
	theta_goals = [0, 0, 0, 3, -3]
	x_d, y_d, theta_d = x_goals[0],y_goals[0],theta_goals[0]
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller
	kp = 0.4
	vel_x = 0
	vel_y = 0
	vel_z = 0
	count = 0
	temp_vx= 0
	temp_vy= 0
	#
	# 
	# Control Loop goes here
	while not rospy.is_shutdown():
		print("hola odom:", hola_x, hola_y, hola_theta)
		# Find error (in x, y and theta) in global frame
		x_error = round(x_d-hola_x, 2)
		y_error = round(y_d-hola_y, 2)
		# dist_error = math.sqrt(math.pow((x_d - hola_x),2) + math.pow((y_d - hola_y),2))
		theta_error = round(theta_d-hola_theta, 2)
		dist_tolerance = 0.04
		print("errors", x_error,x_d,"-",hola_x,"|", y_error,y_d,"-",hola_y,"|", theta_error)
		# the /odom topic is giving pose of the robot in global frame
		# the desired pose is declared above and defined by you in global frame
		# therefore calculate error in global frame
		
		# (Calculate error in body frame)
		x = x_error+x_error*abs(math.cos(theta_error))
		y = y_error+y_error*abs(math.sin(theta_error))
		# But for Controller outputs robot velocity in robot_body frame, 
		# i.e. velocity are define is in x, y of the robot frame, 
		# Notice: the direction of z axis says the same in global and body frame
		# therefore the errors will have have to be calculated in body frame.
 
		# This is probably the crux of Task 1, figure this out and rest should be fine.
		
		# Finally implement a P controller
		if  abs(x_error) < dist_tolerance and abs(y_error) < +dist_tolerance:
			vel.linear.x = 0
			vel.linear.y = 0
			vel.angular.z = 0
			pub.publish(vel)
			print("\n------------------------------------------------------------------\n")
			print("goal reached")
			print("Sleep for 1 sec")
			print("max velocity x:", temp_vx)
			print("max velocity y:", temp_vy)
			print("\n------------------------------------------------------------------\n")
			rospy.sleep(1)
			pub.publish(vel)
			temp_vx=0
			temp_vy=0
			if count < len(x_goals):
				count+=1
				x_d = x_goals[count]
				y_d = y_goals[count]
				theta_d = theta_goals[count] 
		else:
			vel_x = x*kp
			vel_y = y*kp
			print("velocities:",vel_x,vel_y)
			vel_z = 0
			
			#print("moving towards goal:", x_d, y_d, theta_d)


		# to react to the error with velocities in x, y and theta.
		
		# Safety Check
		if vel_x > 0.4:
			vel_x = 0.4
		elif vel_x < -0.4:
			vel_x = -0.4
		temp_vx = max(temp_vx,vel_x)
		if vel_y > 0.4:
			vel_y = 0.4
		elif vel_y < -0.4:
			vel_y = -0.4
		temp_vy = max(temp_vy,vel_y)
		print("\n------velocities after check------\n",vel_x,vel_y,"\n-----------------------------------\n")
		# make sure the velocities are within a range.
		# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
		# we may get away with skipping this step. But it will be very necessary in the long run.
		rate.sleep()
		vel.linear.x = vel_x
		vel.linear.y = vel_y
		vel.angular.z = vel_z
		pub.publish(vel)
		rate.sleep()
	#
	#


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass