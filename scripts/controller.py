#!/usr/bin/env python3


# Importing the required libraries
import rospy

from geometry_msgs.msg import Twist  # publishing to /cmd_vel with msg type: Twist

from nav_msgs.msg import Odometry  # subscribing to /odom with msg type: Odometry

import math  # for finding sin(), cos(), square-root and square

from tf.transformations import euler_from_quaternion  # Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle

hola_x = 0
hola_y = 0
hola_theta = 0

# The callback function for the position of the bot
def odometryCb(msg):
    global hola_x, hola_y, hola_theta
    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y
    hola_theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]


def main():
    # Initialze Node named controller
    rospy.init_node('controller')
    

    # Initialzing Publisher and Subscriber for cmd_vel and odometry
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometryCb)

    # Declare a Twist message for the velocity
    vel = Twist()

    # For maintaining control loop rate.
    rate = rospy.Rate(100)
    
    # defining the goal vaariables
    x_d, y_d, theta_d = 2,2,0.785398

    # initializing variable kp for the P-controller
    kp = 1.5

    # Control Loop
    while not rospy.is_shutdown():
        print("odom: " + str(hola_x)+" "+str(hola_y)+" "+str(hola_theta))

        # Find error (in x, y and theta) in global frame
        dist_error = math.sqrt(math.pow((x_d - hola_x),2) + math.pow((y_d - hola_y),2))
        theta_error = round(theta_d - hola_theta, 2)

        theta_tolerance = 0.0174532925
        dist_tolerance = 0.05

        # defining the variables for velocity
        vel_x = 0
        vel_y = 0
        vel_z = 0
        
        # comparing the errors with their tolerance and moving the bot accordingly
        if -theta_tolerance < theta_error < +theta_tolerance:
            if -dist_tolerance < dist_error < +dist_tolerance:
                # both the errors are in their tolerance values hence the goal has been reached
                vel_x = 0
                print("goal reached")
                break

            elif dist_error > +dist_tolerance:
                # There is error in distance so moving the bot in straight line
                vel_x = dist_error*kp

            elif dist_error < -dist_tolerance:
                # There is error in distance so moving the bot in straight line
                vel_x = dist_error*kp

        # There is theta error so rotating the bot along its axis
        elif theta_error > +theta_tolerance:
            vel_z = theta_error*kp

        # There is theta error so rotating the bot along its axis
        elif theta_error < -theta_tolerance:
            vel_z = -theta_error*kp


        vel.linear.x = vel_x
        vel.linear.y = vel_y
        vel.angular.z = vel_z

        # publishing the velocity
        pub.publish(vel)
        rate.sleep()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass