#!/usr/bin/env python3


# Importing the required libraries
import rospy

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
x_goals, y_goals, theta_goals = [], [], []

# The callback function for the position of the bot


def odometryCb(msg):
    global hola_x, hola_y, hola_theta
    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y
    hola_theta = euler_from_quaternion(
        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]


def task1_goals_Cb(msg):
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


def main():
    # Initialze Node named controller
    rospy.init_node('controller')

    # Initialzing Publisher and Subscriber for cmd_vel and odometry
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometryCb)
    rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)

    # Declare a Twist message for the velocity
    vel = Twist()

    rospy.wait_for_message("/task1_goals", PoseArray)

    # For maintaining control loop rate.
    rate = rospy.Rate(100)

    # defining the goal vaariables
    x_d, y_d, theta_d = x_goals[0], y_goals[0], theta_goals[0]
    # x_d, y_d, theta_d = -1, -1, 1

    # initializing variable kp for the P-controller
    kp = 0.60

    # Control Loop
    while not rospy.is_shutdown():
        print("odom: " + str(hola_x)+" "+str(hola_y)+" "+str(hola_theta))

        # Find error (in x, y and theta) in global frame
        dist_error = math.sqrt(
            math.pow((x_d - hola_x), 2) + math.pow((y_d - hola_y), 2))
        theta_error = round(theta_d - hola_theta, 2)

        x_error = round(x_d-hola_x, 2)
        y_error = round(y_d-hola_y, 2)

        x = x_error+x_error*abs(math.cos(theta_error))
        y = y_error+y_error*abs(math.sin(theta_error))

        theta_tolerance = 0.01745
        dist_tolerance = 0.05

        # defining the variables for velocity
        vel_x = 0
        vel_y = 0
        vel_z = 0
        count = 0
        temp_vx = 0
        temp_vy = 0

        rospy.loginfo("theta error: %f, distance error: %f",
                      theta_error, dist_error)

        # comparing the errors with their tolerance and moving the bot accordingly
        if -theta_tolerance < theta_error < +theta_tolerance:
            if -dist_tolerance < dist_error < +dist_tolerance:
                # both the errors are in their tolerance values hence the goal has been reached
                vel_x = 0
                vel_y = 0
                print("goal reached")

                if count < len(x_goals):
                    count += 1
                    try:
                        x_d = x_goals[count]
                        y_d = y_goals[count]
                        theta_d = theta_goals[count]
                    except:
                        print("Run Completed")
                break

            elif abs(dist_error) > dist_tolerance:
                # There is error in distance so moving the bot in straight line
                vel_x = x*kp
                vel_y = y*kp

        # There is theta error so rotating the bot along its axis
        elif theta_error > +theta_tolerance:
            vel_z = theta_error*kp*1.6

        # There is theta error so rotating the bot along its axis
        elif theta_error < -theta_tolerance:
            vel_z = -theta_error*kp*1.6

        if vel_x > 0.55:
            vel_x = 0.55

        elif vel_x < -0.55:
            vel_x = -0.55

            temp_vx = max(temp_vx, vel_x)

        if vel_y > 0.55:
            vel_y = 0.55

        elif vel_y < -0.55:
            vel_y = -0.55

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
