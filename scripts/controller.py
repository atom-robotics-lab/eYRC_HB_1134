#!/usr/bin/env python3

import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

import numpy as np

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

class hola_bot:
    #initializing the variables
    def __init__(self):

        rospy.init_node('controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odometryCb)
        
        self.pose = []
        self.state = 0
        self.velocity_msg = Twist()
        self.kp = 0.5
        self.theta_precision = 0.5
        self.dist_precision = 0.05

    #callback function for the position
    def odometryCb(self, msg):

        self.hola_x = msg.pose.pose.position.x
        self.hola_y = msg.pose.pose.position.y
        self.hola_theta = [msg.pose.pose.position.x, msg.pose.pose.position.y, euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]]
        # Write your code to take the msg and update the three variables

    #basic move function to give velocity to robot
    def move(self, vel_x , vel_y, vel_z):
        self.velocity_msg.linear.x = vel_x
        self.velocity_msg.linear.y = vel_y
        self.velocity_msg.angular.z = vel_z
        self.pub.publish(self.velocity_msg)

    #move in straight line
    def move_straight(self, error_d):
        self.move(self.kp * error_d, self.kp * error_d, 0)

    #to fix the angle/theta of the bot
    def fix_yaw(self, error_a):
        self.move(0,0,self.kp * error_a)

    def stop(self):
        self.move(0,0,0)

    def main(self, x, y):

        print("this is the main function")
        rospy.loginfo("Main will be our goto function containing the control loop")
        
        goal = False
        while not goal:

            theta_goal = np.arctan((y - self.hola_y)/(x - self.hola_x))   #slope
            if theta_goal>0:
                theta_goal+=0.04
            elif theta_goal<0:
                theta_goal-=0.04
            bot_theta = self.hola_theta  
            theta_error = round(bot_theta - theta_goal, 2)
            rospy.loginfo("THETA ERROR:" + str(theta_error))

            dist_error = math.sqrt(math.pow((x - self.hola_x),2) + math.pow((y - self.hola_y),2))

            if np.abs(theta_error) > self.theta_precision:
                rospy.loginfo("fixing yaw")
                self.fix_yaw(theta_error)
            
            elif dist_error > self.dist_precision:
                rospy.loginfo("moving straight")
                self.move_straight(dist_error)

            else:
                rospy.loginfo("goal reached")
                self.stop()
                goal = True
                return goal

        
        '''
        first we have to calculate the theta error to fix yaw
        then calculate the distance error to move the bot towards goal
        we have to check if we need to only provide linear.x or both linear.x and linear.y
        for it to move straight
        as the main will work as the control loop so we can pass the goal as arguments to it.
        '''


if __name__ == "__main__":
    try:
        bot = hola_bot()
        bot.main(2,2)
    except rospy.ROSInterruptException:
        pass