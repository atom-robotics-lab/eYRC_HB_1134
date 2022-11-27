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
#			[ callback, main ]
# Nodes:		aruco_feedback_node

######################## IMPORT MODULES ##########################

import numpy				# If you find it required
import rospy
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
import math				# If you find it required
# Required to publish ARUCO's detected position & orientation
from geometry_msgs.msg import Pose2D

import imutils


############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D, queue_size=10)
aruco_msg = Pose2D()

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
arucoParams = cv2.aruco.DetectorParameters_create()
vid = cv2.VideoCapture(0)

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code


id = None

center = None
markerID1 = None
radius1 = None
T = 0
dist = 0
count = 0
radians = math.pi/2
global current_frame

def callback(data):

    global cv1_image, current_frame
    # Bridge is Used to Convert ROS Image message to OpenCV image
    br = CvBridge()
    rospy.loginfo("receiving camera frame")

    cv1_image = br.imgmsg_to_cv2(data, "bgr8")

    
    print("control loop")
    # global cv1_image
    Result = aruco_detection(cv1_image)

    cv2.imshow("Frame", Result[4])
    cv2.waitKey(1)

def aruco_detection(image):

    global count, radians

    image = cv1_image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    image = cv2.resize(image, (500,500), interpolation = cv2.INTER_LINEAR)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image,arucoDict, parameters=arucoParams)

    if len(corners) > 0:

        ids = ids.flatten()
        # Creating a circle around the aruco marker and printing its id
        for (markerCorner, markerID) in zip(corners, ids):

            corners = markerCorner.reshape((4, 2))

            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            radius = int(math.sqrt((int(topRight[0]) - int(bottomLeft[0])) ** 2 + (int(topRight[1]) - int(bottomLeft[1])) ** 2) / 2)

            # Center of aruco / bot
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            #center of topleft and topright of aruco
            mx = int((topLeft[0] + topRight[0]) / 2.0)
            my = int((topLeft[1] + topRight[1]) / 2.0)
            
            radians = 1.57+round(math.atan2(my-cY,mx-cX),2)
            print(radians)

            cv2.circle(image, (cX, cY), radius, (0, 0, 255), 3)

            cv2.putText(image, "Aruco Marker ID = " + str(markerID),(topLeft[0] + 20, topLeft[1] - 55),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2)

            center = (((topLeft[0] + bottomRight[0]) / 2.0),((topLeft[1] + bottomRight[1]) / 2.0))

            aruco_msg.x = cX
            aruco_msg.y = cY
            aruco_msg.theta = radians
            aruco_publisher.publish(aruco_msg)

            markerID1 = markerID
            print(markerID1)
            cv2.putText(image, "bot_x = " + str(cX) + ", bot_y = " + str(cY), (cX - 50, cY - 50), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2)
            cv2.putText(image, "radians = " + str(radians), (50, 50), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2)

            radius1 = radius

            dist = math.sqrt(math.pow(cX - 500 , 2) + math.pow(cY - 500 , 2))
            
            rospy.loginfo("bot_x = %f, bot_y = %f", cX, cY)
            rospy.loginfo("Distance = %f", dist)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        exit()

    return [gray, center, radius1, markerID1, image, cX, cY, radians]


def main():
    rospy.init_node('aruco_feedback_node')
    rospy.Subscriber('/overhead_cam/image_raw', Image, callback)
    
    rospy.spin()


if __name__ == '__main__':
    main()