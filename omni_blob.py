#!/usr/bin/env python
# Standard imports
import cv2
import numpy as np;
import pdb
import math
import rospy
from geometry_msgs.msg import Twist
 
# Read image
cap = cv2.VideoCapture('/dev/video1')
cap.set(3, 1440)
cap.set(4, 1440)



detectorParams = cv2.SimpleBlobDetector_Params()
detectorParams.thresholdStep = 20
detectorParams.minThreshold = 200
detectorParams.maxThreshold = 255
detectorParams.minDistBetweenBlobs = 100

detectorParams.filterByArea = True
detectorParams.minArea =1
detectorParams.maxArea = 10000

detectorParams.filterByConvexity = False
detectorParams.minConvexity = 0.3
detectorParams.maxConvexity = 10

detectorParams.filterByInertia = False
detectorParams.minInertiaRatio = 0.01
detectorParams.maxInertiaRatio = 10

detectorParams.filterByColor = True
detectorParams.blobColor = 255

detectorParams.filterByCircularity = False
detectorParams.minCircularity = 0.9
detectorParams.maxCircularity = 1

# Set up the detector with default parameters.
detector = cv2.SimpleBlobDetector_create(detectorParams)

redLower = (8, 150, 150)
redUpper = (17, 220, 220)
# 36 255 192
# 22 165 140
def getAngle():
	angle = 0

	ret, frame = cap.read()
	if frame.shape[0] == 0:
		return angle


	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = cv2.resize(frame, (600, 600))
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
 
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, redLower, redUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# Detect blobs.
	keypoints = detector.detect(mask)

	# print(len(keypoints))
	# if len(keypoints) > 0:
	# 	pdb.set_trace()

	if len(keypoints) > 0:
		y = keypoints[0].pt[1] - 300
		x = keypoints[0].pt[0] - 300
		angle = math.atan2(y, x)
	# print(hsv[300,300])

	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	# Show keypoints
	cv2.imshow("Mask", mask)
	cv2.imshow("Keypoints", im_with_keypoints)
	cv2.waitKey(1)
	return angle

def turret():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    r = rospy.Rate(10)
    while(not rospy.is_shutdown()):
    	angle = getAngle()
    	vel_msg = Twist()
    	z = angle * 2
    	if z > 0.5:
    		z = 0.5
    	elif z < -0.5:
    		z = -0.5

    	vel_msg.angular.z = z
    	print(vel_msg.angular.z)
    	velocity_publisher.publish(vel_msg)

    # Stop the bot
    vel_msg = Twist()
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        turret()
    except rospy.ROSInterruptException: pass