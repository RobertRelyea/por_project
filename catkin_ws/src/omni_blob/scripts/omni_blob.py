#!/usr/bin/env python
# Standard imports
import cv2
import numpy as np;
import pdb
import math
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

# ROS Image
omnivisionImage = np.array([])
bridge = CvBridge()
blob_publisher = {}

# Blob detector parameters
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
# Set up the detector with parameters.
detector = cv2.SimpleBlobDetector_create(detectorParams)
# Upper and lower HSV limits for color mask
redLower = (8, 150, 150)
redUpper = (17, 220, 220)


def getAngle():
    global omnivisionImage
    angle = 0

    if omnivisionImage.shape[0] == 0:
    	return angle

    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = cv2.resize(omnivisionImage, (600, 600))
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

    if len(keypoints) > 0:
    	y = keypoints[0].pt[1] - 300
    	x = keypoints[0].pt[0] - 300
    	angle = math.atan2(y, x)
    # print(hsv[300,300])

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints
    # cv2.imshow("Mask", mask)
    # cv2.imshow("Keypoints", im_with_keypoints)
    # cv2.waitKey(1)

    # Publish the image with keypoints
    blobImage = bridge.cv2_to_imgmsg(im_with_keypoints, "bgr8")
    blob_publisher.publish(blobImage)
    return angle

# Handles new omnivision image data.
def omnivisionCB(data):
    global omnivisionImage
    try:
        omnivisionImage = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        rospy.logerr(e)

def turret():
    global blob_publisher
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    blob_publisher = rospy.Publisher('omnivision/image_blob', Image)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('omnivision/image_raw', Image, omnivisionCB)

    r = rospy.Rate(15)
    while(not rospy.is_shutdown()):
        angle = getAngle()
    	z = angle * 2
    	if z > 0.5:
    		z = 0.5
    	elif z < -0.5:
    		z = -0.5

        vel_msg = Twist()
    	vel_msg.angular.z = z
    	velocity_publisher.publish(vel_msg)

    # Stop the bot
    vel_msg = Twist()
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        turret()
    except rospy.ROSInterruptException: pass
