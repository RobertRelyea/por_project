#!/usr/bin/env python
# Standard imports
import cv2
import numpy as np;
import pdb
import math
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

# ROS Image
omnivisionImage = np.array([])
bridge = CvBridge()
blob_pub = {}
angle_pub = {}

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
    global omnivisionImage, blob_pub, angle_pub
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
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle
        # corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]),
                              (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        # cv2.imshow("Mask", mask)
        # cv2.imshow("Keypoints", im_with_keypoints)
        # cv2.waitKey(1)

        # Publish the image with keypoints
        blobImage = bridge.cv2_to_imgmsg(im_with_keypoints, "bgr8")
        blob_pub.publish(blobImage)
        angle_pub.publish(angle)

# Handles new omnivision image data.
def omnivisionCB(data):
    global omnivisionImage
    try:
        omnivisionImage = cv2.flip(bridge.imgmsg_to_cv2(data, "bgr8"), 1)
        # omnivisionImage = bridge.imgmsg_to_cv2(data, "bgr8")
        # getAngle()
    except CvBridgeError, e:
        rospy.logerr(e)

def omni_blob():
    global blob_pub, angle_pub
    # Starts a new node
    rospy.init_node('omni_blob', anonymous=True)
    blob_pub = rospy.Publisher('omnivision_principles/image_blob', Image, queue_size=10)
    angle_pub = rospy.Publisher('omnivision_principles/blob_angle', Float32, queue_size=10)
    rospy.Subscriber('omnivision_principles/image_raw', Image, omnivisionCB)

    rate = rospy.Rate(15)
    while(not rospy.is_shutdown()):
        getAngle()
        rate.sleep()

if __name__ == '__main__':
    try:
        #Testing our function
        omni_blob()
    except rospy.ROSInterruptException: pass
