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

calibrate = False


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
colorLower = [102, 94, 76]
colorUpper = [131, 183, 255]

def updateHLow(x):
    colorLower[0] = x

def updateSLow(x):
    colorLower[1] = x

def updateVLow(x):
    colorLower[2] = x

def updateHHigh(x):
    colorUpper[0] = x

def updateSHigh(x):
    colorUpper[1] = x

def updateVHigh(x):
    colorUpper[2] = x

if calibrate:
    cv2.namedWindow('image')
    cv2.createTrackbar('H Low', 'image', colorLower[0], 255, updateHLow)
    cv2.createTrackbar('S Low', 'image', colorLower[1], 255, updateSLow)
    cv2.createTrackbar('V Low', 'image', colorLower[2], 255, updateVLow)
    cv2.createTrackbar('H High', 'image', colorUpper[0], 255, updateHHigh)
    cv2.createTrackbar('S High', 'image', colorUpper[1], 255, updateSHigh)
    cv2.createTrackbar('V High', 'image', colorUpper[2], 255, updateVHigh)


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

    # construct a mask for the desired color, then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, np.array(colorLower), np.array(colorUpper))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)



    if calibrate:
        cv2.imshow('image', mask)
        cv2.waitKey(1)
        # blob_pub.publish((bridge.cv2_to_imgmsg(cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB), "bgr8")))

    # Detect blobs.
    keypoints = detector.detect(mask)


    # print(hsv[160,])

    if len(keypoints) > 0:
        y = keypoints[0].pt[1] - 300
        x = keypoints[0].pt[0] - 300
        angle = math.atan2(y, x)

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

cv2.destroyAllWindows()