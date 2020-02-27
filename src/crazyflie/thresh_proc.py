#!/usr/bin/env python

import rospy
import sys

# opencv import
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv_bridge
from cv_bridge import CvBridge
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Vector3Stamped
from crazyflie.msg import CFData
# from crazyflie.msg import CFImage
from crazyflie.msg import CFCommand
from crazyflie.msg import CFMotion
import time
import matplotlib.pyplot as plt
import os
import imutils

## ORANGE

# LOWBOUNDHSV = (18, 25, 150)
# HIGHBOUNDHSV = (40, 250, 255)

LOWBOUNDHSV = (1, 120, 100)
HIGHBOUNDHSV = (10, 255, 255)

# LOWBOUNDHSV = (19, 0, 0)
# HIGHBOUNDHSV = (40, 255, 255)

## BLUE

# LOWBOUNDHSV = (95, 105, 50)
# HIGHBOUNDHSV = (120, 255, 255)

## GREEN

PLATFORM_HSV_LOW = (70, 40, 30)
PLATFORM_HSV_HIGH = (95, 255, 120)

## STATIC HELPER ## 
def get_morph(image, lowBoundHSV=None, highBoundHSV=None):
    sensitivity = 5

    if lowBoundHSV is None:
        lowBoundHSV = LOWBOUNDHSV
    if highBoundHSV is None:
        highBoundHSV = HIGHBOUNDHSV

    # first blur to remove high frequency noise

    imblur = cv2.GaussianBlur(image, (5, 5), 0)
    hsv = cv2.cvtColor(imblur, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    thresh = cv2.inRange(hsv, lowBoundHSV, highBoundHSV)

    morph = thresh

    # erode and dilate
    morph = cv2.erode(morph, (5,5))
    morph = cv2.dilate(morph, (13,13))

    return morph

class ThreshProc:

    # DO_NOTHING_CMD = CFMotion()

    def __init__(self, in_topic="extcam/image", raw=False):
        self.in_topic = in_topic
        self.raw = raw

        self.bridge = CvBridge()

        #need to facilitate a set of publishers per cf node
        if self.raw:
            self.image_sub = rospy.Subscriber('extcam/image', Image, self.image_raw_cb)
        else:
            self.image_sub = rospy.Subscriber('extcam/image', CompressedImage, self.image_compressed_cb)


    ## CALLBACKS ## 
    def image_raw_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        morph = get_morph(img)
        morph2 = get_morph(img, PLATFORM_HSV_LOW, PLATFORM_HSV_HIGH)
        cv2.imshow("frame", img)
        cv2.imshow("thresh", morph)
        cv2.imshow("thresh_platform", morph)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("image quit")

    def image_compressed_cb(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        morph = get_morph(img)
        morph2 = get_morph(img, PLATFORM_HSV_LOW, PLATFORM_HSV_HIGH)
        cv2.imshow("frame", img)
        cv2.imshow("thresh", morph)
        cv2.imshow("thresh_platform", morph2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("image quit")

    ## THREADS ##
    def run(self):
        rospy.spin()



