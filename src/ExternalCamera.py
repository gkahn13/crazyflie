#!/usr/bin/python2

import rospy

import cv_bridge
from cv_bridge import CvBridge
import cv2
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

from sensor_msgs.srv import SetCameraInfo

class ExternalCamera:

    # DO_NOTHING_CMD = CFMotion()

    def __init__(self, cam_id, raw):
        self.cam_id = cam_id
        self.raw = raw

        self.bridge = CvBridge()
        self.mat = None

        #need to facilitate a set of publishers per cf node
        if self.raw:
            self.image_pub = rospy.Publisher('extcam/image', Image, queue_size=10)
        else:
            self.image_pub = rospy.Publisher('extcam/image', CompressedImage, queue_size=10)
        

        self.comp_image_pub = rospy.Publisher('extcam/image/compressed', CompressedImage, queue_size=10)
        self.raw_image_pub = rospy.Publisher('extcam/image_raw', Image, queue_size=10)
        
        self.target_pos_pub = rospy.Publisher('extcam/target_vector', Vector3Stamped, queue_size=10)

        #service
        self.cam_info_service = rospy.Service('extcam/set_camera_info', SetCameraInfo, self.handle_cam_info)

    ## HELPERS ##
    def get_target_pix_and_size(self, image):
        sensitivity = 5

        lowBoundRGB = (0, 100, 0)
        highBoundRGB = (150, 255, 150)

        # orange
        lowBoundHSV = (0, 190, 110)
        highBoundHSV = (16, 250, 210)
        # first blur to remove high frequency noise

        imblur = cv2.GaussianBlur(image, (11, 11), 0)
        hsv = cv2.cvtColor(imblur, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv, lowBoundHSV, highBoundHSV)

        morph = thresh

        # erode and dilate
        morph = cv2.erode(morph, (7,7))
        morph = cv2.dilate(morph, (11,11))

        target = cv2.bitwise_and(image, image, mask=thresh)

        # first two are pixel x,y and third is blob size and fourth is thresholded img
        return (0, 0, 0, morph)

    ## CALLBACKS ## 


    ## THREADS ##
    def run(self):
        try: 
            print 'External Camera Node starting on cam ID: %d' % (self.cam_id)
            cap = cv2.VideoCapture(self.cam_id) # TODO: multiple vid captures in parallel
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            # cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.8)
            # cap.set(cv2.CAP_PROP_CONTRAST, 0.2)
            # cap.set(cv2.CAP_PROP_EXPOSURE, 0.08)
            # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            
            while not rospy.is_shutdown():
                #print("werfiuweruf")
                ret, frame = cap.read()
                height, width, depth = frame.shape

                vs = Vector3Stamped()
                vs.header.stamp = rospy.Time.now()
                vs.vector.x, vs.vector.y, vs.vector.z, thresh = self.get_target_pix_and_size(frame)

                rawimg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
                rawimg.header.stamp = rospy.Time.now()
                compimg = self.bridge.cv2_to_compressed_imgmsg(frame)
                compimg.header.stamp = rospy.Time.now()
                if self.raw:
                    self.image_pub.publish(rawimg)
                else:
                    self.image_pub.publish(compimg)

                self.raw_image_pub.publish(rawimg)
                
                self.comp_image_pub.publish(compimg)

                cv2.imshow('frame', frame)
                cv2.imshow('thresh', thresh)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            print "EXTERNAL CAMERA STREAM FAILED -- CHECK INPUTS"
            print "Error: " + str(e)

        print " -- External Camera Finished -- "


    ## CALIBRATION SERVICE ##

    def handle_cam_info(req):
        resp = SetCameraInfoResponse()
        resp.success=True
        resp.status_msg="Handled by Camera class in crazyflie package (no actions taken)"

        # return empty result that shows success
        return resp

