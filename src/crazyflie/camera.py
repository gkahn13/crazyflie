#!/usr/bin/env python

import rospy
import sys

# opencv import
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv_bridge
from cv_bridge import CvBridge
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from crazyflie.msg import CFData
# from crazyflie.msg import CFImage
from crazyflie.msg import CFCommand
from crazyflie.msg import CFMotion
import time
import matplotlib.pyplot as plt
import os

from sensor_msgs.srv import SetCameraInfo

FINALX = 1440

class Camera:

    # DO_NOTHING_CMD = CFMotion()

    def __init__(self, ID, cam_id, raw):
        self.id = ID
        self.cam_id = cam_id
        self.raw = raw

        self.bridge = CvBridge()
        self.mat = None

        #need to facilitate a set of publishers per cf node
        if self.raw:
            self.image_pub = rospy.Publisher('cf/%d/image'%self.id, Image, queue_size=10)
        else:
            self.image_pub = rospy.Publisher('cf/%d/image'%self.id, CompressedImage, queue_size=10)
        

        self.comp_image_pub = rospy.Publisher('cf/%d/image/compressed'%self.id, CompressedImage, queue_size=10)
        self.raw_image_pub = rospy.Publisher('cf/%d/image_raw'%self.id, Image, queue_size=10)

        #service
        self.cam_info_service = rospy.Service('cf/%d/camera/set_camera_info'%self.id, SetCameraInfo, self.handle_cam_info)

    ## CALLBACKS ## 


    ## THREADS ##
    def run(self):
        try: 
            print('Camera Node %d starting on cam ID: %d' % (self.id, self.cam_id))
            cap = cv2.VideoCapture(self.cam_id) # TODO: multiple vid captures in parallel
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 192)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 144)
            # cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.8)
            # cap.set(cv2.CAP_PROP_CONTRAST, 0.2)
            # cap.set(cv2.CAP_PROP_EXPOSURE, 0.08)
            # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            
            while not rospy.is_shutdown():
                #print("werfiuweruf")
                ret, frame = cap.read()
                height, width, depth = frame.shape
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # imgscale = FINALX/width
                # newX, newY = width*imgscale, height*imgscale
                # print(width, height)
                # gray = cv2.resize(gray, (int(newX), int(newY)))
                #ret, gray = cap.read()
                rawimg = self.bridge.cv2_to_imgmsg(gray, encoding="mono8")
                rawimg.header.stamp = rospy.Time.now()
                compimg = self.bridge.cv2_to_compressed_imgmsg(gray)
                compimg.header.stamp = rospy.Time.now()
                if self.raw:
                    self.image_pub.publish(rawimg)
                else:
                    self.image_pub.publish(compimg)

                self.raw_image_pub.publish(rawimg)
                
                self.comp_image_pub.publish(compimg)

                cv2.imshow('frame', gray)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            print("CAMERA %d STREAM FAILED -- CHECK INPUTS" % self.id)
            print("Error: " + str(e))

        print(" -- Camera %d Finished -- " % self.id)


    ## CALIBRATION SERVICE ##

    def handle_cam_info(req):
        resp = SetCameraInfoResponse()
        resp.success=True
        resp.status_msg="Handled by Camera class in crazyflie package (no actions taken)"

        # return empty result that shows success
        return resp

