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

from sensor_msgs.srv import SetCameraInfo
# from threading import Thread
# from Queue import Queue


N_STABLE = 6
Q_MAX_LEN = 6

WIDTH = 640
HEIGHT = 480

class SimpleTimer:
    def __init__(self, start=False):
        if start:
            self.start_time = time.time()
        self.all_times = []

    def reset(self):
        self.start_time = time.time()
        self.all_times = []

    def record(self):
        self.all_times.append(time.time())

    def format_string(self):
        assert hasattr(self, "start_time") and len(self.all_times) > 0 and self.start_time < self.all_times[0]

        string = ""
        all_with_start = [self.start_time] + self.all_times
        for i,t in enumerate(self.all_times):
            diff = t - all_with_start[i]
            string += "T%d: %.7f, " % (i, diff)

        string += "TOTAL: %.7f" % (self.all_times[-1] - self.start_time)

        return string

class ExternalCamera:

    # DO_NOTHING_CMD = CFMotion()

    def __init__(self, cam_id, raw):
        self.cam_id = cam_id
        self.raw = raw

        self.bridge = CvBridge()
        self.mat = None

        self.normalize = True

        self.is_stable = False
        self.stable_frame_cnt = 0 # don't need to be consecutive
        self.last_rect = (0,0,0)
        self.stable_rect = (0,0,0)
        self.stable_rect_history = []
        self.unstable_frame_cnt = 0 # consecutive

        self.timer = SimpleTimer()

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

    def set_stable(self):
        self.is_stable = True

    def set_not_stable(self):
        self.stable_frame_cnt = 0
        self.is_stable = False
        self.stable_rect = (0,0,0)
        self.stable_rect_history = []

    # can be either interp or hold
    def get_next_stable(self, history, method='hold'):

        if len(history) <= 1 or method == 'hold':
            return history[-1]
        elif method == 'interp':
            size = min(len(history), 4)
            trunc_hist = history[-size:]

            differences_x = [trunc_hist[i+1][0] - trunc_hist[i][0] for i in range(size-1)]
            differences_y = [trunc_hist[i+1][1] - trunc_hist[i][1] for i in range(size-1)]
            differences_area = [trunc_hist[i+1][2] - trunc_hist[i][2] for i in range(size-1)]
            avg_diff_x = sum(differences_x) / len(differences_x)
            avg_diff_y = sum(differences_y) / len(differences_y)

            avg_diff_area = sum(differences_area) / len(differences_area)

            # print("YOLO", differences_x, differences_y, trunc_hist)
            return history[-1][0]+avg_diff_x, history[-1][1]+avg_diff_y, history[-1][2]+avg_diff_area
        else:
            raise NotImplementedError('get next stable invalid method')

    ## HELPERS ##
    def get_target_pix_and_size(self, image):
        sensitivity = 5

        max_pixel_motion = 50 # pixel jump per consecutive frame

        lowBoundRGB = (0, 100, 0)
        highBoundRGB = (150, 255, 150)

        # orange (low light)
        # lowBoundHSV = (1, 175, 20)
        # highBoundHSV = (18, 255, 255)
        # config 2 (high light)
        lowBoundHSV = (1, 80, 20)
        highBoundHSV = (20, 255, 255)
        # first blur to remove high frequency noise

        imblur = cv2.GaussianBlur(image, (7, 7), 0)
        hsv = cv2.cvtColor(imblur, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.inRange(hsv, lowBoundHSV, highBoundHSV)

        morph = thresh

        # erode and dilate
        morph = cv2.erode(morph, (5,5))
        morph = cv2.dilate(morph, (13,13))

        # target = cv2.bitwise_and(gray, gray, mask=morph)
        target_bgr = cv2.bitwise_and(image, image, mask=morph)


        cnts = cv2.findContours(morph, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        final_cnts = []

        ratio = 5./6
        found_list = []
        for c in cnts:
            # cv2.drawContours(image, [c], -1, (255, 0, 0), 2)
            hull = cv2.convexHull(c, returnPoints=True)
            rect = cv2.minAreaRect(hull)
            area = cv2.contourArea(hull)
            w,h = rect[1]
            if w > 1e-10 and h > 1e-10:
                rat = min([w,h]) / max(w,h)
                if area > 1e-10 and abs((w*h) - area) / area < 0.25: # and abs(rat - ratio) / ratio < 0.2:
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(target_bgr,[box],0,(0,0,255),2)
                    cx,cy = rect[0]
                    found_list.append((rect, area))

            # peri = cv2.arcLength(c, True)
            # approx = cv2.approxPolyDP(c, 0.1 * peri, True)
            # looking for rectangles
            # if len(approx) == 4:
            #     try:
            #         M = cv2.moments(c)
            #         cX = int((M["m10"] / M["m00"]))
            #         cY = int((M["m01"] / M["m00"]))
            #         final_cnts.append(c)
            #         cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            #     except ZeroDivisionError as e:
            #         pass
        
        # edges = cv2.Canny(image,100,200)
        # target = cv2.bitwise_and(image, image, mask=target)

        cx = 0
        cy = 0
        area = 0
        for r, a in found_list:
            if a > area:
                cx, cy = r[0]
                cx = int(cx)
                cy = int(cy)
                area = a

        cur_rect = (cx, cy, area)

        # print("CX %.3f, CY: %.3f, A: %.8f" % (cx, cy, area))

        # update consecutive stability if we are close to previous estimate and if we found something this iteration
        if (cur_rect[0] - self.last_rect[0])**2 + (cur_rect[1] - self.last_rect[1])**2 < max_pixel_motion ** 2 and cur_rect != (0,0,0):
            self.stable_frame_cnt += 1
            self.unstable_frame_cnt = 0
            # stable if stable_frame_cnt is bigger than thresh
            if self.stable_frame_cnt > N_STABLE:
                self.set_stable()
            # update stable rect whenever we are within stability
            if self.is_stable:
                self.stable_rect = cur_rect

            # add this rect to history since we received a stable point
            self.stable_rect_history.append(cur_rect)
            # truncate queue
            while len(self.stable_rect_history) > Q_MAX_LEN:
                self.stable_rect_history.pop(0)
        else:
            self.unstable_frame_cnt += 1
            # if we are currently unstable or if we need to switch to unstable
            if not self.is_stable or self.unstable_frame_cnt > N_STABLE:
                self.set_not_stable()
                # print("NOT STABLE")

            if self.is_stable:
                # hold latest stable value
                nextstab = self.get_next_stable(self.stable_rect_history, method='hold')
                # print(self.stable_rect, nextstab)
                self.stable_rect = nextstab
                self.stable_rect_history.append(nextstab)

        # print(len(cnts), len(found_list))

        # print(self.stable_rect_history)
        self.last_rect = cur_rect

        if self.is_stable:
            # green if stable
            cv2.circle(target_bgr, self.stable_rect[:2], 5, (0,255,0), -5)
        else:
            cv2.circle(target_bgr, cur_rect[:2], 5, (0,0,255), -3)

        # first two are pixel x,y and third is blob size and fourth is thresholded img
        return (self.stable_rect[0], self.stable_rect[1], self.stable_rect[2], target_bgr)

    def frame_process(self, frame):
        height, width, depth = frame.shape

        vs = Vector3Stamped()
        vs.header.stamp = rospy.Time.now()
        vs.vector.x, vs.vector.y, vs.vector.z, thresh = self.get_target_pix_and_size(frame)

        if self.normalize:
            vs.vector.x /= WIDTH # 0 to 1
            vs.vector.y /= HEIGHT #  0 to 1
            vs.vector.z /= (WIDTH * HEIGHT) # fractional area

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

        self.target_pos_pub.publish(vs)

        return (('frame', frame), ('thresh', thresh))


    ## CALLBACKS ## 


    ## THREADS ##
    def run(self):
        try: 
            print('External Camera Node starting on cam ID: %d' % (self.cam_id))
            cap = cv2.VideoCapture(self.cam_id)
            # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            # cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.8)
            # cap.set(cv2.CAP_PROP_CONTRAST, 0.2)
            # cap.set(cv2.CAP_PROP_EXPOSURE, 0.08)
            # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            rate = rospy.Rate(20)

            while not rospy.is_shutdown():

                self.timer.reset()
                
                ret, frame = cap.read()
                self.timer.record()

                frame_name_list = self.frame_process(frame)

                self.timer.record()

                for name, frame in frame_name_list:
                    cv2.imshow(name, frame)
                    cv2.imshow(name, frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                rate.sleep()

                # print(self.timer.format_string())


            cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            print("EXTERNAL CAMERA STREAM FAILED -- CHECK INPUTS")
            print("Error: " + str(e))

        print(" -- External Camera Finished -- ")


    ## CALIBRATION SERVICE ##

    def handle_cam_info(req):
        resp = SetCameraInfoResponse()
        resp.success=True
        resp.status_msg="Handled by Camera class in crazyflie package (no actions taken)"

        # return empty result that shows success
        return resp

