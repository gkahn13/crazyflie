#!/usr/bin/python2

import rospy
import sys

# opencv import
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv_bridge
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image, CompressedImage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3Stamped
from crazyflie.msg import CFMotion
from crazyflie.utils import SimpleTimer
import time
import os
import imutils
import math

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from matplotlib import gridspec

from threading import Lock

WIDTH = 640
HEIGHT = 480

class PendulumVisualization:

    def __init__(self, ID, raw, num_trajectories):
        self._id = ID
        self._raw = raw
        self._num_trajectories = num_trajectories
        self.rosbag_unnormalized = False

        self.bridge = CvBridge()

        self.latest_action = None
        self.latest_ext_img = None
        self.latest_traj_markers = [None] * self._num_trajectories
        self.latest_goal_vector = None
        self.latest_target_vector = None

        self.timer = SimpleTimer(start=True)
        self.timer_lock = Lock()

        self.setup_ros()
        self.setup_figure()


    def setup_ros(self):

        self.marker_sub = rospy.Subscriber('mpc/trajectory_marker', Marker, self.marker_cb)
        self.action_sub = rospy.Subscriber('mpc/action_vector', Vector3Stamped, self.action_cb)
        self.goal_sub = rospy.Subscriber('mpc/goal_vector', Vector3Stamped, self.goal_vector_cb)

        #need to facilitate a set of publishers per cf node
        if self._raw:
            self.image_sub = rospy.Subscriber('extcam/image', Image, self.ext_image_cb)
        else:
            self.image_sub = rospy.Subscriber('extcam/image', CompressedImage, self.ext_image_cb)
            
        self.target_sub = rospy.Subscriber('extcam/target_vector', Vector3Stamped, self.target_cb)
        

    def setup_figure(self):
        # Attaching 3D axis to the figure
        self.fig = plt.figure(figsize=(16,8))
        self.gs = gridspec.GridSpec(ncols=2, nrows=1, figure=self.fig)

        self.axImg = self.fig.add_subplot(self.gs[0,0])
        self.ax3D = self.fig.add_subplot(self.gs[0,1], projection='3d')

        # Setting the Image axes properties
        self.axImg.set_axis_off()
        self.axImg.set_title('Image & Trajectories')

        # Setting the 3D axes properties
        self.ax3D.set_xlim3d([1.0, -1.0])
        self.ax3D.set_xlabel('VX')

        self.ax3D.set_ylim3d([1.0, -1.0])
        self.ax3D.set_ylabel('VY')

        self.ax3D.set_zlim3d([-1.0, 1.0])
        self.ax3D.set_zlabel('VZ')

        self.ax3D.set_title('Action Velocities')
        self.ax3D.view_init(azim=0)
        self.ax3D.scatter([0], [0], [0], marker='+', color='green') # origin

        self.ani = animation.FuncAnimation(self.fig, self.update_figure, interval=50)

        self.img = None
        self.quiv = self.ax3D.quiver([0], [0], [0], [0], [0], [0], color='red')

        plt.show()

    def update_figure(self, frame):
        ### Img axis ###
        if self.latest_ext_img is not None:

            if self._raw:
                dup_img = self.bridge.imgmsg_to_cv2(self.latest_ext_img, "rgb8")
            else:
                dup_img = self.bridge.compressed_imgmsg_to_cv2(self.latest_ext_img, "rgb8")

        else:
            dup_img = np.zeros(shape=[HEIGHT, WIDTH, 3], dtype=np.uint8)

        # draw trajectories on image once we have markers for each trajectory
        if self.latest_target_vector is not None:
            # print(self.latest_target_vector)
            # determine color range

            costs = [None] * len(self.latest_traj_markers)
            for i, traj_marker in enumerate(self.latest_traj_markers):
                if traj_marker is not None:
                    cost = float(traj_marker.text)
                    costs[i] = cost
            color_list = self.map_costs_to_colors(costs)

            for i, traj_marker in enumerate(self.latest_traj_markers):
                if traj_marker is not None:
                    if self.rosbag_unnormalized:
                        prev_pt = (int(self.latest_target_vector.vector.x), int(self.latest_target_vector.vector.y))
                    else:
                        prev_pt = self.convert_to_pixel(self.latest_target_vector.vector.x, self.latest_target_vector.vector.y)

                    for pt in traj_marker.points:
                        next_pt = self.convert_to_pixel(pt.x, pt.y)
                        if next_pt[0] >= WIDTH or next_pt[1] >= HEIGHT or next_pt[0] < 0 or next_pt[1] < 0:
                            break
                        cv2.line(dup_img, prev_pt, next_pt, color_list[i], 2)
                        prev_pt = next_pt
                    self.latest_traj_markers[i] = None

        # draw goal position on image
        if self.latest_goal_vector:
            cv2.circle(dup_img, self.convert_to_pixel(self.latest_goal_vector.vector.x, self.latest_goal_vector.vector.y), 5, (0,0,255), -3)

        # draw updated image
        if self.img is None:
            self.img = self.axImg.imshow(dup_img, animated=True)
        else:
            self.img.set_array(dup_img)
        
        ### 3D axis ###

        # draw action quiver
        if self.latest_action is not None:
            # coordinate system (x forward) (y left) (z up)
            self.quiv.set_segments([[[0,0,0], [self.latest_action.vector.x, self.latest_action.vector.y, self.latest_action.vector.z]]])

    # min cost is green, the rest are various shades of white on a log scale
    def map_costs_to_colors(self, cost_list):
        color_list = [(255,255,255) for _ in range(len(cost_list))]
        # log_costs = [None for _ in range(len(cost_list))]
        min_c = math.inf
        max_c = -math.inf
        min_i = None
        for i, c in enumerate(cost_list):
            if c is not None:
                if c < min_c:
                    min_c = c
                    min_i = i
                if c > max_c:
                    max_c = c
                    max_i = i

        if min_i is not None:
            color_list[min_i] = (0,255,0)

        return color_list



    def convert_to_pixel(self, cx, cy):
       return (int(cx * WIDTH), int(cy * HEIGHT))

    ## CALLBACKS ## 
    def action_cb(self, msg):
        self.latest_action = msg

    def marker_cb(self, msg):
        if msg.type == Marker.LINE_STRIP:
            if msg.id >= self._num_trajectories:
                raise Exception("Marker denotes a trajectory with id at least num_trajectories (%d >= %d)" % (msg.id, self._num_trajectories))
            self.latest_traj_markers[msg.id] = msg

    def ext_image_cb(self, msg):
        self.latest_ext_img = msg

    def goal_vector_cb(self, msg):
        self.latest_goal_vector = msg

    def target_cb(self, msg):
        self.latest_target_vector = msg

    ## THREADS ##
    def run(self):
        rate = rospy.Rate(20) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

        print(" --  PendulumVisualization Finished -- ")


    ## CALIBRATION SERVICE ##

    def handle_cam_info(req):
        resp = SetCameraInfoResponse()
        resp.success=True
        resp.status_msg="Handled by Camera class in crazyflie package (no actions taken)"

        # return empty result that shows success
        return resp

