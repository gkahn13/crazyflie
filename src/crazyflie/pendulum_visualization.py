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

    def __init__(self, ID, dt, raw, num_trajectories):
        self._id = ID
        self._raw = raw
        self._dt = dt
        self._num_trajectories = num_trajectories
        self.rosbag_unnormalized = False

        self.bridge = CvBridge()

        self.latest_action = None
        self.latest_ext_img = None
        self.latest_traj_markers = [None] * self._num_trajectories
        self.latest_ac_marker = None # rollout of best action seq
        self.latest_goal_vector = None
        self.latest_target_vector = None
        self.latest_latent_vector = None

        self.timer = SimpleTimer(start=True)
        self.timer_lock = Lock()

        self.latent_lock = Lock()

        self.setup_ros()
        self.setup_figure()


    def setup_ros(self):

        self.marker_sub = rospy.Subscriber('mpc/trajectory_marker', Marker, self.marker_cb)
        self.action_sub = rospy.Subscriber('mpc/action_vector', Vector3Stamped, self.action_cb)
        self.ac_marker_sub = rospy.Subscriber('mpc/action_marker', Marker, self.ac_marker_cb)
        self.goal_sub = rospy.Subscriber('mpc/goal_vector', Vector3Stamped, self.goal_vector_cb)
        self.latent_sub = rospy.Subscriber('mpc/latent_vector', Vector3Stamped, self.latent_vector_cb)

        #need to facilitate a set of publishers per cf node
        if self._raw:
            self.image_sub = rospy.Subscriber('extcam/image', Image, self.ext_image_cb)
        else:
            self.image_sub = rospy.Subscriber('extcam/image', CompressedImage, self.ext_image_cb)
            
        self.target_sub = rospy.Subscriber('extcam/target_vector', Vector3Stamped, self.target_cb)
        

    def setup_figure(self):
        # Attaching 3D axis to the figure
        self.fig = plt.figure(figsize=(16,8))
        self.gs = gridspec.GridSpec(ncols=3, nrows=3, figure=self.fig)

        self.axImg = self.fig.add_subplot(self.gs[:2,0])
        self.ax3D = self.fig.add_subplot(self.gs[:2,1], projection='3d')
        self.ax_updown = self.fig.add_subplot(self.gs[0,2])
        self.ax_leftright = self.fig.add_subplot(self.gs[1,2])
        self.ax_forwardback = self.fig.add_subplot(self.gs[2,2])

        self.ax_latent_mean = self.fig.add_subplot(self.gs[2,:2])
        # self.ax_latent_std = self.fig.add_subplot(self.gs[2,1])

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

        self.ax_updown.set_title("Up Down") # vertical
        self.ax_updown.set_ylim([-.1, .1])

        self.ax_leftright.set_title("Left Right") # horizontal
        self.ax_leftright.set_xlim([-1., 1.])

        self.ax_forwardback.set_title("Forward Back")
        self.ax_forwardback.set_ylim([-1., 1.]) # vertical

        self.ax_latent_mean.set_title("Latent mean") # vertical
        self.ax_latent_mean.set_ylim([-2, 2])
        self.all_latent = []
        self.all_latent_upper = []
        self.all_latent_lower = []
        self.ax_latent_mean_plot, = self.ax_latent_mean.plot([0], [0], color='blue', marker='.')
        self.new_latent = False

        # self.ax_latent_mean_bar = self.ax_latent_mean.bar([0], [0])

        # self.ax_latent_std.set_title("Latent std") # vertical
        # self.ax_latent_std.set_ylim([0, 3])
        # self.ax_latent_std_bar = self.ax_latent_std.bar([0], [0])

        self.ani = animation.FuncAnimation(self.fig, self.update_figure, blit=False, repeat=False, interval=int(self._dt * 1000))

        # objects
        self.img = None
        self.quiv = self.ax3D.quiver([0], [0], [0], [0], [0], [0], color='red')

        self.updown_barv = None
        self.leftright_barh = None
        self.forwardback_barv = None
        # import ipdb; ipdb.set_trace()

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
            center = self.convert_to_pixel(self.latest_target_vector.vector.x, self.latest_target_vector.vector.y)
            side = np.sqrt(self.latest_target_vector.vector.z * WIDTH * HEIGHT)

            upperleft = (int(center[0] - side/2), int(center[1] - side/2))
            bottomright = (int(center[0] + side/2), int(center[1] + side/2))
            cv2.rectangle(dup_img, upperleft, bottomright, (0,255,0), 3)

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
            goal_pt = self.convert_to_pixel(self.latest_goal_vector.vector.x, self.latest_goal_vector.vector.y)
            side = np.sqrt(self.latest_goal_vector.vector.z * WIDTH * HEIGHT)
            upperleft = (int(goal_pt[0] - side/2), int(goal_pt[1] - side/2))
            bottomright = (int(goal_pt[0] + side/2), int(goal_pt[1] + side/2))
            cv2.rectangle(dup_img, upperleft, bottomright, (0,0,255), 3)
            cv2.circle(dup_img, goal_pt, 5, (0,0,255), -3)

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

        if self.latest_ac_marker is not None:
            # each is a vector of the action seq
            # coordinate system (x forward) (y left) (z up)
            ud = []
            lr = []
            fb = []
            for ac in self.latest_ac_marker.points:
                ud.append(ac.z)
                lr.append(ac.y)
                fb.append(ac.x)

            H = len(ud)
            if self.updown_barv is None:
                print("Creating bars")
                self.updown_barv = self.ax_updown.bar(list(range(H)), ud)
                self.leftright_barh = self.ax_leftright.barh(list(range(H)), lr)
                self.forwardback_barv = self.ax_forwardback.bar(list(range(H)), fb)
            else:
                for i in range(H):
                    self.updown_barv[i].set_height(ud[i])
                    self.leftright_barh[i].set_width(lr[i])
                    self.forwardback_barv[i].set_height(fb[i])

        # draw latent
        self.latent_lock.acquire()
        if self.new_latent:
            self.new_latent = False
            self.latent_lock.release()
            mean = self.latest_latent_vector.vector.x
            std = self.latest_latent_vector.vector.y

            self.all_latent.append(mean)
            self.all_latent_upper.append(mean + std)
            self.all_latent_lower.append(mean - std)

            self.ax_latent_mean.collections.clear()
            x = range(len(self.all_latent))
            self.ax_latent_mean_plot.set_xdata(x)
            self.ax_latent_mean_plot.set_ydata(self.all_latent)
            self.ax_latent_mean.fill_between(x, self.all_latent_lower, self.all_latent_upper, facecolor='blue', alpha=0.4)
        else:
            self.latent_lock.release()
            self.all_latent.clear()
            self.all_latent_upper.clear()
            self.all_latent_lower.clear()

            # self.ax_latent_mean_bar[0].set_height(mean)
            # self.ax_latent_std_bar[0].set_height(std)

        # print(time.time())

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

    def ac_marker_cb(self, msg):
        if msg.type == Marker.LINE_STRIP:
            self.latest_ac_marker = msg

    def ext_image_cb(self, msg):
        self.latest_ext_img = msg

    def goal_vector_cb(self, msg):
        self.latest_goal_vector = msg

    def target_cb(self, msg):
        self.latest_target_vector = msg

    def latent_vector_cb(self, msg):
        self.latent_lock.acquire()
        self.new_latent = True
        self.latent_lock.release()
        self.latest_latent_vector = msg

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

