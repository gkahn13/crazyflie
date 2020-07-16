import rospy
import os
import os.path
import copy

import random
import time
import math

import matplotlib.pyplot as plt

from crazyflie.msg import CFData, CFMotion, CFCommand
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from sensor_msgs.msg import CompressedImage, Imu, Joy
from std_msgs.msg import Bool

THROTTLE_AXIS = 1 # up 1
ROLL_AXIS = 2 #left 1
PITCH_AXIS = 3 #up 1
YAW_AXIS = 0 #left 1

ALT_AXIS = 5 # D-pad up

DZ_INCREMENT = 0.02
DT = 0.1

#RP motion
THROTTLE_SCALE = 0.4
ROLL_SCALE = -25
PITCH_SCALE = -25
YAW_SCALE = -120

ROLL_CLIP = abs(ROLL_SCALE) * 0.4
PITCH_CLIP = abs(PITCH_SCALE) * 0.1
THROTTLE_CLIP = abs(THROTTLE_SCALE) * 0.2

#standard motion
VX_SCALE = 0.5
VY_SCALE = 0.5

# arbitrary 8% of max is the std dev of noise 
VXY_NOISE_STD = abs(VX_SCALE) * 0.9 # variance (old) 30% of max speed
VZ_NOISE_STD = abs(THROTTLE_SCALE) * 0.5 # variance (old) 20% of max throttle speed

class PointMassCrazyflieSimulator:
    X_RANGE = (0.,1.)
    Y_RANGE = (0.,1.)
    Z_RANGE = (0.,0.5)

    VISUAL_HIST_LEN = 500

    def __init__(self, ros_prefix, dt, use_ros=True, queue_size=10, lag=0, clip=True, with_figure=True, num_latent=1):
        self.dt = dt
        self.use_ros = use_ros
        self.ros_prefix = ros_prefix
        self.num_latent = num_latent
        assert num_latent > 0 and num_latent < 5  # [1,4]
        assert lag >= 0
        self.lag = lag
        self._clip = clip

        if self.use_ros:
            self.motion_pub = rospy.Publisher(ros_prefix + "motion", CFMotion, queue_size=queue_size)
            self.target_pub = rospy.Publisher("extcam/target_vector", Vector3Stamped, queue_size=queue_size)
            self.latent_pub = rospy.Publisher("extcam/latent_vector", Vector3Stamped, queue_size=queue_size)

        if with_figure:
            self.fig = plt.figure(figsize=(16,8))
            self.ax = self.fig.add_subplot(111)
            self.ax.set_xlim(0,1)
            self.ax.set_ylim(1,0)
        # self.plot_elem, = self.ax.plot([], [])
        self.reset()

    def reset(self, target=None, ret_latent=False):
        self.action = CFMotion()
        self.action.is_flow_motion = True

        # choose a latent context randomly
        self._cur_latent = random.choice(range(self.num_latent))

        self.target_state = Vector3Stamped()
        if target is not None:
            self.target_state.vector.x = self.clip(target[0], self.X_RANGE[0], self.X_RANGE[1])
            self.target_state.vector.y = self.clip(target[1], self.Y_RANGE[0], self.Y_RANGE[1])
            self.target_state.vector.z = self.clip(target[2], self.Z_RANGE[0], self.Z_RANGE[1])
        else:
            self.target_state.vector.x = 0.5
            self.target_state.vector.y = 0.5
            self.target_state.vector.z = 0.01

        # state buffer, top corresponds to current state
        self.state_buffer = [copy.deepcopy(self.target_state) for _ in range(self.lag + 1)]

        if ret_latent:
            return self.target_state, self._cur_latent
        return self.target_state

    def get_obs(self):
        return copy.deepcopy(self.state_buffer[0])

    def clip(self, val, low, up):
        return max(low, min(val, up))

    def step(self, action, dt=None):
        if dt is None:
            dt = self.dt

        ax, ay, adz = action.x, action.y, action.dz

        # state update (x = vx * dy)

        if self._cur_latent % 2 == 1:
            ax *= -1  # flip the action
        if self._cur_latent // 2 == 1:
            adz *= -1

        # no clip (temporary)
        if self._clip:
            self.target_state.vector.x = self.clip(self.target_state.vector.x + ax * dt, self.X_RANGE[0], self.X_RANGE[1])
            self.target_state.vector.y = self.clip(self.target_state.vector.y + adz * dt, self.Y_RANGE[0], self.Y_RANGE[1])
            self.target_state.vector.z = self.clip(self.target_state.vector.z + ay * dt, self.Z_RANGE[0], self.Z_RANGE[1])
        else:
            self.target_state.vector.x += ax * dt
            self.target_state.vector.y += adz * dt
            self.target_state.vector.z += ay * dt

        self.state_buffer.append(copy.deepcopy(self.target_state))
        # self.x_history.append(self.target_state.vector.x)
        # self.y_history.append(self.target_state.vector.y)
        self.state_buffer.pop(0)

        return self.get_obs()

    def run_random_motion(self, niter, render=False, correlated=True, realtime=False, rosbag=None):

        if rosbag:
            start_time = rospy.Time.now().to_sec()

        i = 0
        if realtime:
            rate = rospy.Rate(math.ceil(1/self.dt))

        if render:
            print("Rendering...")
            self.ax.cla()

        self.x_history = [self.get_obs().vector.x]
        self.y_history = [self.get_obs().vector.y]

        while i < niter and not rospy.is_shutdown():

            if correlated:
                self.action.x = self.clip(self.action.x + random.uniform(-1, 1) * 0.2 * self.dt, -VX_SCALE, VX_SCALE)
                self.action.y = self.clip(self.action.y + random.uniform(-1, 1) * 0.2 * self.dt, -VX_SCALE, VX_SCALE)
                self.action.dz = self.clip(self.action.dz + random.uniform(-1, 1) * 0.1 * self.dt, -THROTTLE_SCALE, THROTTLE_SCALE)
            else:
                self.action.x = random.uniform(-VX_SCALE, VX_SCALE)
                self.action.y = random.uniform(-VX_SCALE, VX_SCALE)
                self.action.dz = random.uniform(-THROTTLE_SCALE, THROTTLE_SCALE)

            tol = 1e-5

            cur_obs = self.get_obs()

            if self.action.x < tol and cur_obs.vector.x < self.X_RANGE[0] + 0.0001:
                self.action.x = abs(self.action.x) / 2
            if self.action.dz < tol and cur_obs.vector.y < self.Y_RANGE[0] + 0.0001:
                self.action.dz = abs(self.action.dz) / 2
            if self.action.y < tol and cur_obs.vector.z < self.Z_RANGE[0] + 0.0001:
                self.action.y = abs(self.action.y) / 2

            if self.action.x > - tol and cur_obs.vector.x > self.X_RANGE[1] - 0.0001:
                self.action.x = - abs(self.action.x) / 2
            if self.action.dz > - tol and cur_obs.vector.y > self.Y_RANGE[1] - 0.0001:
                self.action.dz = - abs(self.action.dz) / 2
            if self.action.y > - tol and cur_obs.vector.z > self.Z_RANGE[1] - 0.0001:
                self.action.y = - abs(self.action.y) / 2

            if rosbag:
                self._time = rospy.Time.from_sec(start_time + (self.dt * i))
            elif self.use_ros:
                self._time = rospy.Time.now()

            # unique
            ac = copy.deepcopy(self.action)
            ac.stamp.stamp = rospy.Time(self._time.secs, self._time.nsecs)
            cur_obs.header.stamp = rospy.Time(self._time.secs, self._time.nsecs)

            latent_vec = Vector3Stamped()
            latent_vec.vector.x = self._cur_latent

            if self.use_ros:
                self.motion_pub.publish(ac)
                self.target_pub.publish(cur_obs)
                self.latent_pub.publish(latent_vec)

            if rosbag:
                rosbag.write("extcam/target_vector", cur_obs, cur_obs.header.stamp)
                rosbag.write("extcam/latent_vector", latent_vec, cur_obs.header.stamp)
                rosbag.write(self.ros_prefix + "motion", ac, ac.stamp.stamp)
                rosbag._rosbag.flush()

            # updates obs
            next_obs = self.step(self.action, self.dt)

            self.x_history.append(next_obs.vector.x)
            self.y_history.append(next_obs.vector.y)

            if len(self.x_history) > self.VISUAL_HIST_LEN:
                self.x_history.pop(0)
                self.y_history.pop(0)

            i += 1

            if realtime:
                # sleep
                rate.sleep()

            if render:
                self.ax.plot(self.x_history, self.y_history, color='red')
                self.ax.set_xlim(0,1)
                self.ax.set_ylim(1,0)
                plt.draw()
                plt.pause(1e-17)
