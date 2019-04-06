import rospy
import numpy as np

import tf2_ros as tf

from crazyflie.msg import CFData
# from crazyflie.msg import CFImage
# from sensor_msgs.msg import Image
from crazyflie.msg import CFCommand
from crazyflie.msg import CFMotion

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped
from math import cos, sin
import math

from struct import pack, unpack

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import threading

import time
import signal
import sys
import os
import re

import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie as CF
from cflib.crazyflie.log import LogConfig

MAX_ALT = 1

## for convenience
cmd_type = ['']*3
cmd_type[CFCommand.ESTOP] = 'ESTOP'
cmd_type[CFCommand.LAND] = 'LAND'
cmd_type[CFCommand.TAKEOFF] = 'TAKEOFF'

# for example, half_cage_8_wash_lr (means 4 washers on left and right holds)
cfg_options = [r'^None$', 
        r'^((half)|(full))_cage(_(\d+)_((wash)|(cork))_((lr)|(left)|(right)))*$']

# exp averaging timer
class FreqTimer:
    def __init__(self, name, alpha=0.99):
        self.name = name
        self.counter = 0
        self.time_start = time.time()
        self.time_last = -1
        self.freq = 0
        self.alpha = alpha

    def new_msg(self):
        if self.counter == 1:
            self.freq = 1/(time.time() - self.time_last) # quick set
        elif self.counter > 0:
            new_freq = 1/(time.time() - self.time_last)
            self.freq = self.alpha*self.freq + (1-self.alpha)*new_freq
        self.counter+=1
        self.time_last = time.time()

    def get(self):
        return self.freq

# Handles all interaction with CF through its radio.
class Crazyflie:

    # ID is for human readability
    def __init__(self, cf_id, radio_uri, data_only=False, motion='None', config='None'):
        self._id = cf_id
        self._uri = radio_uri

        # represents any extra hardware configuration details
        # important for capturing weight / config details
        
        if len([a for a in cfg_options if re.compile(a).match(config)]) == 0:
            print("[WARNING] Config option not recognized: %s" % config)

        self._config = config

        self.stop_sig = False

        # self.data_imu_freq_timer = FreqTimer('DATA')
        # self.data_gyro_freq_timer = FreqTimer('GYRODATA')
        self.data_stab_freq_timer = FreqTimer('STABDATA')
        self.data_kalman_freq_timer = FreqTimer('KALMDATA')

        signal.signal(signal.SIGINT, self.signal_handler)

        self.cf_active = False

        self.accept_commands = False
        self.data_only = data_only
        self.motion = prebuilt_motions[motion];

        self.data = None
        self.alt = 0
        self.to_publish = None

        # self._rec_data_imu = False
        # self._rec_data_gyro = False
        self._rec_data_stab = False
        self._rec_data_kalman = False
        # self._rec_data_pos = False
        # self._rec_data_mag = False


        self.cb_lock = threading.Lock()

        # self.bridge = CvBridge()

        cflib.crtp.init_drivers(enable_debug_driver=False)
        # try:
        # with SyncCrazyflie(self._uri) as scf:

        self.cf = CF(rw_cache="./cache")
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)
        self.cf.connection_failed.add_callback(self.connection_lost)
        self.cf.connection_lost.add_callback(self.connection_failed)

        print('Connecting to %s' % radio_uri)
        self.cf.open_link(radio_uri)

        # self.cf.param.set_value('kalman.resetEstimation', '1')
        # time.sleep(0.1)
        # self.cf.param.set_value('kalman.resetEstimation', '0')
        # time.sleep(1.5)


        # except Exception as e:
        #     print(type(e))
        #     print("Unable to connect to CF %d at URI %s" % (self._id, self._uri))
        #     self.scf = None
        #     self.cf = None

        # STATIC PUBLISHERS
        self.stat_tf_br = tf.StaticTransformBroadcaster()
        sts = TransformStamped()
        sts.header.stamp = rospy.Time.now()
        sts.header.frame_id = "world"
        sts.child_frame_id = "map"

        sts.transform.translation.x = 0
        sts.transform.translation.y = 0
        sts.transform.translation.z = 0

        sts.transform.rotation.x = 0
        sts.transform.rotation.y = 0
        sts.transform.rotation.z = 0
        sts.transform.rotation.w = 1

        self.stat_tf_br.sendTransform(sts)

        # NON STATIC PUBLISHERS
        self.tf_br = tf.TransformBroadcaster()
        self.data_pub = rospy.Publisher('cf/%d/data'%self._id, CFData, queue_size=10)
        self.imu_pub = rospy.Publisher('cf/%d/imu'%self._id, Imu, queue_size=10)
        self.pose_pub = rospy.Publisher('cf/%d/pose'%self._id, PoseStamped, queue_size=10)
        self.twist_pub = rospy.Publisher('cf/%d/twist'%self._id, TwistStamped, queue_size=10)
        # self.image_pub = rospy.Publisher('cf/%d/image'%self._id, Image, queue_size=10)
        if not self.data_only:
            self.cmd_sub = rospy.Subscriber('cf/%d/command'%self._id, CFCommand, self.command_cb)
            self.motion_sub = rospy.Subscriber('cf/%d/motion'%self._id, CFMotion, self.motion_cb)

    def signal_handler(self, sig, frame):
        if self.cf_active:
            self.cmd_estop()
            self.cf.close_link()
        self.stop_sig = True
        rospy.signal_shutdown("CtrlC")

        #killing
        os.kill(os.getpgrp(), signal.SIGKILL)


    def quaternion_from_euler(self, roll, pitch, yaw, deg=False):
        # Abbreviations for the various angular functions
        if deg:
            yaw = yaw * math.pi / 180.0
            pitch = pitch * math.pi / 180.0
            roll = roll * math.pi / 180.0

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        return q


    ## CALLBACKS ##
    def connected(self, uri):
        print("Connected to Crazyflie at URI: %s" % uri)

        self.cf_active = True

        # self.log_imu_data = LogConfig(name="ImuData", period_in_ms=25)
        # self.log_imu_data.add_variable('acc.x', 'FP16')
        # self.log_imu_data.add_variable('acc.y', 'FP16')
        # self.log_imu_data.add_variable('acc.z', 'FP16')
        # self.log_imu_data.add_variable('gyro.x', 'FP16')
        # self.log_imu_data.add_variable('gyro.y', 'FP16')
        # self.log_imu_data.add_variable('gyro.z', 'FP16')

        # try:
        #     self.cf.log.add_config(self.log_imu_data)
        #     self.log_imu_data.data_received_cb.add_callback(self.received_data)
        #     self.log_imu_data.start()
        # except KeyError as e:
        #     print('Could not start log configuration,'
        #           '{} not found in TOC'.format(str(e)))
        # except AttributeError as e:
        #     print('Could not add ImuData log config, bad configuration: %s' % str(e))
        
        self.log_kalman_data = LogConfig(name='KalmanData', period_in_ms=25)
        
        self.log_kalman_data.add_variable('kalman_states.vx', 'FP16')
        self.log_kalman_data.add_variable('kalman_states.vy', 'FP16')
        self.log_kalman_data.add_variable('stateEstimate.x', 'FP16')
        self.log_kalman_data.add_variable('stateEstimate.y', 'FP16')
        self.log_kalman_data.add_variable('stateEstimate.z', 'FP16')
        self.log_kalman_data.add_variable('pm.vbat', 'FP16')
    
        try:
            self.cf.log.add_config(self.log_kalman_data)
            self.log_kalman_data.data_received_cb.add_callback(self.received_data)
            self.log_kalman_data.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError as e:
            print('Could not add KalmanData log config, bad configuration: %s' % str(e))

        self.log_stab_data = LogConfig(name='StabilizerData', period_in_ms=25)
        
        self.log_stab_data.add_variable('stabilizer.pitch', 'FP16')
        self.log_stab_data.add_variable('stabilizer.roll', 'FP16')
        self.log_stab_data.add_variable('stabilizer.yaw', 'FP16')
        # IMU DATA
        self.log_stab_data.add_variable('gyro.x', 'FP16')
        self.log_stab_data.add_variable('gyro.y', 'FP16')
        self.log_stab_data.add_variable('gyro.z', 'FP16')
        self.log_stab_data.add_variable('acc.x', 'FP16')
        self.log_stab_data.add_variable('acc.y', 'FP16')
        self.log_stab_data.add_variable('acc.z', 'FP16')
        # self.log_stab_data.add_variable('pm.vbat', 'FP16')
    
        try:
            self.cf.log.add_config(self.log_stab_data)
            self.log_stab_data.data_received_cb.add_callback(self.received_data)
            self.log_stab_data.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError as e:
            print('Could not add StabilizerData log config, bad configuration: %s' % str(e))
        


        #     # self.log_data_mag = LogConfig(name="MagData", period_in_ms=10)
        #     # self.log_data_mag.add_variable('mag.x', 'float')
        #     # self.log_data_mag.add_variable('mag.y', 'float')
        #     # self.log_data_mag.add_variable('mag.z', 'float')

        #     # self.log_data.add_variable('kalman_states.ox', 'float')
        #     # self.log_data.add_variable('kalman_states.oy', 'float')
        #     # self.log_data.add_variable('motion.deltaX', 'int16_t')
        #     # self.log_data.add_variable('motion.deltaY', 'int16_t')


    def disconnected(self, uri):
        self.cf_active = False
        print("Disconnected from Crazyflie at URI: %s" % uri)

    def connection_failed(self, uri, msg):
        self.cf_active = False
        print("Connection Failed")

    def connection_lost(self, uri, msg):
        self.cf_active = False
        print("Connection Lost")

    def command_cb(self, msg):
        print("ALT: %.3f" % self.alt)
        if self.accept_commands:
            print("RECEIVED COMMAND : %s" % cmd_type[msg.cmd])
            if cmd_type[msg.cmd] == 'ESTOP':
                self.cmd_estop()
            elif cmd_type[msg.cmd] == 'LAND':
                self.alt = 0
                self.cmd_land()
            elif cmd_type[msg.cmd] == 'TAKEOFF':
                self.alt = 0.4
                self.cmd_takeoff()
            else:
                print('Invalid Command! %d' % msg.cmd)
        elif cmd_type[msg.cmd] == 'TAKEOFF':
            self.alt = 0.4
            self.cmd_takeoff()
        else:
            print("Not Accepting Commands, but one was sent: %s" % cmd_type[msg.cmd])

    def motion_cb(self, msg):
        # print("ALT: %.3f" % self.alt)
        # print(msg)
        data = (msg.x, msg.y, msg.yaw, msg.dz)

        if self.accept_commands:
            self.update_alt(msg)
            # switching between optical flow and roll pitch motion
            if msg.is_flow_motion:
                self.set_flow_motion(msg.x, msg.y, msg.yaw, self.alt)
            else:
                self.set_rp_motion(msg.x, msg.y, msg.yaw, self.alt)

        elif any(data):
            # if a nonzero motion message was sent, notify
            print("Not Accepting Motion Commands, but a nonzero "
                + "one was sent: [x:%.2f, y:%.2f, yaw:%.2f, dz:%.2f]" % data)

    def update_alt(self, msg):
        
        #what exactly does this do?
        #motion.alt = self.data.alt * 100 if self.data.alt > ALT_TOLERANCE else 0
        self.alt += msg.dz
        if self.alt < 0:
            self.alt = 0
        elif self.alt > MAX_ALT:
            self.alt = MAX_ALT

    def f32_from_f16(self, float16):
        
        s = int((float16 >> 15) & 0x00000001)    # sign
        e = int((float16 >> 10) & 0x0000001f)    # exponent
        f = int(float16 & 0x000003ff)            # fraction

        if e == 0:
            if f == 0:
                return int(s << 31)
            else:
                while not (f & 0x00000400):
                    f = f << 1
                    e -= 1
                e += 1
                f &= ~0x00000400
                #print(s,e,f)
        elif e == 31:
            if f == 0:
                return int((s << 31) | 0x7f800000)
            else:
                return int((s << 31) | 0x7f800000 | (f << 13))

        e = e + (127 -15)
        f = f << 13
        
        i = int((s << 31) | (e << 23) | f)

        tmp = pack('I',i)
        return unpack('f', tmp)[0]


    def received_data(self, timestamp, data, logconf):
        # print("DATA RECEIVED: CF time: %d, Sys time: %d" % (timestamp, 1000*time.time()))

        if self.to_publish == None:
            self.to_publish = CFData()
            self.to_publish.ID = self._id

        try:
            # if logconf.name == 'ImuData':
            #     self.data = data
            #     self.to_publish.accel_x = self.f32_from_f16(data['acc.x'])
            #     self.to_publish.accel_y = self.f32_from_f16(data['acc.y'])
            #     self.to_publish.accel_z = self.f32_from_f16(data['acc.z'])
            #     self.to_publish.gyro_x = self.f32_from_f16(data['gyro.x'])
            #     self.to_publish.gyro_y = self.f32_from_f16(data['gyro.y'])
            #     self.to_publish.gyro_z = self.f32_from_f16(data['gyro.z'])
            #     # print(data)
            #     # print("DATA")
            #     self.data_imu_freq_timer.new_msg()

            #     self._rec_data_imu = True

            if logconf.name == 'StabilizerData':
                self.to_publish.yaw = self.f32_from_f16(data['stabilizer.yaw'])
                self.to_publish.pitch = self.f32_from_f16(data['stabilizer.pitch'])
                self.to_publish.roll = self.f32_from_f16(data['stabilizer.roll'])
                # IMU
                self.to_publish.accel_x = self.f32_from_f16(data['acc.x'])
                self.to_publish.accel_y = self.f32_from_f16(data['acc.y'])
                self.to_publish.accel_z = self.f32_from_f16(data['acc.z'])
                self.to_publish.gyro_x = self.f32_from_f16(data['gyro.x'])
                self.to_publish.gyro_y = self.f32_from_f16(data['gyro.y'])
                self.to_publish.gyro_z = self.f32_from_f16(data['gyro.z'])

                self._rec_data_stab = True
                # print("STABDATA")
                self.data_stab_freq_timer.new_msg()

            elif logconf.name == 'KalmanData':
                self.to_publish.kalman_vx = self.f32_from_f16(data['kalman_states.vx'])
                self.to_publish.kalman_vy = self.f32_from_f16(data['kalman_states.vy'])
                self.to_publish.pos_x = self.f32_from_f16(data['stateEstimate.x'])
                self.to_publish.pos_y = self.f32_from_f16(data['stateEstimate.y'])
                self.to_publish.alt = self.f32_from_f16(data['stateEstimate.z'])
                self.to_publish.v_batt = self.f32_from_f16(data['pm.vbat'])

                self._rec_data_kalman = True
                # print("KALMDATA")
                self.data_kalman_freq_timer.new_msg()

            # elif logconf.name == 'MagData':
            #     self.to_publish.magx = float(data['mag.x'])
            #     self.to_publish.magy = float(data['mag.y'])
            #     self.to_publish.magz = float(data['mag.z'])
            #     self._rec_data_mag = True
            #     # print("MAGDATA")
            

            # message sending
            if self._rec_data_stab: #self._rec_data_imu:
                self.cb_lock.acquire()
                
                ## ROS COORDINATES
                imu_msg = Imu();
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.angular_velocity = self.quaternion_from_euler(self.to_publish.gyro_y, self.to_publish.gyro_x, self.to_publish.gyro_z)
                imu_msg.linear_acceleration = self.quaternion_from_euler(-self.to_publish.accel_y, self.to_publish.accel_x, self.to_publish.accel_z)
                imu_msg.orientation_covariance[0] = -1 # orientation estimates not present

                # send the imu message
                self.imu_pub.publish(imu_msg)
                self._rec_data_stab = False
                
                # if all data is available, send pose, twist, and cf data
                if self._rec_data_kalman:
                    self.to_publish.ext_config = self._config
                    self.data_pub.publish(self.to_publish)
                    # self.to_publish = None

                    ### THIS IS ALL IN ROS COORDINATES (X: right, Y: forward, Z: up)
                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = "map"
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.pose.position = Vector3(-self.to_publish.pos_y, self.to_publish.pos_x, self.to_publish.alt)
                    pose_msg.pose.orientation = self.quaternion_from_euler(self.to_publish.pitch, self.to_publish.roll, self.to_publish.yaw, deg=True)
                    
                    twist_msg = TwistStamped()
                    twist_msg.header.stamp = rospy.Time.now()
                    twist_msg.twist.linear = Vector3(-self.to_publish.kalman_vy, self.to_publish.kalman_vx, 0)
                    twist_msg.twist.angular = Vector3(self.to_publish.gyro_y, self.to_publish.gyro_x, self.to_publish.gyro_z)

                    self.pose_pub.publish(pose_msg)
                    self.twist_pub.publish(twist_msg)

                    t = TransformStamped()
                    t.transform.translation = pose_msg.pose.position
                    t.transform.rotation = pose_msg.pose.orientation
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "map"
                    t.child_frame_id = "cf"

                    self.tf_br.sendTransform(t)
                    # self.tf_br.sendTransform((-self.to_publish.pos_y, self.to_publish.pos_x, self.to_publish.alt), 
                                # (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w), rospy.Time.now(), 'cf', 'world')
                    # print("RATES: ImuData: %.2f, StabilizerData: %.2f, KalmanData: %.2f"% (self.data_imu_freq_timer.get(), self.data_stab_freq_timer.get(), self.data_kalman_freq_timer.get()))

                    # self._rec_data_stab = False
                    self._rec_data_kalman = False


                self.cb_lock.release()

        except Exception as e:
            print("Exception in received_data:", str(e))

        # d.alt = float(data['posEstimatorAlt.estimatedZ'])

    ## COMMANDS ##

    def set_flow_motion(self, vx, vy, yaw, alt):
        self.cf.commander.send_hover_setpoint(vx, vy, yaw, alt)

    def set_rp_motion(self, roll_a, pitch_a, yaw_r, alt):
        self.cf.commander.send_zdistance_setpoint(roll_a, pitch_a, yaw_r, alt)

    def cmd_estop(self):
        print("---- Crazyflie %d Emergency Stopping ----" % self._id)
        self.cf.commander.send_stop_setpoint()
        self.accept_commands = False

    def cmd_takeoff(self, alt=0.4):
        for y in range(10):
            print("taking off")
            self.cf.commander.send_hover_setpoint(0, 0, 0, y / 10 * alt)
            time.sleep(0.1)
        self.accept_commands = True

    def cmd_land(self, alt=0.4):
        if self.accept_commands==False:
            print("CMD_LAND SENT: cannot land right now")
        else:
            for y in range(10):
                self.cf.commander.send_hover_setpoint(0, 0, 0, alt - (y / 10 * alt))
                time.sleep(0.1)
            self.cmd_estop()

    def sign(self, x):
        return 1 if x >= 0 else -1

    def goto_kp(self, dx, dy, kp=1.0, dt=0.05, TOL=0.05, alt=0.4, vcap=0.5):

        start_x = self.to_publish.pos_x
        start_y = self.to_publish.pos_y

        err_x = (start_x + dx) - self.to_publish.pos_x
        err_y = (start_y + dy) - self.to_publish.pos_y

        while math.sqrt(err_x**2 + err_y**2) > TOL:
            print("ERR:", err_x, ",", err_y)
            vx = min(vcap, kp * abs(err_x)) * self.sign(err_x)
            vy = min(vcap, kp * abs(err_y)) * self.sign(err_y)
            self.cf.commander.send_hover_setpoint(vx, vy, 0, alt)
            time.sleep(dt)

            err_x = (start_x + dx) - self.to_publish.pos_x
            err_y = (start_y + dy) - self.to_publish.pos_y

    # waits for dt * count, stationary motion
    def block_wait(self, count, alt=0.4, dt=0.1):
        for i in range(count):
            self.cf.commander.send_hover_setpoint(0, 0, 0, alt)
            time.sleep(dt)

    def square_motion(self, alt=0.4):
        print("TAKEOFF")
        for y in range(10):
            self.cf.commander.send_hover_setpoint(0, 0, 0, y / 10 * alt)
            time.sleep(0.1)

        dt = 0.1
        TOL = 0.04 #m tolerance
        kp = 1.0
        edge = 0.5

        self.block_wait(30, alt, dt) #3s wait

        print("FIRST")
        self.goto_kp(edge, 0, kp=kp)
        self.block_wait(30, alt, dt) #3s wait

        print("SECOND")
        self.goto_kp(0, -edge, kp=kp)
        self.block_wait(30, alt, dt) #3s wait

        print("THIRD")
        self.goto_kp(-edge, 0, kp=kp)
        self.block_wait(30, alt, dt) #3s wait

        print("FOURTH")
        self.goto_kp(0, edge, kp=kp)
        self.block_wait(30, alt, dt) #3s wait

        print("LAND")
        for y in range(10):
            self.cf.commander.send_hover_setpoint(0, 0, 0, alt - (y / 10 * alt))
            time.sleep(0.1)
        self.cmd_estop()

    def run(self):
        print("WAITING FOR ACTIVE CONNECTION")
        while not self.cf_active:
            time.sleep(0.1)
        print("FOUND ACTIVE CONNECTION")

        #handles image reads
        # threading.Thread(target=self.image_thread).start()

        rate = rospy.Rate(25)

        if self.motion:
            self.motion(self);

        rospy.spin()

        self.log_data.stop()
        if not self.stop_sig:
            self.cmd_estop()
            self.cf.close_link()



prebuilt_motions = {'square': Crazyflie.square_motion, 'takeoff': Crazyflie.cmd_takeoff, 'None': None}
