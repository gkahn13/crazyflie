import rospy
import numpy as np

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
from math import cos, sin

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import threading

import time
import signal
import sys
import os

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

# Handles all interaction with CF through its radio.
class Crazyflie:

    # ID is for human readability
    def __init__(self, cf_id, radio_uri, data_only=False):
        self._id = cf_id
        self._uri = radio_uri

        self.stop_sig = False

        signal.signal(signal.SIGINT, self.signal_handler)

        self.cf_active = False

        self.accept_commands = False
        self.data_only = data_only

        self.data = None
        self.alt = 0
        self.to_publish = None

        self._rec_data = False
        self._rec_data_gyro = False
        self._rec_data_stab = False
        self._rec_data_kalman = False
        self._rec_data_pos = False
        self._rec_data_mag = False

        self._imu_rec_acc = False
        self._imu_rec_gyro = False

        self._lock = threading.Lock()

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
        self.stop_sig = True
        rospy.signal_shutdown("CtrlC")

        #killing
        os.kill(os.getpgrp(), signal.SIGKILL)


    def quaternion_from_euler(self, roll, pitch, yaw):
        # Abbreviations for the various angular functions
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

        try:
            self.log_data = LogConfig(name="Data", period_in_ms=10)
            self.log_data.add_variable('acc.x', 'float')
            self.log_data.add_variable('acc.y', 'float')
            self.log_data.add_variable('acc.z', 'float')
            self.log_data.add_variable('pm.vbat', 'float')


            self.log_gyro_data = LogConfig(name="GyroData", period_in_ms=10)
            self.log_gyro_data.add_variable('gyro.x', 'float')
            self.log_gyro_data.add_variable('gyro.y', 'float')
            self.log_gyro_data.add_variable('gyro.z', 'float')

            self.log_data_stab = LogConfig(name="StabilizerData", period_in_ms=10)
            self.log_data_stab.add_variable('stabilizer.yaw', 'float')
            self.log_data_stab.add_variable('stabilizer.pitch', 'float')
            self.log_data_stab.add_variable('stabilizer.roll', 'float')
            # self.log_data.add_variable('kalman_states.vx', 'float')
            # self.log_data.add_variable('kalman_states.vy', 'float')

            self.log_data_kalman = LogConfig(name="KalmanData", period_in_ms=10)
            self.log_data_kalman.add_variable('kalman_states.vx', 'float')
            self.log_data_kalman.add_variable('kalman_states.vy', 'float')
            self.log_data_kalman.add_variable('stateEstimate.x', 'float')
            self.log_data_kalman.add_variable('stateEstimate.y', 'float')
            self.log_data_kalman.add_variable('stateEstimate.z', 'float')

            # self.log_data_pos = LogConfig(name="PosData", period_in_ms=10)
            # self.log_data_pos.add_variable('stateEstimate.x', 'float')
            # self.log_data_pos.add_variable('stateEstimate.y', 'float')
            # self.log_data_pos.add_variable('stateEstimate.z', 'float')

            # self.log_data_mag = LogConfig(name="MagData", period_in_ms=10)
            # self.log_data_mag.add_variable('mag.x', 'float')
            # self.log_data_mag.add_variable('mag.y', 'float')
            # self.log_data_mag.add_variable('mag.z', 'float')
            # self.log_data.add_variable('kalman_states.ox', 'float')
            # self.log_data.add_variable('kalman_states.oy', 'float')
            # self.log_data.add_variable('motion.deltaX', 'int16_t')
            # self.log_data.add_variable('motion.deltaY', 'int16_t')
            self.cf.log.add_config(self.log_data)
            self.cf.log.add_config(self.log_gyro_data)
            self.cf.log.add_config(self.log_data_stab)
            self.cf.log.add_config(self.log_data_kalman)
            # self.cf.log.add_config(self.log_data_pos)
            # self.cf.log.add_config(self.log_data_mag)

            self.log_data.data_received_cb.add_callback(self.received_data)
            self.log_gyro_data.data_received_cb.add_callback(self.received_data)
            self.log_data_stab.data_received_cb.add_callback(self.received_data)
            self.log_data_kalman.data_received_cb.add_callback(self.received_data)
            # self.log_data_pos.data_received_cb.add_callback(self.received_data)
            # self.log_data_mag.data_received_cb.add_callback(self.received_data)


            #begins logging and publishing
            self.log_data.start()
            self.log_gyro_data.start()
            self.log_data_stab.start()
            self.log_data_kalman.start()
            # self.log_data_pos.start()
            # self.log_data_mag.start()

            print("Logging Setup Complete. Starting...")
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add log config, bad configuration.')


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
            print("Not Accepting Commands -- but one was sent!")

    def motion_cb(self, msg):
        print("ALT: %.3f" % self.alt)
        # print(msg)
        if self.accept_commands:
            self.update_alt(msg)
            # switching between optical flow and roll pitch motion
            if msg.is_flow_motion:
                self.set_flow_motion(msg.x, msg.y, msg.yaw, self.alt)
            else:
                self.set_rp_motion(msg.x, msg.y, msg.yaw, self.alt)

        else:
            print("Not Accepting Motion Commands -- but one was sent!")

    def update_alt(self, msg):
        
        #what exactly does this do?
        #motion.alt = self.data.alt * 100 if self.data.alt > ALT_TOLERANCE else 0
        self.alt += msg.dz
        if self.alt < 0:
            self.alt = 0
        elif self.alt > MAX_ALT:
            self.alt = MAX_ALT

    def received_data(self, timestamp, data, logconf):
        # print("DATA RECEIVED")
        if self.to_publish == None:
            self.to_publish = CFData()
            self.to_publish.ID = self._id


        self._lock.acquire()

        try:
            if logconf.name == 'Data':
                self.data = data
                self.to_publish.accel_x = float(data['acc.x'])
                self.to_publish.accel_y = float(data['acc.y'])
                self.to_publish.accel_z = float(data['acc.z'])
                self.to_publish.v_batt = float(data['pm.vbat'])

                # imu msg
                self._imu_rec_acc = True

                self._rec_data = True
            elif logconf.name == 'GyroData':
                self.to_publish.gyro_x = float(data['gyro.x'])
                self.to_publish.gyro_y = float(data['gyro.y'])
                self.to_publish.gyro_z = float(data['gyro.z'])

                # imu msg
                self._imu_rec_gyro = True

                self._rec_data_gyro = True

            elif logconf.name == 'StabilizerData':
                self.to_publish.yaw = float(data['stabilizer.yaw'])
                self.to_publish.pitch = float(data['stabilizer.pitch'])
                self.to_publish.roll = float(data['stabilizer.roll'])
                self._rec_data_stab = True
            elif logconf.name == 'KalmanData':
                self.to_publish.kalman_vx = float(data['kalman_states.vx'])
                self.to_publish.kalman_vy = float(data['kalman_states.vy'])
                self.to_publish.pos_x = float(data['stateEstimate.x'])
                self.to_publish.pos_y = float(data['stateEstimate.y'])
                self.to_publish.alt = float(data['stateEstimate.z'])
                self._rec_data_kalman = True

            elif logconf.name == 'MagData':
                self.to_publish.magx = float(data['mag.x'])
                self.to_publish.magy = float(data['mag.y'])
                self.to_publish.magz = float(data['mag.z'])
                self._rec_data_mag = True
                   
            if self._rec_data and self._rec_data_gyro and self._rec_data_stab and self._rec_data_kalman: # and self._rec_data_mag: 
                self.data_pub.publish(self.to_publish)
                # self.to_publish = None

                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.pose.position = Vector3(self.to_publish.pos_x, self.to_publish.pos_y, self.to_publish.alt)
                pose_msg.pose.orientation = self.quaternion_from_euler(self.to_publish.roll, self.to_publish.pitch, self.to_publish.yaw)
                
                twist_msg = TwistStamped()
                twist_msg.header.stamp = rospy.Time.now()
                twist_msg.twist.linear = Vector3(self.to_publish.kalman_vx, self.to_publish.kalman_vy, 0)
                twist_msg.twist.angular = Vector3(self.to_publish.gyro_x, self.to_publish.gyro_y, self.to_publish.gyro_z)
                # print(pose_msg)
                # print(twist_msg)
                self.pose_pub.publish(pose_msg)
                
                self.twist_pub.publish(twist_msg)
                

                self._rec_data = False
                self._rec_data_gyro = False
                self._rec_data_stab = False
                self._rec_data_kalman = False
                # self._rec_data_mag = False

            if self._imu_rec_gyro and self._imu_rec_acc:

                imu_msg = Imu();
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.angular_velocity = self.quaternion_from_euler(self.to_publish.gyro_x, self.to_publish.gyro_y, self.to_publish.gyro_z)
                imu_msg.linear_acceleration = self.quaternion_from_euler(self.to_publish.accel_x, self.to_publish.accel_y, self.to_publish.accel_z)
                imu_msg.orientation_covariance[0] = -1 # orientation estimates not present

                self.imu_pub.publish(imu_msg)


                self._imu_rec_acc = False
                self._imu_rec_gyro = False
                # print("MAG:", data)

        except Exception as e:
            print(str(e))

        self._lock.release()
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
            print("cannot land right now")
        else:
            for y in range(10):
                self.cf.commander.send_hover_setpoint(0, 0, 0, alt - (y / 10 * alt))
                time.sleep(0.1)
            self.cmd_estop()

    def run(self):
        print("WAITING FOR ACTIVE CONNECTION")
        while not self.cf_active:
            pass
        print("FOUND ACTIVE CONNECTION")

        #handles image reads
        # threading.Thread(target=self.image_thread).start()

        rate = rospy.Rate(25)

        rospy.spin()

        self.log_data.stop()
