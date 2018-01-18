import rospy
import numpy as np
import cv2

from crazyflie.msg import CFData
# from crazyflie.msg import CFImage
from sensor_msgs.msg import Image
from crazyflie.msg import CFCommand
from crazyflie.msg import CFMotion

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import time
import signal
import sys

import threading

import cv_bridge
from cv_bridge import  CvBridge

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

## for convenience
cmd_type = ['']*3
cmd_type[CFCommand.ESTOP] = 'ESTOP'
cmd_type[CFCommand.LAND] = 'LAND'
cmd_type[CFCommand.TAKEOFF] = 'TAKEOFF'

# Handles all interaction with CF through its radio.
class Crazyflie:

    # ID is for human readability
    def __init__(self, cf_id, radio_uri):
        self._id = cf_id
        self._uri = radio_uri


        self.data = None

        self.bridge = CvBridge()

        cflib.crtp.init_drivers(enable_debug_driver=False)
        # try:
        with SyncCrazyflie(self._uri) as scf:

            self.cf = scf.cf

            self.cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            self.cf.param.set_value('kalman.resetEstimation', '0')
            time.sleep(1.5)


            # except Exception as e:
            #     print(type(e))
            #     print("Unable to connect to CF %d at URI %s" % (self._id, self._uri))
            #     self.scf = None
            #     self.cf = None

            try:
                self.log_data = LogConfig(name="Data", period_in_ms=100)
                self.log_data.add_variable('imu.acc_x', 'float')
                self.log_data.add_variable('imu.acc_y', 'float')
                self.log_data.add_variable('imu.acc_z', 'float')
                self.log_data.add_variable('pm.vbat', 'float')
                self.log_data.add_variable('posEstimatorAlt.estimatedZ', 'float')
                self.cf.log.add_config(self.log_data)
                self.log_data.data_received_cb.add_callback(self.received_data)
            except KeyError as e:
                print('Could not start log configuration,'
                      '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add log config, bad configuration.')

        self.data_pub = rospy.Publisher('cf/data', CFData, queue_size=10)
        self.image_pub = rospy.Publisher('cf/image', Image, queue_size=10)

        self.cmd_sub = rospy.Subscriber('cf/%d/command'%self._id, CFCommand, self.command_cb)
        self.motion_sub = rospy.Subscriber('cf/%d/motion'%self._id, CFMotion, self.motion_cb)

    ## CALLBACKS ##

    def command_cb(self, msg):
        if cmd_type[msg.cmd] == 'ESTOP':
            self.cmd_estop()
        elif cmd_type[msg.cmd] == 'LAND':
            self.cmd_land()
        elif cmd_type[msg.cmd] == 'TAKEOFF':
            self.cmd_takeoff()
        else:
            print('Invalid Command! %d' % msg.cmd)

    def motion_cb(self, msg):
        self.set_motion(msg.vx, msg.vy, msg.yaw, msg.alt)

    def received_data(self, timestamp, data, logconf):
        self.data = data
        d = CFData()
        d.ID = self._id
        d.accel_x = float(data['imu.acc_x'])
        d.accel_y = float(data['imu.acc_y'])
        d.accel_z = float(data['imu.acc_z'])
        d.v_batt = float(data['pm.vbat'])
        d.alt = float(data['posEstimatorAlt.estimatedZ'])

        self.data_pub.publish(d)

    ## COMMANDS ##

    def set_motion(self, vx, vy, yaw, alt):
        self.cf.commander.send_hover_setpoint(vx, vy, yaw, alt)

    def cmd_estop(self):
        print("---- Crazyflie %d Stopping ----" % self._id)
        self.cf.commander.send_stop_setpoint()

    def cmd_takeoff(self, alt=0.4):
        for y in range(10):
            self.cf.commander.send_hover_setpoint(0, 0, 0, y / 10 * alt)
            time.sleep(0.1)

    def cmd_land(self, alt=0.4):
        for y in range(10):
            self.cf.commander.send_hover_setpoint(0, 0, 0, alt - (y / 10 * alt))
            time.sleep(0.1)


    ## IMAGE HANDLING / THREADS ##

    # runs in parallel to main thread
    def image_thread(self):
        image_rate = rospy.Rate(20)
        cap = cv2.VideoCapture(1) # TODO: multiple vid captures in parallel
        cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(gray, gray.type()))
            rate.sleep()
        cap.release()
        cv2.destroyAllWindows()


    def run(self):
        #begins logging and publishing
        self.log_data.start()

        #handles image reads
        threading.Thread(target=self.image_thread).start()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rospy.spinOnce()
            rate.sleep()

        self.cmd_estop()
