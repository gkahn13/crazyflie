import rospy

from crazyflie.msg import CFData
from crazyflie.msg import CFImage
from crazyflie.msg import CFCommand
from crazyflie.msg import CFMotion

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import time
import signal
import sys

import threading

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Handles all interaction with CF through its radio.
class CFNode:

    # ID is for human readability
    def __init__(self, cf_id, radio_uri):
        self._id = cf_id
        self._uri = radio_uri

        cflib.crtp.init_drivers(enable_debug_driver=False)
        try:
            self.scf = SyncCrazyflie(URI)
            self.cf = self.scf.cf
        except e:
            print("Unable to connect to CF %d at URI %s" % (self._id, self._uri))
            self.scf = None
            self.cf = None

        self.data_pub = rospy.Publisher('cf/data', CFData, queue_size=10)
        self.image_pub = rospy.Publisher('cf/image', CFImage, queue_size=10)

        self.cmd_sub = rospy.Subscriber('cf/%d/command'%self._id, CFCommand, command_cb)
        self.motion_sub = rospy.Subscriber('cf/%d/motion'%self._id, CFMotion, motion_cb)

        threading.Thread(target=self.image_thread).start()

    ## CALLBACKS ##

    def command_cb(self, msg):
        pass

    def motion_cb(self, msg):
        pass

    ## COMMANDS ##

    def set_motion(self, vx, vy, yaw, alt):
        self.cf.commander.send_hover_setpoint(vx, vy, yaw, alt)

    def cmd_estop(self, frame):
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

    # TODO: Katie figure this out -- gets images and sends it across network
    # runs in parallel to main thread
    def image_thread(self):
        pass


DEFAULT_URI = 'radio://0/80/250K'

if __name__ == '__main__':
    rospy.init_node('CFNode', anonymous=True)

    if not rospy.has_param('id') or not rospy.has_param('uri'):
        print("No ID or URI Specified! Abort.")
        return

    cf = CFNode(int(rospy.get_param('id', '0')), rospy.get_param('uri', DEFAULT_URI))

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.spinOnce()
        rate.sleep()
