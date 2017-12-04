#!/usr/bin/env python

import rospy

import Crazyflie

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import time
import signal
import sys


DEFAULT_URI = 'radio://0/80/250K'

if __name__ == '__main__':

    if not rospy.has_param('id') or not rospy.has_param('uri'):
        print("No ID or URI Specified! Abort.")
	sys.exit(0)

    ID = int(rospy.get_param('id', '0'))
    URI = rospy.get_param('uri', DEFAULT_URI)

    rospy.init_node('Crazyflie %d' % ID, anonymous=True)

    cf = Crazyflie(ID, URI)
    cf.run()