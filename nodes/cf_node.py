#!/usr/bin/env python

import rospy

import sys

sys.path.append( "/home/katiekang/catkin_ws/src/crazyflie/src")

import Crazyflie

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import time
import signal

DEFAULT_URI = 'radio://0/80/250K'

if __name__ == '__main__':

    id_param = rospy.search_param('id')
    uri_param = rospy.search_param('uri')
    if not id_param or not uri_param: 
        print("No ID or URI Specified! Abort.")
        sys.exit(0)

    ID = int(rospy.get_param(id_param, '0'))
    URI = rospy.get_param(uri_param, DEFAULT_URI)

 #    if not rospy.has_param('id') or not rospy.has_param('uri'):
 #        print("No ID or URI Specified! Abort.")
	# sys.exit(0)

 #    ID = int(rospy.get_param('id', '0'))
 #    URI = rospy.get_param('uri', DEFAULT_URI)

    rospy.init_node('Crazyflie %d' % ID, anonymous=True)

    cf = Crazyflie(ID, URI)
    cf.run()
