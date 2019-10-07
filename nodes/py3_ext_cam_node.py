#!/usr/bin/env python

import sys
# import os
import rospy

from crazyflie import ExternalCamera

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import time
import signal


if __name__ == '__main__':

    rospy.init_node('ExternalCamera', anonymous=True)

    cam_id_param = rospy.search_param('cam_id')
    is_raw_param = rospy.search_param('is_raw')

    if not cam_id_param: 
        print("No Camera ID Specified! Abort.")
        sys.exit(0)

    if not is_raw_param:
        print("No is_raw specified. Default is False.")
        is_raw = 'False'
    else:
        is_raw = rospy.get_param(is_raw_param, 'False')

    cam_id = int(rospy.get_param(cam_id_param, '0'))

    cam = ExternalCamera(int(cam_id), is_raw=='True')
    cam.run()