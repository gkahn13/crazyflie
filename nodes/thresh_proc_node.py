#!/usr/bin/env python

import sys
# import os
import rospy

from crazyflie.thresh_proc import ThreshProc

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import time
import signal


if __name__ == '__main__':

    rospy.init_node('ThreshProc', anonymous=True)

    in_topic_param = rospy.search_param('in_topic')
    is_raw_param = rospy.search_param('is_raw')

    if not in_topic_param: 
        print("No in_topic specified. Abort!.")
        sys.exit(0)

    if not is_raw_param:
        print("No is_raw specified. Default is False.")
        is_raw = 'False'
    else:
        is_raw = rospy.get_param(is_raw_param, 'False')

    in_topic = rospy.get_param(in_topic_param, 'extcam/image')

    cam = ThreshProc(in_topic, is_raw=='True')
    cam.run()