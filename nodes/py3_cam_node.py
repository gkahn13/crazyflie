#!/usr/bin/env python

import sys
import rospy

from crazyflie import Camera

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import time
import signal

DEFAULT_URI = 'radio://0/80/250K'

if __name__ == '__main__':

    rospy.init_node('Camera', anonymous=True)

    if len(sys.argv) == 4: 
        ID = sys.argv[1]
        cam_id = sys.argv[2]
        raw = sys.argv[3]
        
        cam = Camera(int(ID), int(cam_id), raw == 'True')
        cam.run()

    else:
        print("More/Less than 2 arguments passed in, needs: [ID] [cam_ID] [is_raw]")
