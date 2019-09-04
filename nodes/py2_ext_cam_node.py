#!/usr/bin/python2

import sys
import rospy

# sys.path.append( "$HOME/catkin_ws/src/crazyflie/src")
import crazyflie
from ExternalCamera import ExternalCamera

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import time
import signal

if __name__ == '__main__':

    rospy.init_node('Py2ExternalCamera', anonymous=True)

    if len(sys.argv) == 3:
        cam_id = sys.argv[1]
        raw = sys.argv[2]
        
        cam = ExternalCamera(int(cam_id), raw == 'True')
        cam.run()

    else:
        print "More/Less than 2 arguments passed in, needs: [cam_ID] [is_raw]"
