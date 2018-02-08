#!/usr/bin/python2

import sys

sys.path.append( "/home/katiekang/catkin_ws/src/crazyflie/src")

from Camera import Camera

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import time
import signal

DEFAULT_URI = 'radio://0/80/250K'

if __name__ == '__main__':

    if len(sys.argv) == 2: 
        ID = sys.argv[1]

        cam = Camera(ID)
        cam.run()

    else:
        print "More/Less than 2 arguments passed in"