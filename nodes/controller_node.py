#!/usr/bin/env python

import rospy
import sys

sys.path.append("/home/katiekang/catkin_ws/src/crazyflie/src")

import Controller


if __name__ == '__main__':

    if not rospy.has_param('~id'):
        print("No ID or URI Specified! Abort.")
    sys.exit(0)

    ID = int(rospy.get_param('~id', '0'))

    rospy.init_node('Controller %d' % ID, anonymous=True)
    
    cam = Controller(ID)
    cam.run()
