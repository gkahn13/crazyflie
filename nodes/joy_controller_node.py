#!/usr/bin/env python

import rospy
import sys

sys.path.append("/home/katiekang/catkin_ws/src/crazyflie/src")

from Controller import Controller


if __name__ == '__main__':

    rospy.init_node('JoyController', anonymous=True)

    idParam = rospy.search_param('id')
    joyParam = rospy.search_param('joy_topic')
    if not idParam or not joyParam:
        print("No ID or JOY TOPIC Specified! Abort.")
        sys.exit(0)

    ID = int(rospy.get_param(idParam, '0'))
    joy_topic = int(rospy.get_param(joyParam, '/joy'))

    # if not rospy.has_param('id'):
    #     print("No ID or URI Specified! Abort.")
    # sys.exit(0)

    # ID = int(rospy.get_param('id', '0'))


    
    control = Controller(ID)
    control.run()
