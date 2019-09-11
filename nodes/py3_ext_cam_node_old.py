#!/usr/bin/env python

import subprocess
import os
import sys

import rospy

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

    path = os.path.join(os.getenv('HOME'), "catkin_ws", "src", "crazyflie", "nodes", "py2_ext_cam_node.py")
    py2_cmd = "python2 %s %d %s" % (path, cam_id, is_raw)

    p = subprocess.Popen(py2_cmd.split())

    while not rospy.is_shutdown():
    	pass

    print(" -- Remotely Terminating Camera Node -- ")
    p.kill()
