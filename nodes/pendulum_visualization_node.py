#!/usr/bin/env python

import rospy
import os
import sys

from crazyflie.pendulum_visualization import PendulumVisualization

import logging
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import time
import signal

# DEFAULT_URI = 'radio://0/80/250K'

if __name__ == '__main__':

    rospy.init_node('PendulumVisualization', anonymous=True)

    id_param = rospy.search_param('id')
    traj_param = rospy.search_param('num_trajectories')
    is_raw_param = rospy.search_param('is_raw')
    # uri_param = rospy.search_param('uri')
    # data_only_param = rospy.search_param('data_only')
    # motion_param = rospy.search_param('motion')
    # config_param = rospy.search_param('config')

    if not id_param: 
        print("No ID Specified! Abort.")
        sys.exit(0)

    ID = int(rospy.get_param(id_param, '0'))
    
    if not is_raw_param:
        print("No is_raw specified. Default is False.")
        is_raw = False
    else:
        is_raw = bool(rospy.get_param(is_raw_param, 'False'))

    if not traj_param:
        print("No trajectory specified. Default is 1.")
        num_trajectories = 1
    else:
        num_trajectories = int(rospy.get_param(traj_param, '1'))


    # URI = rospy.get_param(uri_param, DEFAULT_URI)
    # data_only = bool(rospy.get_param(data_only_param, DEFAULT_URI))
    # motion = rospy.get_param(motion_param, 'None')
    # config = rospy.get_param(config_param, 'None')

 #    if not rospy.has_param('id') or not rospy.has_param('uri'):
 #        print("No ID or URI Specified! Abort.")
	# sys.exit(0)

 #    ID = int(rospy.get_param('id', '0'))
 #    URI = rospy.get_param('uri', DEFAULT_URI)

    pv = PendulumVisualization(ID, is_raw, num_trajectories)
    pv.run()
