import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie as CF
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie as SyncCF
from cflib.crazyflie.log import LogConfig

import rospy
import std_msgs.msg

from crazyflie.msg import CFCommand, CFMotion


class Crazyflie:

    DEFAULT_HEIGHT = 0.6

    def __init__(self, id, radio_uri):
        self._is_airborne = False
        self._scf = self._init_cf(radio_uri)

        self._command_msg = None
        self._motion_msg = None
        self._ros_publishers = self._init_ros(id, self._scf.cf)

    ############
    ### Init ###
    ############

    def _init_cf(self, radio_uri):
        print('Initializing Crazyflie...')

        cflib.crtp.init_drivers(enable_debug_driver=False)

        scf = SyncCF(radio_uri, cf=CF(rw_cache='./cache'))
        scf.open_link()

        log_data = LogConfig(name="Data", period_in_ms=10)
        log_data.add_variable('acc.x', 'float')
        log_data.add_variable('acc.y', 'float')
        log_data.add_variable('acc.z', 'float')
        log_data.add_variable('pm.vbat', 'float')
        log_data.add_variable('stateEstimate.z', 'float')

        log_data_stab = LogConfig(name="StabilizerData", period_in_ms=10)
        log_data_stab.add_variable('stabilizer.yaw', 'float')
        log_data_stab.add_variable('stabilizer.pitch', 'float')
        log_data_stab.add_variable('stabilizer.roll', 'float')

        log_data_kalman = LogConfig(name="KalmanData", period_in_ms=10)
        log_data_kalman.add_variable('kalman_states.vx', 'float')
        log_data_kalman.add_variable('kalman_states.vy', 'float')

        log_data_mag = LogConfig(name="MagData", period_in_ms=10)
        log_data_mag.add_variable('mag.x', 'float')
        log_data_mag.add_variable('mag.y', 'float')
        log_data_mag.add_variable('mag.z', 'float')

        for l in (log_data, log_data_stab, log_data_kalman, log_data_mag):
            scf.cf.log.add_config(l)
            l.data_received_cb.add_callback(self._cf_received_data_callback)
            l.start()

        print('Initialized Crazyflie.')

        return scf

    def _init_ros(self, id, cf):
        print('Initializing ROS...')

        ### publishers
        ros_publishers = dict()
        for log_block in cf.log.log_blocks:
            for log_var in log_block.variables:
                log_name = log_var.name
                ros_topic = log_name.replace('.', '/')
                assert log_name not in ros_publishers
                ros_publishers[log_name] = rospy.Publisher('cf/{0:0d}/{1}'.format(id, ros_topic),
                                                           std_msgs.msg.Float32,
                                                           queue_size=10)

        ### subscribers
        rospy.Subscriber('cf/{0:d}/command'.format(id), CFCommand, self._ros_command_callback)
        rospy.Subscriber('cf/{0:d}/motion'.format(id), CFMotion, self._ros_motion_callback)

        print('Initialized ROS.')

        return ros_publishers

    ####################
    ### CF callbacks ###
    ####################

    def _cf_received_data_callback(self, timestamp, data, logconf):
        self._data = data
        for log_name, log_value in data.items():
            self._ros_publishers[log_name].publish(std_msgs.msg.Float32(log_value))

    ###################
    ### CF commands ###
    ###################

    def _cf_command_takeoff(self):
        print("---- Crazyflie Taking Off ----")

        self._scf.cf.param.set_value('kalman.resetEstimation', '1')
        rospy.sleep(0.1)
        self._scf.cf.param.set_value('kalman.resetEstimation', '0')
        rospy.sleep(1)

        for height in np.linspace(0, Crazyflie.DEFAULT_HEIGHT, 10):
            self._scf.cf.commander.send_hover_setpoint(0, 0, 0, height)
            rospy.sleep(0.1)
        self._is_airborne = True

    def _cf_command_land(self):
        print("---- Crazyflie Landing ----")
        for height in np.linspace(0, Crazyflie.DEFAULT_HEIGHT, 10)[::-1]:
            self._scf.cf.commander.send_hover_setpoint(0, 0, 0, height)
            rospy.sleep(0.1)
        self._cf_command_estop()
        self._is_airborne = False

    def _cf_command_estop(self):
        print("---- Crazyflie Emergency Stopping ----")
        self._scf.cf.commander.send_stop_setpoint()
        self._is_airborne = False

    def _cf_command_flow_motion(self, vx, vy, yaw, alt):
        self._scf.cf.commander.send_hover_setpoint(vx, vy, yaw, alt)

    #####################
    ### ROS callbacks ###
    #####################

    def _ros_command_callback(self, msg):
        self._command_msg = msg

    def _ros_motion_callback(self, msg):
        self._motion_msg = msg

    ####################
    ### Enter / Exit ###
    ####################

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._scf.close_link()

    ###########
    ### Run ###
    ###########

    def run(self):
        print('\nIN RUNNNNN')

        rate = rospy.Rate(10.)

        while not rospy.is_shutdown():
            if self._command_msg:
                command_msg = self._command_msg
                if command_msg.cmd == CFCommand.TAKEOFF and not self._is_airborne:
                    self._cf_command_takeoff()
                elif command_msg.cmd == CFCommand.LAND and self._is_airborne:
                    self._cf_command_land()
                elif command_msg.cmd == CFCommand.ESTOP and self._is_airborne:
                    self._cf_command_estop()

                self._command_msg = None

            if self._motion_msg and self._is_airborne:
                motion_msg = self._motion_msg
                self._cf_command_flow_motion(motion_msg.x, motion_msg.y, motion_msg.yaw, motion_msg.height)

            rate.sleep()
