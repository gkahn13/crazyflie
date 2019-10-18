import rospy
import rosbag
import os
import os.path

import random
import argparse
import time
import math

import threading
import subprocess, shlex
import signal

import matplotlib.pyplot as plt

from crazyflie.msg import CFData, CFMotion, CFCommand
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from sensor_msgs.msg import CompressedImage, Imu, Joy
from std_msgs.msg import Bool
from crazyflie.sim import PointMassCrazyflieSimulator

plt.ion()

class RolloutRosbag:
    def __init__(self, rosbag_dir, realtime):
        self._rosbag_proc = None
        self._rosbag = None

        self._dir = rosbag_dir
        if not os.path.exists(self._dir):
            os.mkdir(self._dir)

        self._realtime = realtime
        self._last_write = None

    def _rosbag_name(self, num):
        return os.path.join(self._dir, 'rosbag{0:04d}.bag'.format(num))

    @property
    def is_open(self):
        if self._realtime:
            return (self._rosbag_proc is not None)
        else:
            return (self._rosbag is not None)

    # topics is a list of strings to record, or empty if all
    def open(self, topics):
        assert (not self.is_open)

        bag_num = 0
        while os.path.exists(self._rosbag_name(bag_num)):
            bag_num += 1

        self._output_bag_name = self._rosbag_name(bag_num)

        if self._realtime:
            self._command = "rosbag record -O %s" % self._output_bag_name

            if topics:
                for topic in topics:
                    self._command += " %s" % topic
            else:
                self._command += " -a"

            args = shlex.split(self._command)
            self._rosbag_proc = subprocess.Popen(args)

            # wait for process to open file
            while not os.path.exists(self._output_bag_name + ".active"):
                pass
        else:
            print("Recording to: %s" % self._output_bag_name)
            self._rosbag = rosbag.Bag(self._rosbag_name(bag_num), 'w', compression='bz2')
            
        self._last_write = rospy.Time.now()

    def write(self, topic, msg, stamp):
        assert (self._rosbag is not None)
        self._rosbag.write(topic, msg, stamp)

    def close(self):
        assert (self.is_open)

        # self._rosbag_proc.send_signal(subprocess.signal.SIGINT)
        # os.killpg(os.getpgid(self._rosbag_proc.pid), signal.SIGINT)
        if self._realtime:
            terminate_process_and_children(self._rosbag_proc)
            # # wait for process to save file
            while not os.path.exists(self._output_bag_name):
                pass
        else:
            print("  Bag Saved: %s" % self._output_bag_name)
            self._rosbag.close()

        self._rosbag_proc = None
        self._rosbag = None
        self._last_write = None

    def trash(self):
        assert (self.is_open)

        try:
            self.close()
            time.sleep(0.2)
        except:
            pass

        os.remove(self._output_bag_name)

# # TODO max_steps
# def main(rosbag_dir, ros_prefix, is_flow_motion, max_steps):


def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.decode('ascii').split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
    # p.terminate()
    os.kill(p.pid, signal.SIGINT)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # parser.add_argument('-env', type=str, required=True)
    parser.add_argument('--rosbag-dir', type=str, required=True)
    parser.add_argument('--niters', type=int, default=100, help="Number of steps per rollout.")
    parser.add_argument('--nrollouts', type=int, default=100, help="Number of rollouts.")
    parser.add_argument('--ros-prefix', type=str, default="/cf/0/")
    parser.add_argument('--is-flow', type=bool, default=True)
    parser.add_argument('--no-action-noise', action='store_true')
    parser.add_argument('--render', action='store_true')
    parser.add_argument('--uncorrelated', action='store_true', help="actions are uncorrelated")
    parser.add_argument('--action-bounding', action='store_true')
    parser.add_argument('--enable-yaw', action='store_true')
    parser.add_argument('--lag', type=int, default=0, help="how much artificial lag to add to system")
    # parser.add_argument('-ca', '--ctrl_arg', action='append', nargs=2, default=[])
    # parser.add_argument('-o', '--override', action='append', nargs=2, default=[])
    # parser.add_argument('-model-dir', type=str, required=True)
    # parser.add_argument('-logdir', type=str, required=True)
    args = parser.parse_args()

    # main(args.rosbag_dir, args.ros_prefix, args.is_flow, args.max_steps)

    rospy.init_node("DataCapture", anonymous=True)

    _ros_prefix =  args.ros_prefix
    # _is_flow_motion = args.is_flow
    _rosbag = RolloutRosbag(args.rosbag_dir, realtime=False)
    # ensuring no dropped msgs
    _env = PointMassCrazyflieSimulator(_ros_prefix, 0.1, use_ros=False, lag=args.lag)

    # _ros_msg_queue_buffer = dict()

    toplist = [ 
        # (_ros_prefix + 'image', CompressedImage),
        (_ros_prefix + 'motion', CFMotion),
        ('extcam/target_vector', Vector3Stamped),
        # ('/joy', Joy),
    ]

    _ros_topics_and_types = dict(toplist)

    _ros_global_topics = []
    
    ## START OF CB
    
    def bound_action_by_target_loc(motion, vector):
        nx,ny,ndz = motion.x, motion.y, motion.dz
        if motion.y > 0 and vector.x < 0.05: # left moving and we are close to left edge
            ny = 0
        if motion.y < 0 and vector.x > 0.95: # right
            ny = 0
        if motion.dz > 0 and vector.y < 0.05: # up
            ndz = 0
        if motion.dz < 0 and vector.y > 0.95: # down
            ndz = 0

        return nx, ny, ndz

    for r in range(args.nrollouts):
        print("## ROLLOUT %d" % r)

        _rosbag.open(list(_ros_topics_and_types.keys()) + list(_ros_global_topics))

        _env.reset(target=[random.uniform(0,1), random.uniform(0,1), random.uniform(0,0.5)])
        _env.run_random_motion(args.niters, render=args.render, realtime=args.render, rosbag=_rosbag, correlated=not args.uncorrelated)

        if args.render:
            input("Press enter to continue: ")

        _rosbag.close()

    ## ISSUES LANDING (never happens)