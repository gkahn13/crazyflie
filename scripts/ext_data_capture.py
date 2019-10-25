import rospy
import rosbag
import os
import os.path

import random
import argparse
import time

import threading
import subprocess, shlex
import signal

from crazyflie.msg import CFData, CFMotion, CFCommand
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from sensor_msgs.msg import CompressedImage, Imu, Joy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from crazyflie.utils import MasterSigController, STOP, START

THROTTLE_AXIS = 1 # up 1
ROLL_AXIS = 2 #left 1
PITCH_AXIS = 3 #up 1
YAW_AXIS = 0 #left 1

ALT_AXIS = 5 # D-pad up

DZ_INCREMENT = 0.02
DT = 0.1

#RP motion
THROTTLE_SCALE = 0.05
ROLL_SCALE = -25
PITCH_SCALE = -25
YAW_SCALE = -120

ROLL_CLIP = abs(ROLL_SCALE) * 0.4
PITCH_CLIP = abs(PITCH_SCALE) * 0.1
THROTTLE_CLIP = abs(THROTTLE_SCALE) * 0.2

#standard motion
VX_SCALE = 0.5
VY_SCALE = 0.5

# arbitrary 8% of max is the std dev of noise 
VXY_NOISE_STD = abs(VX_SCALE) * 0.30 # variance 30% of max speed
VZ_NOISE_STD = abs(THROTTLE_SCALE) * 0.2 * DT # variance 20% of max throttle speed

class RolloutRosbag:
    def __init__(self, rosbag_dir):
        self._rosbag_proc = None

        self._dir = rosbag_dir
        if not os.path.exists(self._dir):
            os.mkdir(self._dir)

        self._last_write = None

    def _rosbag_name(self, num):
        return os.path.join(self._dir, 'rosbag{0:04d}.bag'.format(num))

    @property
    def is_open(self):
        return (self._rosbag_proc is not None)

    # topics is a list of strings to record, or empty if all
    def open(self, topics):
        assert (not self.is_open)

        bag_num = 0
        while os.path.exists(self._rosbag_name(bag_num)):
            bag_num += 1

        self._output_bag_name = self._rosbag_name(bag_num)

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

        # self._rosbag = rosbag.Bag(self._rosbag_name(bag_num), 'w', compression='bz2')
        self._last_write = rospy.Time.now()

    # def write(self, topic, msg, stamp):
    #     assert (self._rosbag is not None)

    #     if msg is not None and stamp is not None:
    #         if stamp > self._last_write:
    #             self._rosbag.write(topic, msg)

    # def write_all(self, topics, msg_dict, stamp_dict):

    #     for topic in topics:
    #         self.write(topic, msg_dict.get(topic), stamp_dict.get(topic))

        # print("Starting write all")
        # for topic, ls in queue_list.items():
        #     while len(ls) > 0:
        #         msg, st = ls.pop()
        #         self.write(topic, msg, st) 
        # print("Ending write all")


    def close(self):
        assert (self.is_open)

        # self._rosbag_proc.send_signal(subprocess.signal.SIGINT)
        # os.killpg(os.getpgid(self._rosbag_proc.pid), signal.SIGINT)
        terminate_process_and_children(self._rosbag_proc)
        # # wait for process to save file
        while not os.path.exists(self._output_bag_name):
            pass

        self._rosbag_proc = None
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
    parser.add_argument('--max-steps', type=int, default=-1, help="Max Number of steps per episode.")
    parser.add_argument('--ros-prefix', type=str, default="/cf/0/")
    parser.add_argument('--is-flow', type=bool, default=True)
    parser.add_argument('--no-action-noise', action='store_true')
    parser.add_argument('--action-bounding', action='store_true')
    parser.add_argument('--enable-yaw', action='store_true')
    parser.add_argument('--policy', type=str, help='policy to use for data collection (teleop or mpc) [teleop]', default="teleop")
    # parser.add_argument('-ca', '--ctrl_arg', action='append', nargs=2, default=[])
    # parser.add_argument('-o', '--override', action='append', nargs=2, default=[])
    # parser.add_argument('-model-dir', type=str, required=True)
    # parser.add_argument('-logdir', type=str, required=True)
    args = parser.parse_args()

    # main(args.rosbag_dir, args.ros_prefix, args.is_flow, args.max_steps)

    rospy.init_node("DataCapture", anonymous=True)

    _rosbag_lock = threading.Lock()
    _queue_lock = threading.Lock()
    # _buffer_lock = threading.Lock()

    _ros_prefix =  args.ros_prefix
    _is_flow_motion = args.is_flow
    _rosbag = RolloutRosbag(args.rosbag_dir)
    _is_collision = False
    _adjustment_period = False
    _curr_motion = CFMotion()
    _curr_joy = Joy()
    _prev_joy = Joy()

    _joy_start_btn = 1 # A
    _joy_stop_trash_btn = 3 # Y
    _joy_stop_save_btn = 0 # X
    _joy_estop_trash_btn = 2 # B
    _joy_estop_pause_btn = 5 # RB

    _joy_stop_trash_btn_pressed = False
    _joy_stop_save_btn_pressed = False
    _joy_estop_trash_btn_pressed = False
    _joy_estop_pause_btn_pressed = False

    _master = MasterSigController(_ros_prefix)

    _ros_msg_times = dict()
    _ros_msg_queue = dict()
    # _ros_msg_queue_buffer = dict()

    toplist = [ 
        # (_ros_prefix + 'image', CompressedImage),
        (_ros_prefix + 'data', CFData),
        (_ros_prefix + 'motion', CFMotion),
        (_ros_prefix + 'command', CFCommand),
        (_ros_prefix + 'imu', Imu),
        (_ros_prefix + 'pose', PoseStamped),
        (_ros_prefix + 'twist', TwistStamped),
        (_ros_prefix + 'coll', Bool),
        ('extcam/image', CompressedImage),
        ('extcam/target_vector', Vector3Stamped),
        ('mpc/trajectory_marker', Marker),
        ('mpc/action_vector', Vector3Stamped),
        ('mpc/action_marker', Marker),
        ('mpc/goal_vector', Vector3Stamped),
        ('mpc/reward_vector', Vector3Stamped),
        # ('/joy', Joy),
    ]

    _ros_topics_and_types = dict(toplist)

    _ros_global_topics = ['/tf', '/tf_static']
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

    def msg_cb(msg, args):
        topic = args[0]
        global _is_collision

        msg_time = rospy.Time.now()

        _ros_msg_times[topic] = msg_time
        
        # if _rosbag.is_open:

        if 'coll' in topic:
            if not _is_collision and msg.data == 1:
                _is_collision = True

        # _queue_lock.acquire()
        _ros_msg_queue[topic] = msg
        # _queue_lock.release()
            # elif (_buffer_lock.acquire(False)):
            #     _ros_msg_queue_buffer[topic].insert(0, (msg, msg_time))
            #     _buffer_lock.release()
            # else:
            #     print("Dropping message!!")

            # _rosbag.write(topic, msg, msg_time)

    def joy_cb(msg):
        global _prev_joy
        global _curr_joy
        global _joy_stop_trash_btn_pressed
        global _joy_stop_save_btn_pressed
        global _joy_estop_trash_btn_pressed
        global _joy_estop_pause_btn_pressed
        global _curr_motion

        _prev_joy = _curr_joy
        _curr_joy = msg

        # print(_curr_joy.buttons)
        # print(_prev_joy.buttons)
        # print()
        
        # need to translate curr motion
        _curr_motion = CFMotion()

        if not _rosbag.is_open:
            _curr_motion.is_flow_motion = True
            _curr_motion.y = _curr_joy.axes[ROLL_AXIS] * VY_SCALE
            _curr_motion.x = _curr_joy.axes[PITCH_AXIS] * VX_SCALE
            _curr_motion.yaw = _curr_joy.axes[YAW_AXIS] * YAW_SCALE

            if _is_btn(ALT_AXIS, axes=True):
                _curr_motion.dz = _curr_joy.axes[ALT_AXIS] * DZ_INCREMENT
            else:
                _curr_motion.dz = 0
            # computing regular vx, vy, yaw, alt motion

        else:
            if _is_btn(_joy_stop_trash_btn):
                _joy_stop_trash_btn_pressed = True
            if _is_btn(_joy_stop_save_btn):
                _joy_stop_save_btn_pressed = True
            if _is_btn(_joy_estop_trash_btn):
                _joy_estop_trash_btn_pressed = True
            if _is_btn(_joy_estop_pause_btn):
                _joy_estop_pause_btn_pressed = True

            if _is_flow_motion:
                _curr_motion.y = _curr_joy.axes[ROLL_AXIS] * VY_SCALE
                _curr_motion.x = _curr_joy.axes[PITCH_AXIS] * VX_SCALE
            else:
                _curr_motion.x = _curr_joy.axes[ROLL_AXIS] * ROLL_SCALE
                _curr_motion.y = _curr_joy.axes[PITCH_AXIS] * PITCH_SCALE

                _curr_motion.x = _curr_motion.x if abs(_curr_motion.x) > ROLL_CLIP else 0
                _curr_motion.y = _curr_motion.y if abs(_curr_motion.y) > PITCH_CLIP else 0

            #common
            if args.enable_yaw:
                _curr_motion.yaw = _curr_joy.axes[YAW_AXIS] * YAW_SCALE
            else:
                _curr_motion.yaw = 0

            _curr_motion.dz = _curr_joy.axes[THROTTLE_AXIS] * THROTTLE_SCALE
            _curr_motion.dz = _curr_motion.dz if abs(_curr_motion.dz) > THROTTLE_CLIP else 0

            # adding action noise during recording
            if not args.no_action_noise and _is_flow_motion:
                _curr_motion.x += random.random() * VXY_NOISE_STD
                _curr_motion.y += random.random() * VXY_NOISE_STD
                _curr_motion.dz += random.random() * VZ_NOISE_STD

            if args.action_bounding:
                if 'extcam/target_vector' in _ros_msg_queue:
                    vec = _ros_msg_queue['extcam/target_vector'].vector
                    if vec.x != 0 or vec.y != 0 or vec.z != 0:
                        # assumes normalization
                        nx,ny,ndz = bound_action_by_target_loc(_curr_motion, vec)
                        _curr_motion.x = nx
                        _curr_motion.y = ny
                        _curr_motion.dz = ndz

            # extra bounding for altitude (to make sure CF and alt sensor are in their working range)
            if _ros_prefix + 'data' in _ros_msg_queue:
                data = _ros_msg_queue[_ros_prefix + 'data']
                if data.alt < 0.15 and _curr_motion.dz < 0:# minimum 15 cm off the ground
                    _curr_motion.dz = 0
                if data.alt > 1.0 and _curr_motion.dz > 0:# maximum is 1m off the ground
                    _curr_motion.dz = 0

            _curr_motion.is_flow_motion = _is_flow_motion

       ## END OF CB

       ## START OF BACKGROUND THREAD

    def _background_thread():
        global _curr_motion

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # don't publish motion if there is an active policy
            if not (args.policy == 'mpc' and _rosbag.is_open):
                _curr_motion.stamp.stamp = rospy.Time.now()
                _motion_pub.publish(_curr_motion)

                # drop things into the rosbag
                # _queue_lock.acquire()
                # _rosbag.write_all(_ros_msg_queue)
                # _buffer_lock.acquire()
                # _swap()
                # _queue_lock.release()
                # _buffer_lock.release()

            # else:

            #     temp_motion = CFMotion()
            #     temp_motion.stamp.stamp = rospy.Time.now()
            #     if _curr_joy.axes:
            #         temp_motion.y = _curr_joy.axes[ROLL_AXIS] * VY_SCALE
            #         temp_motion.x = _curr_joy.axes[PITCH_AXIS] * VX_SCALE
            #         temp_motion.yaw = _curr_joy.axes[YAW_AXIS] * YAW_SCALE

            #     temp_motion.is_flow_motion = True
            #     _motion_pub.publish(temp_motion)

            rate.sleep()


    ## END OF BACKGROUND THREAD

    ## START OF HELPER

    def _swap():
        for key in _ros_msg_queue_buffer:
            _ros_msg_queue[key] = _ros_msg_queue_buffer[key]
            _ros_msg_queue_buffer[key] = list()

    def _is_btn(btn, axes=False):
        if axes:
            if not _prev_joy.axes or not _curr_joy.axes:
                return False
            return not _prev_joy.axes[btn] and _curr_joy.axes[btn]
        else:
            if not _prev_joy.buttons or not _curr_joy.buttons:
                return False
            return not _prev_joy.buttons[btn] and _curr_joy.buttons[btn]

    def _reset():
        global _is_collision
        _is_collision = False

        global _joy_stop_trash_btn_pressed
        global _joy_stop_save_btn_pressed
        global _joy_estop_trash_btn_pressed
        global _joy_estop_pause_btn_pressed
        global _curr_motion

        _joy_stop_trash_btn_pressed = False
        _joy_stop_save_btn_pressed = False
        _joy_estop_trash_btn_pressed = False
        _joy_estop_pause_btn_pressed = False
    
        _curr_motion = CFMotion()
        _curr_motion.is_flow_motion = True

    def _ros_is_good(display=True):
        for topic in _ros_topics_and_types.keys():
            if 'data' in topic:
                if topic not in _ros_msg_times:
                    if display:
                        print('--> Topic {0} has never been received'.format(topic))
                    return False
                elapsed = (rospy.Time.now() - _ros_msg_times[topic]).to_sec()
                if elapsed > 2:
                    if display:
                        print('--> Topic {0} was received {1} seconds ago'.format(topic, elapsed))
                    return False
        return True

       ## END OF HELPER

    for topic, type in _ros_topics_and_types.items():
        # _rosbag_locks[topics] = threading.Lock()
        _ros_msg_queue[topic] = None
        # _ros_msg_queue_buffer[topic] = list()
        rospy.Subscriber(topic, type, msg_cb, (topic,))

    _joy_sub = rospy.Subscriber("/joy", Joy, joy_cb)
    _cmd_pub = rospy.Publisher(_ros_prefix + "command", CFCommand, queue_size=10)
    _motion_pub = rospy.Publisher(_ros_prefix + "motion", CFMotion, queue_size=10)
    
    threading.Thread(target=_background_thread).start()

    time.sleep(2.0)

    ## ISSUE TAKEOFF

    print("## TAKEOFF.")

    cmd = CFCommand()
    cmd.cmd = CFCommand.TAKEOFF
    cmd.stamp.stamp = rospy.Time.now()

    _cmd_pub.publish(cmd)

    time.sleep(5.0)

    while True:

        print('## Waiting for active topics...')

        while not _ros_is_good(display=False):
            pass

        print("## Waiting for target to be in frame...")
        rate = rospy.Rate(10)
        vec = _ros_msg_queue['extcam/target_vector'].vector
        while (vec.x < 1e-4 and vec.y < 1e-4):
            vec = _ros_msg_queue['extcam/target_vector'].vector
            rate.sleep()

        _adjustment_period = True
        print('## Waiting for Start Button...')
        while not _is_btn(_joy_start_btn):
            pass
        _adjustment_period = False

        # waiting for policy to be up
        if args.policy == 'mpc':
            _master.reset_sig() # forcing no signal received yet
            print('## Waiting for MPC Ack...')
            # sends in while loop every second, checks every 100ms
            _master.wait_send_sig(START, rate=rospy.Rate(10), resend=10)

        print('## Beginning Episode.')

        # _rosbag_lock.acquire()
        _rosbag.open(list(_ros_topics_and_types.keys()) + list(_ros_global_topics))
        # _rosbag_lock.release()
        
        print('## Resetting.')

        _reset()

        save = False

        #### START ITERATION
        rate = rospy.Rate(10)
        while   (
                    not _is_collision and
                    not _joy_stop_trash_btn_pressed and
                    not _joy_stop_save_btn_pressed and
                    not _joy_estop_trash_btn_pressed and
                    not _joy_estop_pause_btn_pressed and
                    not (args.policy == 'mpc' and _master.sender_stopped()) # servant send a kill request
                ):
            # _rosbag_lock.acquire()

            # _queue_lock.acquire()
            # new_topics = [topic for topic in _ros_topics_and_types.keys() if _ros_msg_queue[topic] is not None]
            # queue_cop = dict(_ros_msg_queue)
            # time_cop = dict(_ros_msg_times)

            # for key in new_topics:
            #     _ros_msg_queue[key] = None
            # _queue_lock.release()
            # print("Calling write...", rospy.Time.now().to_sec())
            # _rosbag.write_all(_ros_topics_and_types.keys(), queue_cop, time_cop)
            # # _rosbag_lock.release()
            rate.sleep()



        #### END ITERATION
        mpc_save = args.policy == 'mpc' and _master.sender_stopped() # todo add max iters

        if _joy_stop_save_btn_pressed or mpc_save:
            print("## Saving Rollout as", _rosbag._output_bag_name)
            save = True
        else:
            print("## Trashing Rollout.")

        # _rosbag_lock.acquire()
        if save:
            _rosbag.close()
        else:
            _rosbag.trash()
        # _rosbag_lock.release()
        # acknowledge the stop if we forced one
        if args.policy == 'mpc':
            _master.send_sig(STOP)
            # wait for ack only if we were the senders of STOP
            if not _master.sender_stopped():
                _master.reset_sig()
                _master.wait_sig(STOP, rate=rospy.Rate(10))

        if _joy_estop_pause_btn_pressed or _joy_estop_trash_btn_pressed:
            cmd = CFCommand()
            cmd.cmd = CFCommand.ESTOP
            cmd.stamp.stamp = rospy.Time.now()

            _cmd_pub.publish(cmd)
            time.sleep(2.0)

        if _joy_estop_pause_btn_pressed:
            print('## Paused. Waiting for Start Button.')
            while not _is_btn(_joy_start_btn):
                pass

        if _joy_estop_pause_btn_pressed or _joy_estop_trash_btn_pressed:
            cmd = CFCommand()
            cmd.cmd = CFCommand.TAKEOFF
            cmd.stamp.stamp = rospy.Time.now()

            _cmd_pub.publish(cmd)

            time.sleep(5.0)


    ## ISSUES LANDING (never happens)