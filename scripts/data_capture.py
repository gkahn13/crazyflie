import rospy
import rosbag
import os
import os.path

import argparse
import time

import threading
import subprocess, shlex
import signal

from crazyflie.msg import CFData, CFMotion, CFCommand
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import CompressedImage, Imu, Joy
from std_msgs.msg import Bool


THROTTLE_AXIS = 5 # up 1
ROLL_AXIS = 2 #left 1
PITCH_AXIS = 3 #up 1
YAW_AXIS = 0 #left 1

#RP motion
THROTTLE_SCALE = 0.1
ROLL_SCALE = -25
PITCH_SCALE = -25
YAW_SCALE = -120

ROLL_CLIP = abs(ROLL_SCALE) * 0.4
PITCH_CLIP = abs(PITCH_SCALE) * 0.1

#standard motion
VX_SCALE = 1# 0.5
VY_SCALE = 1# 0.5


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
    parser.add_argument('--is-flow', type=bool, default=False)
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

    _ros_msg_times = dict()
    _ros_msg_queue = dict()
    # _ros_msg_queue_buffer = dict()
    _ros_topics_and_types = dict([ 
        # (_ros_prefix + 'image', CompressedImage),
        (_ros_prefix + 'data', CFData),
        (_ros_prefix + 'motion', CFMotion),
        (_ros_prefix + 'command', CFCommand),
        (_ros_prefix + 'imu', Imu),
        (_ros_prefix + 'pose', PoseStamped),
        (_ros_prefix + 'twist', TwistStamped),
        (_ros_prefix + 'coll', Bool),
        # ('/joy', Joy),
    ])

    _ros_global_topics = ['/tf', '/tf_static']
    ## START OF CB
    
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

        if _is_btn(_joy_stop_trash_btn):
            _joy_stop_trash_btn_pressed = True
        if _is_btn(_joy_stop_save_btn):
            _joy_stop_save_btn_pressed = True
        if _is_btn(_joy_estop_trash_btn):
            _joy_estop_trash_btn_pressed = True
        if _is_btn(_joy_estop_pause_btn):
            _joy_estop_pause_btn_pressed = True

        # need to translate curr motion
        _curr_motion = CFMotion()
        _curr_motion.is_flow_motion = _is_flow_motion
        
        # computing regular vx, vy, yaw, alt motion

        if _is_flow_motion:
            _curr_motion.y = _curr_joy.axes[ROLL_AXIS] * VY_SCALE
            _curr_motion.x = _curr_joy.axes[PITCH_AXIS] * VX_SCALE
        else:
            _curr_motion.x = _curr_joy.axes[ROLL_AXIS] * ROLL_SCALE
            _curr_motion.y = _curr_joy.axes[PITCH_AXIS] * PITCH_SCALE

            _curr_motion.x = _curr_motion.x if abs(_curr_motion.x) > ROLL_CLIP else 0
            _curr_motion.y = _curr_motion.y if abs(_curr_motion.y) > PITCH_CLIP else 0

        #common
        _curr_motion.yaw = _curr_joy.axes[YAW_AXIS] * YAW_SCALE

        _curr_motion.dz = 0


       ## END OF CB

       ## START OF BACKGROUND THREAD

    def _background_thread():
        global _curr_motion

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            if _rosbag.is_open:

                _curr_motion.stamp.stamp = rospy.Time.now()
                _motion_pub.publish(_curr_motion)

                # drop things into the rosbag
                # _queue_lock.acquire()
                # _rosbag.write_all(_ros_msg_queue)
                # _buffer_lock.acquire()
                # _swap()
                # _queue_lock.release()
                # _buffer_lock.release()

            else:

                temp_motion = CFMotion()
                temp_motion.stamp.stamp = rospy.Time.now()
                if _curr_joy.axes:
                    temp_motion.y = _curr_joy.axes[ROLL_AXIS] * VY_SCALE
                    temp_motion.x = _curr_joy.axes[PITCH_AXIS] * VX_SCALE
                    temp_motion.yaw = _curr_joy.axes[YAW_AXIS] * YAW_SCALE

                temp_motion.is_flow_motion = True
                _motion_pub.publish(temp_motion)

            rate.sleep()


    ## END OF BACKGROUND THREAD

    ## START OF HELPER

    def _swap():
        for key in _ros_msg_queue_buffer:
            _ros_msg_queue[key] = _ros_msg_queue_buffer[key]
            _ros_msg_queue_buffer[key] = list()

    def _is_btn(btn):
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

        print('## Waiting for active topics.')

        while not _ros_is_good(display=False):
            pass

        print('## Waiting for Start Button.')
        while not _is_btn(_joy_start_btn):
            pass
            
        print('## Beginning Episode.')

        # _rosbag_lock.acquire()
        _rosbag.open(_ros_topics_and_types.keys() + _ros_global_topics)
        # _rosbag_lock.release()
        
        print('## Resetting.')

        _reset()

        save = False

        #### START ITERATION
        rate = rospy.Rate(40)
        while   (
                    not _is_collision and
                    not _joy_stop_trash_btn_pressed and
                    not _joy_stop_save_btn_pressed and
                    not _joy_estop_trash_btn_pressed and
                    not _joy_estop_pause_btn_pressed
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

        if _joy_stop_save_btn_pressed:
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