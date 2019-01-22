import rospy
from sensor_msgs.msg import Joy
from crazyflie.msg import CFCommand, CFMotion
from Crazyflie import Crazyflie


THROTTLE_AXIS = 5 # up 1
ROLL_AXIS = 2 # left 1
PITCH_AXIS = 3 # up 1
YAW_AXIS = 0 # left 1

VX_SCALE = 1.5
VY_SCALE = 1.5

THROTTLE_SCALE = 0.03
YAW_SCALE = -180

# ROLL_CLIP = abs(ROLL_SCALE) * 0.4
# PITCH_CLIP = abs(PITCH_SCALE) * 0.1

TAKEOFF_CHANNEL = 7 # RT
ESTOP_CHANNEL = 2 # B
LAND_CHANNEL = 6 # LT
UNLOCK_ESTOP_CHANNEL = 0 # X

TOLERANCE = 0.05
ALT_TOLERANCE = 0.08

class JoystickCommander:

    def __init__(self, id, joystick_topic):
        self._height = Crazyflie.DEFAULT_HEIGHT

        self._command_pub = rospy.Publisher('cf/{0:d}/command'.format(id), CFCommand, queue_size=10)
        self._motion_pub = rospy.Publisher('cf/{0:d}/motion'.format(id), CFMotion, queue_size=10)

        self._joy_msg = None
        rospy.Subscriber(joystick_topic, Joy, self._joy_callback)

    #####################
    ### ROS callbacks ###
    #####################

    def _joy_callback(self, msg):
        self._joy_msg = msg

    ################
    ### Commands ###
    ################

    def _apply_deadband(self, joy_msg):
        new_axes = [0] * len(joy_msg.axes)
        for i in range(len(joy_msg.axes)):
            new_axes[i] = joy_msg.axes[i] if abs(joy_msg.axes[i]) > TOLERANCE else 0
        joy_msg.axes = new_axes
        return joy_msg

    ###########
    ### Run ###
    ###########

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            rate.sleep()

            joy_msg = self._joy_msg
            if joy_msg is None:
                continue

            # takeoff / land / estop
            if joy_msg.buttons[TAKEOFF_CHANNEL]:
                self._command_pub.publish(CFCommand(cmd=CFCommand.TAKEOFF))
                self._height = Crazyflie.DEFAULT_HEIGHT
            elif joy_msg.buttons[LAND_CHANNEL]:
                self._command_pub.publish(CFCommand(cmd=CFCommand.LAND))
                self._height = 0.
            elif joy_msg.buttons[ESTOP_CHANNEL]:
                self._command_pub.publish(CFCommand(cmd=CFCommand.ESTOP))
                self._height = 0.


            # motion
            joy_msg = self._apply_deadband(joy_msg)

            motion_msg = CFMotion()
            motion_msg.is_flow_motion = True

            motion_msg.x = joy_msg.axes[PITCH_AXIS] * VX_SCALE
            motion_msg.y = joy_msg.axes[ROLL_AXIS] * VY_SCALE

            # motion.x = motion.x if abs(motion.x) > ROLL_CLIP else 0
            # motion.y = motion.y if abs(motion.y) > PITCH_CLIP else 0

            motion_msg.yaw = joy_msg.axes[YAW_AXIS] * YAW_SCALE
            self._height += joy_msg.axes[THROTTLE_AXIS] * THROTTLE_SCALE
            motion_msg.height = self._height

            self._motion_pub.publish(motion_msg)