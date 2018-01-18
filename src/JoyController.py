import rospy
# import cv_bridge
# from cv_bridge import CvBridge, CvBridgeError
# import cv2


from sensor_msgs.msg import Image
from crazyflie.msg import CFData
# from crazyflie.msg import CFImage
from crazyflie.msg import CFCommand
from crazyflie.msg import CFMotion

from sensor_msgs import Joy

import Controller


THROTTLE_AXIS = 0
PITCH_AXIS = 1
ROLL_AXIS = 3
YAW_AXIS = 4

class JoyController(Controller):
    def __init__(self, ID, joystick_topic):
        Controller.__init__(self, ID)
        self.joy_sub = rospy.Subscriber(joystick_topic, Joy, joy_cb)
        self.curr_joy = None

    #Override
    def compute_motion(self):
        #pulls latest joystick data
        print("Computing Motion From Joystick")
        motion = CFMotion()
        if self.curr_joy:
            #TODO: update range
            motion.alt = self.data.alt + self.curr_joy.axis[THROTTLE_AXIS]
            motion.vx = self.curr_joy.axis[ROLL_AXIS]
            motion.vy = self.curr_joy.axis[PITCH_AXIS]
            motion.yaw = self.curr_joy.axis[YAW_AXIS]

        return motion

    def joy_cb(self, msg):
        self.curr_joy = msg
