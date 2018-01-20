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


THROTTLE_AXIS = 1 # up 1
ROLL_AXIS = 2 #left 1
PITCH_AXIS = 3 #up 1
YAW_AXIS = 0 #left 1

THROTTLE_SCALE = 0.5
ROLL_SCALE = 0.5
PITCH_SCALE = 0.5
YAW_SCALE = 0.5

TAKEOFF_CHANNEL = 7 #RT
ESTOP_CHANNEL = 2 #B
LAND_CHANNEL = 6 #LT
UNLOCK_ESTOP_CHANNEL = 0 #X

TOLERANCE = 0.05

#A is 1, Y is 3

class JoyController(Controller):

    def __init__(self, ID, joystick_topic):
        Controller.__init__(self, ID)
        self.joy_sub = rospy.Subscriber(joystick_topic, Joy, joy_cb)
        self.curr_joy = None

        self.cmd = -1 # -1 : NONE

    #Override
    def compute_motion(self):
        #pulls latest joystick data
        print("Computing Motion From Joystick")

        motion = None

        if self.cmd != -1:
            motion = CFCommand()
            if self.cmd == CFCommand.ESTOP:
                motion.cmd = CFCommand.ESTOP

            elif self.cmd == CFCommand.TAKEOFF:
                motion.cmd = CFCommand.TAKEOFF

            elif self.cmd == CFCommand.LAND:
                motion.cmd = CFCommand.LAND

            #reset
            self.cmd = -1

        #repeat send at 10Hz
        elif self.curr_joy:
            motion = CFMotion()
            #TODO: update range
            motion.alt = self.data.alt + self.curr_joy.axes[THROTTLE_AXIS] * THROTTLE_SCALE
            motion.vx = self.curr_joy.axes[ROLL_AXIS] * ROLL_SCALE
            motion.vy = self.curr_joy.axes[PITCH_AXIS] * PITCH_SCALE
            motion.yaw = self.curr_joy.axes[YAW_AXIS] * YAW_SCALE
            
        return motion


    def dead_band(self, signal):
        for i in range(len(signal.axes)):
            signal.axes[0] = signal.axes[0] if abs(signal.axes[0]) > TOLERANCE else 0


    def joy_cb(self, msg):
        if msg.buttons[ESTOP_CHANNEL] and not self.curr_joy.buttons[ESTOP_CHANNEL]:
            #takeoff
            self.cmd = CFCommand.ESTOP
        elif msg.buttons[TAKEOFF_CHANNEL] and not self.curr_joy.buttons[TAKEOFF_CHANNEL]:
            #takeoff
            self.cmd = CFCommand.TAKEOFF
        elif msg.buttons[LAND_CHANNEL] and not self.curr_joy.buttons[LAND_CHANNEL]:
            #takeoff
            self.cmd = CFCommand.LAND
            
        self.dead_band(msg)
        self.curr_joy = msg
