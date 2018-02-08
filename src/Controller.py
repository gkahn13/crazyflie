import rospy
import numpy as np
# import cv_bridge
# from cv_bridge import CvBridge, CvBridgeError

from PIL import Image

import io

import cv2


from sensor_msgs.msg import CompressedImage
from crazyflie.msg import CFData
# from crazyflie.msg import CFImage
from crazyflie.msg import CFCommand
from crazyflie.msg import CFMotion


## for convenience
cmd_type = ['']*3
cmd_type[CFCommand.ESTOP] = 'ESTOP'
cmd_type[CFCommand.LAND] = 'LAND'
cmd_type[CFCommand.TAKEOFF] = 'TAKEOFF'

class Controller:

    DO_NOTHING_CMD = CFMotion()


    def __init__(self, ID):
        self.id = ID

        self.mat = None
        self.data = None

        #need to facilitate a set of publishers per cf node

        self.data_sub = rospy.Subscriber('cf/%d/data' % ID, CFData, self.data_cb)
        self.image_sub = rospy.Subscriber('cf/%d/image' % ID, CompressedImage, self.image_cb)

        self.cmd_pub = rospy.Publisher('cf/%d/command'% self.id, CFCommand, queue_size=10)
        self.motion_pub = rospy.Publisher('cf/%d/motion'% self.id, CFMotion, queue_size=10)

    def compute_motion(self):
        print("Doing nothing -- ")
        return None



    ## CALLBACKS ## 

    def image_cb(self, msg):
        pil_jpg = io.BytesIO(msg.data)
        pil_arr = Image.open(pil_jpg).convert('L') #grayscale

        self.mat = np.array(pil_arr)

        # self.mat = self.convert_to_cv(msg)
        pass

    def data_cb(self, msg):
        # print("----------> Data CB: <--------")
        # print(msg)
        # print("------------------------------")
        self.data = msg
        pass

    ## SETTERS ##

    ## THREADS ##
    def run(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            action = self.compute_motion()
            if isinstance(action, CFMotion):
                # if action != Controller.DO_NOTHING_CMD:
                self.motion_pub.publish(action)
                # else:
                #     print("--- DO NOTHING CMD SENT ---")
            elif isinstance(action, CFCommand):
                self.cmd_pub.publish(action)
                print( "CALLED COMMAND -> %s" % cmd_type[action.cmd])

            else:
                pass
            
            # rospy.spinOnce()
            rate.sleep()
