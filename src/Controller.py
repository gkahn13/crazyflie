import rospy
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
import cv2


from sensor_msgs.msg import Image
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
    def __init__(self, ID):
        self.id = ID

        self.bridge = cv_bridge.CvBridge()

        self.mat = None
        self.data = None

        #need to facilitate a set of publishers per cf node

        self.data_sub = rospy.Subscriber('cf/%d/data' % ID, CFData, data_cb)
        self.image_sub = rospy.Subscriber('cf/%dimage' % ID, Image, image_cb)

        self.cmd_pub = rospy.Publisher('cf/%d/command'% self.id, CFCommand, queue_size=10)
        self.motion_pub = rospy.Publisher('cf/%d/motion'% self.id, CFMotion, queue_size=10)

    ## HELPERS ##

    def convert_to_cv(self, msg):
        if msg is None:
            return None

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        except cv_bridge.CvBridgeError as e:
            return None

        return cv_image

    def compute_motion(self):
        return None



    ## CALLBACKS ## 

    def image_cb(self, msg):
        self.mat = self.convert_to_cv(msg)
        pass

    def data_cb(self, msg):
        self.data = msg
        pass

    ## SETTERS ##

    ## THREADS ##
    def run(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            action = self.compute_motion()
            if isinstance(action, CFMotion):
                self.motion_pub.publish(action)
            elif isinstance(action, CFCommand):
                self.cmd_pub.publish(action)
                print "CALLED COMMAND -> %s" % cmd_type[action.cmd]
            else:
                pass

            rospy.spinOnce()
            rate.sleep()
