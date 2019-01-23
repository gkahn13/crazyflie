import argparse

from PIL import Image
from io import BytesIO
import numpy as np
import matplotlib.pyplot as plt

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge


class CFtoSVO(object):

    def __init__(self, topic):
        rospy.init_node('CFtoSVO', anonymous=True)

        self._image_msg = None
        rospy.Subscriber(topic, sensor_msgs.msg.CompressedImage, self._image_callback)

        self._bridge = CvBridge()

        self._svo_pub = rospy.Publisher('/camera/image_raw', sensor_msgs.msg.Image, queue_size=10)

    def _image_callback(self, msg):
        self._image_msg = msg

    def run(self):
        rate = rospy.Rate(30.)

        while not rospy.is_shutdown():
            rate.sleep()

            msg = self._image_msg

            if msg is None:
                continue

            recon_pil_jpg = BytesIO(msg.data)
            recon_pil_arr = Image.open(recon_pil_jpg)
            im = np.array(recon_pil_arr)

            self._svo_pub.publish(self._bridge.cv2_to_imgmsg(im, 'mono8'))

parser = argparse.ArgumentParser()
parser.add_argument('--topic', type=str, default='/cf/0/image')
args = parser.parse_args()

cf_to_svo = CFtoSVO(topic=args.topic)
cf_to_svo.run()