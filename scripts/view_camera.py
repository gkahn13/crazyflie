import argparse

from PIL import Image
from io import BytesIO
import numpy as np
import matplotlib.pyplot as plt

import rospy
import sensor_msgs.msg


class ViewCamera(object):

    def __init__(self, topic):
        rospy.init_node('ViewCamera', anonymous=True)

        self._image_msg = None
        rospy.Subscriber(topic, sensor_msgs.msg.CompressedImage, self._image_callback)

    def _image_callback(self, msg):
        self._image_msg = msg

    def run(self):
        f, ax = plt.subplots()
        imshow = None

        rate = rospy.Rate(30.)

        while not rospy.is_shutdown():
            rate.sleep()

            msg = self._image_msg

            if msg is None:
                continue

            recon_pil_jpg = BytesIO(msg.data)
            recon_pil_arr = Image.open(recon_pil_jpg)
            im = np.array(recon_pil_arr)

            if imshow is None:
                imshow = ax.imshow(im, cmap='Greys_r')
            else:
                imshow.set_data(im)

            plt.pause(0.02)
            f.canvas.draw()

        plt.close(f)


parser = argparse.ArgumentParser()
parser.add_argument('--topic', type=str, default='/cf/0/image')
args = parser.parse_args()

view_camera = ViewCamera(topic=args.topic)
view_camera.run()