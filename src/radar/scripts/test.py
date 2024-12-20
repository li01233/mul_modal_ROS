import numpy as np
import cv2
from cv_bridge import CvBridge

import rospy

from sensor_msgs.msg import Image

import filter as RadarFilter

class Test():
    def __init__(self):
        self.subscription = rospy.Subscriber('/USV/Polar',Image, 
                                             self.write_image, queue_size=10)
        
        self.bridge = CvBridge()

    def write_image(self, img):
        rospy.INFO(f'received new image')
        polar_image = self.bridge.imgmsg_to_cv2(img)
        cv2.imwrite('test/polar_image.jpg', polar_image)
        
        RadarFilter.generate_map(r=polar_image, k=None, p=None, K=None, area_threshold=50, gamma=None)


def main():
    rospy.init_node("test")
    test = Test()
    rospy.spin()


if __name__ == '__main__':
    main()
