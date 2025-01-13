import numpy as np
import cv2
from cv_bridge import CvBridge

import rospy

from std_msgs.msg import String
from message_interface.msg import Spoke
from sensor_msgs.msg import Image

MAX_SPOKE_LENGTH = 256
MAX_INTENSITY = 128
MAX_SPOKE_COUNT = 250

class Slam():
    def __init__(self):
        self.image_publisher_ = rospy.Publisher("/radar_image", Image, queue_size=10)

        self.subscription = rospy.Subscriber('/topic_radar_spoke',Spoke, 
                                             self.radar_spoke_callback, queue_size=10)
        
        # request polar image every 3 second
        self.polar_image_timer  = rospy.Timer(rospy.Duration(2.5),self.imager_callback)
        
        self.polar_image_publisher = rospy.Publisher('/Navtech/Polar', Image, queue_size=10)
        
        self.spokes = np.zeros((MAX_SPOKE_COUNT, MAX_SPOKE_LENGTH), np.uint8)
        self.spokes_updated = 0
        self.bridge = CvBridge()


    def radar_spoke_callback(self, spoke):
        if self.spokes is None: return

        self.spokes[spoke.azimuth, :len(spoke.data)] = spoke.data
        self.spokes_updated += 1


    def imager_callback(self, event):
        polar_image = self.get_polar_image()
        self.polar_image_publisher.publish(self.bridge.cv2_to_imgmsg(polar_image, encoding="passthrough"))
        
        # self.generate_map(r=polar_image, k=None, p=None, K=None, area_threshold=50, gamma=None)
        
        self.spokes = np.zeros((MAX_SPOKE_COUNT, MAX_SPOKE_LENGTH), np.uint8)
        self.spokes_updated = 0


    def get_polar_image(self):
        # Uncomment below for testing
        # self.spokes = np.random.uniform(low=0, high=MAX_INTENSITY, size=(250, MAX_SPOKE_LENGTH)) # random spokes
        # polar_image = np.copy(self.spokes/MAX_INTENSITY * 255).astype(np.uint8)
        polar_image = np.copy(self.spokes/MAX_INTENSITY * 255).astype(np.uint8)
        
        # cv2.imshow('polar image', polar_image); cv2.waitKey(0)
        cv2.imwrite('test/polar_image.jpg', polar_image)
        
        return polar_image


def main():
    rospy.init_node("slam")
    slam= Slam()
    rospy.spin()



if __name__ == '__main__':
    main()
