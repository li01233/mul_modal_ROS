from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import rospy

# 创建 publisher
pub1 = rospy.Publisher("/img1", Image, queue_size=1)
pub2 = rospy.Publisher("/img2", Image, queue_size=1)
pub3 = rospy.Publisher("/img3", Image, queue_size=1)
pub4 = rospy.Publisher("/img4", Image, queue_size=1)

def unzip1(msg):
    bridge = CvBridge()
    cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    unzip_msg = CvBridge().cv2_to_imgmsg(cv_image, "bgr8")
    pub1.publish(unzip_msg)

def unzip2(msg):
    bridge = CvBridge()
    cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    unzip_msg = CvBridge().cv2_to_imgmsg(cv_image, "bgr8")
    pub2.publish(unzip_msg)

def unzip3(msg):
    bridge = CvBridge()
    cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    unzip_msg = CvBridge().cv2_to_imgmsg(cv_image, "bgr8")
    pub3.publish(unzip_msg)

def unzip4(msg):
    bridge = CvBridge()
    cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    unzip_msg = CvBridge().cv2_to_imgmsg(cv_image, "bgr8")
    pub4.publish(unzip_msg)


if __name__ == "__main__":
    rospy.init_node("unzip")
    # 创建Subscriber
    sub1 = rospy.Subscriber("/compressedimg1", CompressedImage, unzip1)
    sub2 = rospy.Subscriber("/compressedimg2", CompressedImage, unzip2)
    sub3 = rospy.Subscriber("/compressedimg3", CompressedImage, unzip3)
    sub4 = rospy.Subscriber("/compressedimg4", CompressedImage, unzip4)
    rospy.spin()
