#! /usr/bin/env python
import rospy
from hik_tem.msg import PtzCtrl
from opencpn2ros.msg import RadarTarget

mode = "Auto"

def target_pub(msg):
    p = PtzCtrl()
    p.Pan = msg[0].pitch
    p.Tilt = 0
    p.Zoom = 0
    p.Pan %= 360
    pub.publish(p)
    

if __name__ == "__main__":
    rospy.init_node("ptz_ctrl")
    pub = rospy.Publisher("/ptz_ctrl", PtzCtrl, queue_size=10)
    
    # 巡航模式
    if mode == "Auto":
        p = PtzCtrl()
        p.Pan = 135
        p.Tilt = 0
        p.Zoom = 0
        while not rospy.is_shutdown():
            pub.publish(p)  #发布消息
            p.Pan += 10
            p.Pan %= 360
            
            rospy.sleep(3)  #休眠
            rospy.loginfo("Pan:%s, Tilt:%d, Zoom:%d",p.Pan, p.Tilt, p.Zoom)
    elif mode == "Manual":
        rospy.Subscriber("/radar_target", RadarTarget, target_pub, queue_size=10)
    else:
        rospy.loginfo("Invalid mode")
