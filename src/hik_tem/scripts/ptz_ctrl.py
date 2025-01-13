#! /usr/bin/env python
import rospy
from message_interface.msg import RadarTarget,PtzCtrl
from pynput import keyboard
import time

class PTZController:
    def __init__(self, mode= "Manual"):
        self.p = PtzCtrl()
        self.p.Pan = 135
        self.p.Tilt = 0
        self.p.Zoom = 0
        self.mode = mode

        self.pub = rospy.Publisher("/ptz_ctrl", PtzCtrl, queue_size=10)
        self.listener = keyboard.Listener(on_press=self.on_press,on_release=self.on_release)
        
        self.sub = None
        
        self.init_ptz() # 2s初始化云台
        self.listener.start()
        # 如果不是Auto，则关闭订阅器
        if self.mode != "Auto":
            self.close_subscriber()
        
        self.cruise_flag = False
        if self.mode == "Fix":
            self.cruise_flag = True
        self.timer = rospy.Timer(rospy.Duration(2), self.cruise)

        self.key_pressed_time = 0

    def init_ptz(self):
        start_time = time.time()
        while time.time() - start_time < 2:
            self.pub.publish(self.p)
        rospy.loginfo(f"PTZ Initialized")

    def target_pub(self, msg):
        # 在这里写处理导航目标检测数据的逻辑
        self.p.Pan = msg[0].pitch
        self.p.Pan %= 360
        rospy.loginfo("Pan:%s, Tilt:%d, Zoom:%d",self.p.Pan, self.p.Tilt, self.p.Zoom)
        self.pub.publish(self.p)

    def on_press(self, key):
        try:
            # 若是巡航模式或自动，则不响应按键
            if self.mode == "Fix" or self.mode == "Auto":
                return
            # 手动模式下，响应按键
            if key.char == 'a':
                self.p.Pan -= 10
                self.p.Pan %= 360
                rospy.loginfo("Turn Left, Pan: %d", self.p.Pan)
            elif key.char == 'd':
                self.p.Pan += 10
                self.p.Pan %= 360
                rospy.loginfo("Turn Right, Pan: %d", self.p.Pan)
            elif key.char == 'w':
                self.p.Tilt += 5
                self.p.Tilt = 90 if self.p.Tilt > 90 else self.p.Tilt
                rospy.loginfo("Turn Up, Tilt: %d", self.p.Tilt)
            elif key.char == 's':
                self.p.Tilt -= 5
                self.p.Tilt = 0 if self.p.Tilt < -0 else self.p.Tilt
                rospy.loginfo("Turn Down, Tilt: %d", self.p.Tilt)
            self.pub.publish(self.p)

            if key == keyboard.Key.space:
                self.key_pressed_time = time.time()
        except AttributeError:
            pass

    def on_release(self, key):
        press_duration = time.time() - self.key_pressed_time
        if key == keyboard.Key.space and press_duration > 1: # 如果是空格键，且按下时间超过2秒，则切换模式
            if self.mode == "Manual" or self.mode == "Fix":
                self.mode = "Auto"
                self.open_subscriber() # 打开订阅者
                self.cruise_flag = False
            else:
                self.mode = "Manual" # 自动模型会切换为手动模式，需要再短按进入巡航模式
                self.close_subscriber() # 关闭订阅者
                self.cruise_flag = False
        elif key == keyboard.Key.space and press_duration < 2: # 如果是空格键，且按下时间小于2秒，则切换手动或巡航模式
            if self.mode == "Manual":
                self.mode = "Fix"
                self.cruise_flag = True
            elif self.mode == "Fix":
                self.mode = "Manual"
                self.cruise_flag = False

    def close_subscriber(self):
        if self.sub is not None:
            self.sub.unregister()  # 关闭订阅者
            self.sub = None        

    def open_subscriber(self):
        if self.sub is None:
            self.sub = rospy.Subscriber("/radar_target", RadarTarget, self.target_pub)

    def cruise(self, event):
        while self.cruise_flag:
            self.p.Pan += 10
            self.p.Pan %= 360
            self.pub.publish(self.p)
            rospy.sleep(2)
            rospy.loginfo("Pan:%s, Tilt:%d, Zoom:%d",self.p.Pan, self.p.Tilt, self.p.Zoom)

if __name__ == "__main__":
    rospy.init_node("ptz_ctrl")
    controller = PTZController()
    rospy.spin()
