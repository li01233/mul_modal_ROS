#! /usr/bin/env python3
import rospy
import socket
import struct
from message_interface.msg import RadarTarget
import time

class NmeaARPAParser:
    def __init__(self):
        # 创建UDP套接字
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 设置时间生存值（TTL）
        ttl = struct.pack('b', 2)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

        # 读取param
        self.ip = rospy.get_param('~ip', '0.0.0.1')
        self.port = rospy.get_param('~port', 8888)
        # 绑定本地地址和端口
        local_address = (self.ip, self.port)
        self.sock.bind(local_address)
        
        self.pub = rospy.Publisher("/ptz_ctrl", RadarTarget, queue_size=10)
        self.timer_target = rospy.Timer(rospy.Duration(5), self.pub_target)
        self.timer_target_del = rospy.Timer(rospy.Duration(5), self.del_target)
        self.timer_target_add = rospy.Timer(rospy.Duration(0.1), self.run)

        self.radar_target = RadarTarget()
        self.target = {}

    def run(self, event):
        data, _ = self.sock.recvfrom(1024)
        nmea_sentence = data.decode()
        self.parse(nmea_sentence)

    def parse(self, nmea_sentence):
        time = time.time()
        if nmea_sentence.startswith("$RATTM"):
            fields = nmea_sentence.split(",")
            # 解析字段
            target_number = int(fields[1])
            target_distance = float(fields[2])
            bearing_from_own_ship = float(fields[3])
            bearing_units = fields[4]
            target_speed = float(fields[5])
            target_course = float(fields[6])
            course_units = fields[7]
            distance_to_cpa = float(fields[8])
            time_to_cpa = float(fields[9])
            speed_distance_units = fields[10]
            target_name = fields[11]
            target_status = fields[12]
            reference_target = fields[13]
            
            if target_number not in self.target:
                self.target[target_number] = {
                    'target_distance': target_distance,
                    'bearing_from_own_ship': bearing_from_own_ship,
                    'bearing_units': bearing_units,
                    'target_speed': target_speed,
                    'target_course': target_course,
                    'course_units': course_units,
                    'distance_to_cpa': distance_to_cpa,
                    'time_to_cpa': time_to_cpa,
                    'speed_distance_units': speed_distance_units,
                    'target_name': target_name,
                    'target_status': target_status,
                    'reference_target': reference_target,
                    'time': time
                }
    # 发布距离自己最近的目标的角度
    def pub_target(self, event):
        if len(self.target) == 0:
            return
        
        min_target_distance = min(target['target_distance'] for target in self.target.values())
        min_target = next(target for target in self.target.values() if target['target_distance'] == min_target_distance)
        self.radar_target.pitch = min_target['bearing_from_own_ship'] 
        self.pub.publish(self.radar_target) 
    
    # 超过5s删除目标
    def del_target(self, event):
        if len(self.target) == 0:
            return
        
        for target in self.target.values():
            if time.time() - target['time'] > 5:
                del target


if __name__ == "__main__":
    rospy.init_node("arpa_parser")
    parser = NmeaARPAParser()
    rospy.spin()