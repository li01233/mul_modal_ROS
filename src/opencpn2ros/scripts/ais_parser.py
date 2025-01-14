#! /usr/bin/env python3
import rospy
import socket
import struct
from pyais import decode
from message_interface.msg import AIS

class NmeaAISParser:
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
        
        self.pub = rospy.Publisher("/ais", AIS, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.run)

        self.ais = AIS()

    def run(self, event):
        data, _ = self.sock.recvfrom(1024)
        nmea_sentence = data.decode()
        self.parse(nmea_sentence)

    def parse(self, nmea_sentence):
        decoded = decode(nmea_sentence)
        self.msg2ros(decoded)
    
    def msg2ros(self, ais_message):
        self.ais.header.stamp = rospy.Time.now()
        self.ais.message_type = ais_message.msg_type.name
        self.ais.repeat_indicator = ais_message.repeat_indicator
        self.ais.MMSI = ais_message.mmsi
        self.ais.nav_status = ais_message.nav_status
        self.ais.turn = ais_message.turn
        self.ais.speed = ais_message.speed
        self.ais.accuracy = ais_message.accuracy
        self.ais.latitude = ais_message.lat
        self.ais.longitude = ais_message.lon
        self.ais.course = ais_message.course
        self.ais.heading = ais_message.heading
        self.ais.second = ais_message.second
        self.ais.maneuver = ais_message.maneuver
        self.ais.raim = ais_message.raim
        self.ais.radio = ais_message.radio
        self.pub.publish(self.ais)


if __name__ == "__main__":
    rospy.init_node("ais_parser")
    parser = NmeaAISParser()
    rospy.spin()