#include "IMU.h"
#include <arpa/inet.h>

#include <cstdint>
#include <vector>
#include <cmath>
#include <iostream>

// 定义一个常量来表示 IMUHeader
const std::vector<unsigned char> Imu::IMUHeader = {0xAA, 0x55};

Imu::Imu() {}

void Imu::parse()
{
    if(getDeviceBuffer().empty())
        return;

    std::vector<unsigned char>& _channelBuffer = getDeviceBuffer();

    if(_channelBuffer.size() < sizeof(IMUFrame))
        return;

    if(_channelBuffer[0] == 0xAA && _channelBuffer[1] == 0x55) {
        int st = 0; // 假设IMUHeader是连续的两个字节0xAA和0x55
        if(st >= 0 && _channelBuffer.size() >= st + sizeof(IMUFrame)) {
            //目前仅处理数据，未处理错误信息
            IMUFrame* frame = (IMUFrame*)(_channelBuffer->data() + st);
            _channelBuffer.erase(_channelBuffer.begin(), _channelBuffer.begin() + st + sizeof(IMUFrame)); //删除已经查询到的数据
            if(frame->verifyChecksum()) {
                tData.head	   = frame->head * 0.0001;
                tData.pitch	   = frame->pitch * 0.0001;
                tData.roll	   = frame->roll * 0.0001;
                tData.headRot  = frame->headRatio * 0.0001 * 180 / M_PI * 60;
                tData.pitchRot = frame->pitchRatio * 0.0001 * 180 / M_PI * 60;
                tData.rollRot  = frame->rollRatio * 0.0001 * 180 / M_PI * 60;

                sig_SendImuData(tData);

            } else {
                std::cerr << "no pass verify ,device=imu" << std::endl;
            }
        }
    }

    //发布数据
    ROS_INFO("Stream CallBack, IMU DATA IS RECEIVED.");
    pub.publish(std::to_string(tData.head));
}

std::vector<unsigned char>& Imu::getDeviceBuffer()
{
    return _Buffer;
}

void Imu::readUdpPendingDatagrams()
{
    auto socket = static_cast<UdpSocket*>(_socket);

    while(socket->hasPendingDatagrams()) {

        auto datagram = socket->receiveDatagram();

        //拼包
        if(isJointUdp) {
            if(_Buffer.length() + datagram.data().length() > MAX_BUFFER_LEN)
                _Buffer.clear();

            _Buffer += datagram.data();
        } else
            _Buffer = datagram.data();
    }
}


bool Imu::initIMU(in_addr_t ip_addr_n, in_port_t port_n)
{
    if(_socket != -1){
        struct sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);

        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;       // Use IPV4
        addr.sin_port   = port_n;
        addr.sin_addr.s_addr = ip_addr_n;

        if(bind(_socket, (struct sockaddr*)&addr, addr_len) == -1){
            printf("Failed to bind socket on port");
            close(_socket);
            return false;
        }
    }
    return true;   
}

bool Imu::initROSIO(ros::NodeHandle& priv_node)
{
    /// imu parameter
    priv_node.param<std::string>("ip_addr", ip_addr,"192.168.5.100");
    ROS_INFO("ip address:\t%s", ip_addr.c_str());

    priv_node.param<int>("port",port, 8000);
    ROS_INFO("port:      \t%d", port);
    
    // turn it to network byte order
    in_addr_t ip_addr_n = inet_addr(ip_addr.c_str());
    if (ip_addr_n == INADDR_NONE) {
        std::cerr << "Invalid IP address: " << info._bindIP << std::endl;
        return false;
    }
    in_port_t port_n = htons(port);
    
    // bind socket and init publisher
    if(initIMU(ip_addr_n, port_n))
    {
        ROS_INFO("initIMU success");
        pub = priv_node->advertise<std_msgs::String>("/imu",10);
        timer = priv_node->createTimer(ros::Duration(0.01),readUdpPendingDatagrams,false,true);
        return true;
    }
    else
    {
        ROS_INFO("initIMU fail");
        return false;
    }
}

void Imu::run()
{
    ros::NodeHandle priv_node("~");

    if(initROSIO(priv_node))ros::spin();
}