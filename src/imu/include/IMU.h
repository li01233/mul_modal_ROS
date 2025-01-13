#ifndef Imu_H
#define Imu_H

#include <vector>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <cmath>
#include <arpa/inet.h>


#include "std_msgs/String.h"
#include "message_interface/EnvData.h"
#include "message_interface/Ownship.h"
#include "rapidjson/document.h"


#define BUFF_LEN 1024


class Imu {
public:
    explicit Imu();
    void run();

private:
    void parse(char *buffer);
    void parseOwnShip(rapidjson::Document& doc);
    void parseEnvData(rapidjson::Document& doc);

    void readUdpPendingDatagrams(const ros::TimerEvent& event);
    bool initIMU(in_addr_t ip_addr_n, in_port_t port_n);
    bool initROSIO(ros::NodeHandle& priv_node);

private:
    int _socket = -1; 
    char buffer[BUFF_LEN];
    bool isJointUdp = false; //是否开启UDP拼包模式，默认式不开启
    struct sockaddr_in addr;

    ros::Publisher pub_ship, pub_env;
    ros::Timer timer;

    std::string ip_addr;
    int port;

};

#endif // Imu_H
