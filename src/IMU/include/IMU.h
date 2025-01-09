#ifndef Imu_H
#define Imu_H

#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstdint>
#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/socket.h>

#include "IMU/ImuData.h"

using quint32_be = std::uint32_t;
using qint16_be = std::int16_t;

// 1字节对齐
#pragma pack(1)

//协议类型
enum class IMUFrameType : unsigned char { Zero = 0, One = 1 };

enum class IMUDeviceType : unsigned char {
    Init = 0x40, //初始对准状态
    Navi = 0x41	 //导航状态
};

enum class ErrorFlag : unsigned short {
    ACC_X_ERR  = 0x01,		//加速度计X故障
    ACC_Y_ERR  = 0x02,		//加速度计Y故障
    ACC_Z_ERR  = 0x04,		//加速度计Z故障
    SPIN_X_ERR = 0x10,		//陀螺X故障
    SPIN_Y_ERR = 0x20,		//陀螺Y故障
    SPIN_Z_ERR = 0x40,		//陀螺Z故障
    TEMP_ERR   = 0x80,		//温度传感器故障
    TEMP_X_ERR = 0x01 << 8, //温度传感器X故障
    TEMP_Y_ERR = 0x02 << 8, //温度传感器Y故障
    TEMP_Z_ERR = 0x04 << 8	//温度传感器Z故障
};

enum class AlignFlag : unsigned char {
    Start_Dock		  = 0x00, //码头启动
    Start_Sea_Pos	  = 0x01, //海上启动-位置组合
    Start_Sea_Pos_Err = 0x11, //海上启动-位置组合-失败
    Start_V			  = 0x02, //海上启动-速度组合
    Start_V_Err		  = 0x12  //海上启动-速度组合-失败
};

enum class DataValidFlag : unsigned char { ValidPos = 0x01, ValidV = 0x10 };

//惯性导航测量装置
struct IMUFrame {
    short header;
    char  frameType;
    char  DeviceType;
    //航向-大端序，单位0.0001度
    quint32_be head;
    //纵摇-大端序，单位0.0001度
    qint32_be pitch;
    //横摇-大端序，单位0.0001度
    qint32_be roll;
    //航向角速率，单位0.001弧度/秒
    qint16_be headRatio;
    //纵摇角速率，单位0.001弧度/秒
    qint16_be pitchRatio;
    //横摇角速率，单位0.001弧度/秒
    qint16_be rollRatio;

    int			  alignTime;   //对准时间
    char		  reserved[3]; //保留
    ErrorFlag	  errFlag;	   //元件故障标志位
    AlignFlag	  alignFlag;   //对准方式标志
    DataValidFlag validFlag;
    char		  unused[74]; //其余未使用
    char		  checksum;	  //校验字节

    bool verifyChecksum()
    {
        char* p	   = &frameType;
        char  rslt = 0;
        for(int i = 0; i < 106 - 2; i++) {
            rslt += *(p + i);
        }
        return rslt == this->checksum;
    }

    void toNetworkByteOrder()
    {
        head = htonl(head);
        pitch = htonl(pitch);
        roll = htonl(roll);
        headRatio = htons(headRatio);
        pitchRatio = htons(pitchRatio);
        rollRatio = htons(rollRatio);
    }

    void fromNetworkByteOrder()
    {
        head = ntohl(head);
        pitch = ntohl(pitch);
        roll = ntohl(roll);
        headRatio = ntohs(headRatio);
        pitchRatio = ntohs(pitchRatio);
        rollRatio = ntohs(rollRatio);
    }
};

#pragma pack()

struct sockaddr_in{
    unsigned short         sin_family;    
    unsigned short int     sin_port;      
    struct in_addr         sin_addr;      
    unsigned char          sin_zero[8];   
};

struct in_addr{
    unsigned long     s_addr;
};


class Imu {
public:
    explicit Imu();
    void Imu::run();

private:
    virtual void parse() override;
    std::vector<unsigned char>& getDeviceBuffer();
    void Imu::readUdpPendingDatagrams();
    bool Imu::initIMU(in_addr_t ip_addr_n, in_port_t port_n);
    bool Imu::initROSIO(ros::NodeHandle& priv_node);

private:
    static const std::vector<unsigned char> IMUHeader;
    ImuData tData;
    
    int _socket = socket(AF_INET, SOCK_DGRAM, 0);

    std::vector<unsigned char>	_Buffer = {};
    static const size_t MAX_BUFFER_LEN = 1024; // 假设最大缓冲区长度为 1024 字节
    bool isJointUdp = false; //是否开启UDP拼包模式，默认式不开启

    ros::Publisher pub;
    ros::Timer timer;

    int port;
    std::string ip_addr;
};

#endif // Imu_H
