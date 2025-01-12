#ifndef _HIK_CAMERA_H_
#define _HIK_CAMERA_H_

#include <stdio.h>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse.h>

#include "hik_sdk/HCNetSDK.h"
#include "hik_sdk/plaympeg4.h"
#include "hik_tem/PtzCtrl.h"
#include "hik_tem/P2PData.h"

using namespace std;

class HikCamera{

    public:
        ~HikCamera();
        void run();
    private:
        /// ros parameters
        ros::Publisher p2p_data_pub;
        ros::Subscriber ptz_ctrl;
        ros::Timer timer;
        
        /// camera parameters
        int image_width;
        int image_height;

        std::string ip_addr;
        std::string usr_name;
        std::string password;
        std::string frame_id;
        std::string camera_name;
        std::string camera_info_url;

        int port;
        int channel;
        int link_mode;

        LONG user_id;
        
        NET_DVR_JPEGPICTURE_WITH_APPENDDATA struJpegWithAppendAata = { 0 };

        void PtzCtrlCallback(const hik_tem::PtzCtrl::ConstPtr& msg);
        void P2PDataCollet(const ros::TimerEvent& e);

        bool initHikSDK();
        bool initThemCam();
        void initROSIO(ros::NodeHandle& priv_node);

        static int dec_to_hex(int param){
            int out = 0 , ind = 0;
            char buf[10] = {0};
            while(param){
                buf[ind] = param%10;
                ind++;
                param /= 10;
            }
            for(int i=ind-1;i>=0;i--){
                out = out *16 + buf[i];
            }
            return out;
        }

};



#endif // _HIK_CAMERA_H_
