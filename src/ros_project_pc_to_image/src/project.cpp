#include <ros/ros.h>
#include "ros/package.h"
#include <iostream>
#include <numeric>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <termios.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <Eigen/Dense>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher project_img_pub1, project_img_pub2, project_img_pub3, project_img_pub4;
float maxX = 100.0, maxY = 100.0, minZ = -2.0;

struct initial_parameters
{
    /* data */
    std::string camera_topic;
    std::string lidar_topic;
    cv::Mat cameraIn;
    cv::Mat RT;
    cv::Mat camera_distort;
}i_params1, i_params2, i_params3, i_params4;
std::vector<initial_parameters> i_params = {i_params1, i_params2, i_params3, i_params4};


void initParams(initial_parameters* i_params, int idx)
{
    std::string pkg_loc = ros::package::getPath("ros_project_pc_to_image");
    std::ifstream infile(pkg_loc + "/cfg/camera_"+std::to_string(idx+1)+".txt");
    infile >> i_params->camera_topic;
    infile >> i_params->lidar_topic;
    double_t cameraIn[12];
    double_t RT[16];
    double_t camera_distort[4];

    for (int i = 0; i < 12; i++){
        infile >> cameraIn[i];
    }
    cv::Mat(3, 4, 6, &cameraIn).copyTo(i_params->cameraIn);//cameraIn params

    for (int i = 0; i < 16; i++){
        infile >> RT[i];
    }
    cv::Mat(4, 4, 6, &RT).copyTo(i_params->RT);//lidar to camera params

    for (int i = 0; i < 5; i++){
        infile >> camera_distort[i];
    }
    cv::Mat(4, 1, 6, &camera_distort).copyTo(i_params->camera_distort);//lidar to camera params
}

// 函数：将正常图像中的点映射到畸变图像中
cv::Point2f distortPoint(const cv::Point2f& point, const cv::Mat& cameraMatrix, float k1, float k2) 
{
    float x = point.x;
    float y = point.y;

    // 将点的坐标转换为归一化坐标
    float x_norm = (x - cameraMatrix.at<double>(0, 2)) / cameraMatrix.at<double>(0, 0);
    float y_norm = (y - cameraMatrix.at<double>(1, 2)) / cameraMatrix.at<double>(1, 1);

    // 计算径向畸变
    float r2 = x_norm * x_norm + y_norm * y_norm;
    float x_distorted = x_norm * (1 + k1 * r2 + k2 * r2 * r2);
    float y_distorted = y_norm * (1 + k1 * r2 + k2 * r2 * r2);

    // 将畸变后的归一化坐标转换回图像坐标
    float x_distorted_img = x_distorted * cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(0, 2);
    float y_distorted_img = y_distorted * cameraMatrix.at<double>(1, 1) + cameraMatrix.at<double>(1, 2);

    return cv::Point2f(x_distorted_img, y_distorted_img);
}

sensor_msgs::Image projection(const sensor_msgs::CompressedImage::ConstPtr& img, 
                         const sensor_msgs::PointCloud2::ConstPtr& pc, initial_parameters i_params)
{   
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        sensor_msgs::Image msg;
        return msg;
    }
    cv::Mat raw_img = cv_ptr->image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pc, *cloud);
    
    cv::Mat visImg = raw_img.clone();
    cv::Mat overlay = visImg.clone();
    
    cv::Mat X(4,1,cv::DataType<double>::type);
    cv::Mat Y(3,1,cv::DataType<double>::type);
    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++){
        if (isnan(it->x) || isnan(it->y) || isnan(it->z))
        {
            continue;
        }
        if(abs(it->x) > maxX  || abs(it->y) > maxY || it->z < minZ ) 
        {
            continue;
        }
        
        X.at<double>(0,0) = it->x;
        X.at<double>(1,0) = it->y;
        X.at<double>(2,0) = it->z;
        X.at<double>(3,0) = 1;

        Y = i_params.cameraIn * i_params.RT * X;  //tranform the point to the camera coordinate

        cv::Point2f pt;
        pt.x = Y.at<double>(0,0) / Y.at<double>(2,0);
        pt.y = Y.at<double>(1,0) / Y.at<double>(2,0);
        
        // Distort the point
        cv::Mat K = i_params.cameraIn(cv::Range::all(), cv::Range(0,3));

        pt = distortPoint(pt, K, i_params.camera_distort.at<double>(0,0), i_params.camera_distort.at<double>(1,0));

        float val = it->intensity;
        float maxVal = 128.0;
        int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
        int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
        cv::circle(overlay, pt, 1, cv::Scalar(0, green, red), -1);
        
    }


    // Publish the image projection
    ros::Time time = ros::Time::now();
    cv_ptr->encoding = "bgr8";
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "/camera_main";
    cv_ptr->image = overlay;
    return *(cv_ptr->toImageMsg());
    // image_publisher.publish(cv_ptr->toImageMsg());
    // project_img_pub.publish(cv_ptr->toImageMsg());
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& ori_pointcloud, const sensor_msgs::CompressedImage::ConstPtr& ori_img1,
              const sensor_msgs::CompressedImage::ConstPtr& ori_img2, const sensor_msgs::CompressedImage::ConstPtr& ori_img3,
              const sensor_msgs::CompressedImage::ConstPtr& ori_img4)
{
    std::cout<<"get pc and image data"<<std::endl;
    sensor_msgs::Image img_msg1 = projection(ori_img1, ori_pointcloud, i_params[0]);
    sensor_msgs::Image img_msg2 = projection(ori_img2, ori_pointcloud, i_params[1]);
    sensor_msgs::Image img_msg3 = projection(ori_img3, ori_pointcloud, i_params[2]);
    sensor_msgs::Image img_msg4 = projection(ori_img4, ori_pointcloud, i_params[3]);

    project_img_pub1.publish(img_msg1);
    project_img_pub2.publish(img_msg2);
    project_img_pub3.publish(img_msg3);
    project_img_pub4.publish(img_msg4);
    std::cout<<"project picture is published!"<<std::endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "project_pc_to_image");
    
    ros::NodeHandle nh;
    for (int i = 0; i < 4; i++){
        initParams(&(i_params[i]), i);
        std::cout<<i<<":"<<std::endl;
        std::cout<<i_params[i].RT<<std::endl;
        std::cout<<i_params[i].cameraIn<<std::endl;
        std::cout<<i_params[i].camera_distort<<std::endl;
    }
    
    project_img_pub1 = nh.advertise<Image>("/project_pc_image1",10);
    project_img_pub2 = nh.advertise<Image>("/project_pc_image2",10);
    project_img_pub3 = nh.advertise<Image>("/project_pc_image3",10);
    project_img_pub4 = nh.advertise<Image>("/project_pc_image4",10);

    // ros::Publisher project_img_pub;
    message_filters::Subscriber<CompressedImage> image_sub1(nh, "/compressedimg2", 10);
    message_filters::Subscriber<CompressedImage> image_sub2(nh, "/compressedimg2", 10);
    message_filters::Subscriber<CompressedImage> image_sub3(nh, "/compressedimg2", 10);
    message_filters::Subscriber<CompressedImage> image_sub4(nh, "/compressedimg2", 10);
    message_filters::Subscriber<PointCloud2> pcl_sub(nh, "/pointcloud", 10);
    
    typedef sync_policies::ApproximateTime<PointCloud2, CompressedImage, CompressedImage, CompressedImage, CompressedImage> MySyncPolicy;
    // 消息同步与回调   
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), pcl_sub, image_sub1, image_sub2, image_sub3, image_sub4); //queue size=10
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
    // image_transport::ImageTransport imageTransport(nh);
    // image_publisher = imageTransport.advertise("/project_pc_image", 20);
    ros::spin();
    return 0;
}