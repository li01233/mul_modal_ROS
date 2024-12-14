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
float maxX = 25.0, maxY = 6.0, minZ = -1.4;

struct initial_parameters
{
    /* data */
    std::string camera_topic;
    std::string lidar_topic;
    cv::Mat camtocam_mat;
    cv::Mat cameraIn;
    cv::Mat RT;
}i_params;


void initParams()
{
    std::string pkg_loc = ros::package::getPath("ros_project_pc_to_image");
    std::ifstream infile(pkg_loc + "/cfg/initial_params.txt");
    infile >> i_params.camera_topic;
    infile >> i_params.lidar_topic;
    double_t camtocam[12];
    double_t cameraIn[16];
    double_t RT[16];
    for (int i = 0; i < 16; i++){
        infile >> camtocam[i];
    }
    cv::Mat(4, 4, 6, &camtocam).copyTo(i_params.camtocam_mat);//cameratocamera params

    for (int i = 0; i < 12; i++){
        infile >> cameraIn[i];
    }
    cv::Mat(4, 4, 6, &cameraIn).copyTo(i_params.cameraIn);//cameraIn params

    for (int i = 0; i < 16; i++){
        infile >> RT[i];
    }
    cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT);//lidar to camera params
    std::cout<<i_params.RT<<std::endl;
}

sensor_msgs::Image projection(const sensor_msgs::CompressedImage::ConstPtr& img, 
                         const sensor_msgs::PointCloud2::ConstPtr& pc)
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
    std::cout<<"get pc and image data"<<std::endl;
    
    cv::Mat X(4,1,cv::DataType<double>::type);
    cv::Mat Y(3,1,cv::DataType<double>::type);
    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++){
        if(it->x > maxX || it->x < 0.0 || abs(it->y) > maxY || it->z < minZ ) 
        {
            continue;
        }
        
        X.at<double>(0,0) = it->x;
        X.at<double>(1,0) = it->y;
        X.at<double>(2,0) = it->z;
        X.at<double>(3,0) = 1;

        Y = i_params.cameraIn * i_params.camtocam_mat * i_params.RT * X;  //tranform the point to the camera coordinate

        cv::Point pt;
        pt.x = Y.at<double>(0,0) / Y.at<double>(0,2);
        pt.y = Y.at<double>(1,0) / Y.at<double>(0,2);

        float val = it->x;
        float maxVal = 20.0;
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
    std::cout<<"project picture is published!"<<std::endl;
    return *(cv_ptr->toImageMsg());
    // image_publisher.publish(cv_ptr->toImageMsg());
    // project_img_pub.publish(cv_ptr->toImageMsg());
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& ori_pointcloud, const sensor_msgs::CompressedImage::ConstPtr& ori_img1,
              const sensor_msgs::CompressedImage::ConstPtr& ori_img2, const sensor_msgs::CompressedImage::ConstPtr& ori_img3,
              const sensor_msgs::CompressedImage::ConstPtr& ori_img4)
{
    sensor_msgs::Image img_msg1 = projection(ori_img1, ori_pointcloud);
    sensor_msgs::Image img_msg2 = projection(ori_img2, ori_pointcloud);
    sensor_msgs::Image img_msg3 = projection(ori_img3, ori_pointcloud);
    sensor_msgs::Image img_msg4 = projection(ori_img4, ori_pointcloud);

    project_img_pub1.publish(img_msg1);
    project_img_pub2.publish(img_msg2);
    project_img_pub3.publish(img_msg3);
    project_img_pub4.publish(img_msg4);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "project_pc_to_image");
    
    ros::NodeHandle nh;
    initParams();

    project_img_pub1 = nh.advertise<Image>("/project_pc_image1",10);
    project_img_pub2 = nh.advertise<Image>("/project_pc_image2",10);
    project_img_pub3 = nh.advertise<Image>("/project_pc_image3",10);
    project_img_pub4 = nh.advertise<Image>("/project_pc_image4",10);

    // ros::Publisher project_img_pub;
    message_filters::Subscriber<CompressedImage> image_sub1(nh, "/compressedimg1", 1);
    message_filters::Subscriber<CompressedImage> image_sub2(nh, "/compressedimg1", 1);
    message_filters::Subscriber<CompressedImage> image_sub3(nh, "/compressedimg1", 1);
    message_filters::Subscriber<CompressedImage> image_sub4(nh, "/compressedimg1", 1);
    message_filters::Subscriber<PointCloud2> pcl_sub(nh, "/pointcloud", 1);
    
    typedef sync_policies::ApproximateTime<PointCloud2, CompressedImage, CompressedImage, CompressedImage, CompressedImage> MySyncPolicy;
    // 消息同步与回调   
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pcl_sub, image_sub1, image_sub2, image_sub3, image_sub4); //queue size=10
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
    // image_transport::ImageTransport imageTransport(nh);
    // image_publisher = imageTransport.advertise("/project_pc_image", 20);
    ros::spin();
    return 0;
}