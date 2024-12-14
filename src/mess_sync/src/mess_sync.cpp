#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <ros/ros.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher Img_info_pub1, Img_info_pub2, Img_info_pub3, Img_info_pub4;
ros::Publisher PC_info_pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& ori_pointcloud, const sensor_msgs::CompressedImage::ConstPtr& ori_img1,
              const sensor_msgs::CompressedImage::ConstPtr& ori_img2, const sensor_msgs::CompressedImage::ConstPtr& ori_img3,
              const sensor_msgs::CompressedImage::ConstPtr& ori_img4)
{
  // 指针转为msg
  sensor_msgs::PointCloud2 syn_pointcloud = *ori_pointcloud;
  sensor_msgs::CompressedImage syn_img1 = *ori_img1;
  sensor_msgs::CompressedImage syn_img2 = *ori_img2;
  sensor_msgs::CompressedImage syn_img3 = *ori_img3;
  sensor_msgs::CompressedImage syn_img4 = *ori_img4;
  // 打印结果
  cout << "syn velodyne points‘ timestamp : " << syn_pointcloud.header.stamp << endl;
  cout << "syn Img1‘s timestamp : " << syn_img1.header.stamp << endl;
  cout << "syn Img2‘s timestamp : " << syn_img2.header.stamp << endl;
  cout << "syn Img3‘s timestamp : " << syn_img3.header.stamp << endl;
  cout << "syn Img4‘s timestamp : " << syn_img4.header.stamp << endl;

  // 发送话题
  Img_info_pub1.publish(syn_img1);
  Img_info_pub2.publish(syn_img2);
  Img_info_pub3.publish(syn_img3);
  Img_info_pub4.publish(syn_img4);


  // 去除NAN
  pcl::PointCloud<pcl::PointXYZI>* cloud = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::fromROSMsg(*ori_pointcloud, *cloud);

  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

  sensor_msgs::PointCloud2 out_pointcloud;
  pcl::toROSMsg(*cloud, out_pointcloud);

  PC_info_pub.publish(out_pointcloud);
}


int main(int argc, char** argv)
{
  // 初始化节点
  ros::init(argc, argv, "msg_synchronizer");
  ros::NodeHandle nh;
  // 初始化发布话题
  Img_info_pub1 = nh.advertise<sensor_msgs::CompressedImage>("/compressedimg1",10);
  Img_info_pub2 = nh.advertise<sensor_msgs::CompressedImage>("/compressedimg2",10);
  Img_info_pub3 = nh.advertise<sensor_msgs::CompressedImage>("/compressedimg3",10);
  Img_info_pub4 = nh.advertise<sensor_msgs::CompressedImage>("/compressedimg4",10);
  PC_info_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud",10);
  // 初始化消息订阅者
  message_filters::Subscriber<CompressedImage> camera_sub1(nh, "/hik_cam_node_1/hik_camera/compressed", 1);
  message_filters::Subscriber<CompressedImage> camera_sub2(nh, "/hik_cam_node_2/hik_camera/compressed", 1);
  message_filters::Subscriber<CompressedImage> camera_sub3(nh, "/hik_cam_node_3/hik_camera/compressed", 1);
  message_filters::Subscriber<CompressedImage> camera_sub4(nh, "/hik_cam_node_4/hik_camera/compressed", 1);
  message_filters::Subscriber<PointCloud2> velodyne_sub(nh, "/rslidar_points", 1);
  // 初始化同步消息规则
  typedef sync_policies::ApproximateTime<PointCloud2, CompressedImage, CompressedImage, CompressedImage, CompressedImage> MySyncPolicy;
  // 消息同步与回调   
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), velodyne_sub, camera_sub1, camera_sub2, camera_sub3, camera_sub4); //queue size=10
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

  ros::spin();
  return 0;
}

