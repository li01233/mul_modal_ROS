# 2024.12.20 更新 Quantum2雷达和一个未实现的内容

## Quantum2雷达

感谢<url>https://github.com/Aquatic-Drone-Testbed/ROS-USV-Packages</url>提供的Quantum雷达数据解析源代码，我将其改为了ROS1版本，使其集成到我们整个系统中

输出在话题/quantum_spoke中，是一个250*256的矩阵（250为辐条个数，256为每个辐条的长度），采集时依旧使用

roslaunch start_collect start.launch\
rosbag record /pointcloud /compressedimg1  /compressedimg2  /compressedimg3  /compressedimg4 /quantum_spoke

关于一些uantum2雷达集成ROS的细节（和吐槽）我都写进了新博客，欢迎大家参考

## 一个未实现的内容

这部分主要是想实现如何将雷达数据和相机数据联合标注转换后，点云映射到图像上的效果，目前参考了<url>https://github.com/jhzhang19/ros_project_pc_to_image</url>的包，但还没有实现出来，等实现了我会提交2.2版本的

# 说明
本程序用于海康威视监控相机+RoboSense128线激光雷达同步数据采集，本程序已经做好封装，时间同步和去NaN值\
更多请看<url>https://blog.csdn.net/u011549111/article/details/144158995</url>

# 编译
## 检查相机配置
在src/hikvision/launch/hik.launch中更改ip等如下\
(具体配置联系管理员获取，这里仅参考)\
\<arg name="ip_addr" default="192.168.xxx.xxx"/>\
\<arg name="user_name" default="admin"/>\
\<arg name="password" default="***********"/>

## 检查雷达配置
在src/rslidar_sdk/config/config.yaml中更改雷达型号

## 在本文件所在路径下make
catkin_make

使用conda的话先激活环境安装empy，再catkin_make
pip install empy==3.3.4
catkin_make -DPYTHON_EXECUTABLE=/home/li012/miniconda3/envs/ros/bin/python3 # 这里改成你的环境路径

# 运行与录制
## 运行ros
roscore

## 启动新的一个终端，用于启动传感器
source devel/setup.bash\
roslaunch start_collect start.launch\
##在弹出的rviz中可以查看同时的图像和点云数据\
##使用rostopic list可以查看发布了的话题

## 启动一个新终端，使用rosbag记录数据
rosbag record /pointcloud /compressedimg1  /compressedimg2  /compressedimg3  /compressedimg4
##使用rosbag info xxx.bag 可以查看两个传感器采样频率是否对齐

# 回放
roscore\
source devel/setup.bash\
roslaunch start_collect play.launch bag_path:="/home/li012/robosense_ws/data/1208lidar/3.bag" # 必须为绝对路径

# 解码为jpg和pcd

## 使用ros2imgAndpcd目录下的py文件完成image和点云数据的解码
conda activate ros ## 激活pyhton环境\
修改bagfile_path = './xxx.bag'和保存路径


## 附：抓取pcap
sudo tcpdump -i 网卡名 -w a.pcap -c 30000 'udp dst port 7788 or 6699' # 网卡名用ifconfig查看
