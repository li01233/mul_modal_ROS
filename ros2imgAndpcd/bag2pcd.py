import os
import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
 
class ExtractBagData(object):
 
    def __init__(self, bagfile_path, camera_topic, pointcloud_topic, root):
        self.bagfile_path = bagfile_path
        self.camera_topic = camera_topic
        self.pointcloud_topic = pointcloud_topic
        self.root = root
        self.image_dir = os.path.join(root, "images")
        self.pointcloud_dir = os.path.join(root, "pointcloud")
 
        #创建提取图片和点云的目录 ./root/images  root/pointcloud
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        if not os.path.exists(self.pointcloud_dir):
            os.makedirs(self.pointcloud_dir)
 
    def extract_camera_topic(self):
        bag = rosbag.Bag(self.bagfile_path, "r")
        bridge = CvBridge()
        for i in range(len(self.camera_topic)):
            bag_data_imgs = bag.read_messages(self.camera_topic[i])

            image_dir = os.path.join(self.image_dir, "camera_{}".format(i+1))
            if not os.path.exists(image_dir):
                os.makedirs(image_dir)

            index = 0

            pbar = tqdm(bag_data_imgs)
            for topic, msg, t in pbar: # type: ignore
                pbar.set_description("Processing extract image id: %s" % (index+1))
                cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                # 如果你需要使用时间戳对提取的图片命名，可以使用msg.header.stamp.to_sec()获取时间戳
                timestr = "%.6f" %  msg.header.stamp.to_sec()
                cv2.imwrite(os.path.join(image_dir, str(timestr) + ".jpg"), cv_image) # type: ignore
                index += 1
 
    def extract_pointcloud_topic(self):
        '''
        # 提取点云数据为pcd后缀文件,默认提取以为时间戳命名
        # 提取命令:rosrun pcl_ros bag_to_pcd result.bag /velodyne_points ./pointcloud
        # 提取点云以时间戳命令:1616554905.476288682.pcd
        :return:
        '''
        cmd = "rosrun pcl_ros bag_to_pcd %s /pointcloud %s" % (self.bagfile_path, self.pointcloud_dir)
        os.system(cmd)
 
        # 再读取提取的pcd点云数据，把文件名修改为按照顺序索引名
        #pcd_files_list = os.listdir(self.pointcloud_dir)
        # 因为提取的pcd是以时间戳命令的，但是存放到列表中并不是按照时间戳从小到大排列，这里对时间戳进行重新排序
        #pcd_files_list_sorted = sorted(pcd_files_list)
        # print(zip(pcd_files_list, pcd_files_list_sorted))
 
        #index = 0
        #pbar = tqdm(pcd_files_list_sorted)
        #for pcd_file in pbar:
        #    pbar.set_description("Processing extract poindcloud id: %s" % (index + 1))
        #    os.rename(os.path.join(self.pointcloud_dir, pcd_file),
        #              os.path.join(self.pointcloud_dir, str(index) + ".pcd"))
        #    print("pcd_file name: ", pcd_file)
        #    index += 5
 
if __name__ == '__main__':
    #需要处理的bag包的路径
    bagfile_path = "2024-12-20-15-08-01.bag"
    camera_topic = ["/compressedimg1","/compressedimg2","/compressedimg3","/compressedimg4"]
    pointcloud_topic = "/pointcloud"
    #将bag转换为pcd和jpg格式的保存路径
    extract_bag = ExtractBagData(bagfile_path, camera_topic, pointcloud_topic,  "./data")
    extract_bag.extract_camera_topic()
    extract_bag.extract_pointcloud_topic()
    
