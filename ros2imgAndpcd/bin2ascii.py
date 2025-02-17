import pcl
from rosbag.bag import Bag
from pcl import PointCloud
import os
from tqdm import tqdm

pcd_files = os.listdir("./pointcloud/")
pbar = tqdm(pcd_files)
for name in pbar:
    path = os.path.join("./pointcloud/", name)
    new_path = ath = os.path.join("./pointcloud_ascii/", name)
    pcd = pcl.load_pcd(path)
    # 默认就是binary=False, 所以pcd将存储成为ascii格式的
    pcl.save_pcd(new_path, pcd, binary=False)

