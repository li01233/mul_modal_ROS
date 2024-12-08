import os
import numpy as np
import pcl

def convert_csv_to_pcd(csv_file, pcd_file):
    # 读取CSV文件，假设每行包含x, y, z和intensity
    data = np.loadtxt(csv_file, delimiter=',',skiprows=1)  # 读取CSV文件

    # 创建一个包含4个字段 (x, y, z, intensity) 的点云
    cloud = pcl.create_xyzi(data[:,:4])

    # 将点云保存为PCD文件
    pcl.save_pcd(pcd_file, cloud)
    print(f"已保存：{pcd_file}")

def batch_convert_csv_to_pcd(input_dir, output_dir):
    # 确保输出目录存在
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 获取所有CSV文件
    csv_files = [f for f in os.listdir(input_dir) if f.endswith('.csv')]

    # 批量转换
    for csv_file in csv_files:
        input_file = os.path.join(input_dir, csv_file)
        output_file = os.path.join(output_dir, os.path.splitext(csv_file)[0] + '.pcd')
        convert_csv_to_pcd(input_file, output_file)

# 示例用法
input_directory = './'  # 输入CSV文件夹路径
output_directory = 'path/to/output/pcd'  # 输出PCD文件夹路径
batch_convert_csv_to_pcd(input_directory, output_directory)
