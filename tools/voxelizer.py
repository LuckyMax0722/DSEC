import numpy as np
from tqdm import tqdm
import open3d as o3d
import logging
import sys
import os
from datetime import datetime
from glob import glob

from os import path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from config.config import CONF


class Voxelizer:
    def __init__(self):
        # Setup logging
        log_dir = CONF.PATH.LOG
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = os.path.join(log_dir, f'dsec_bin_2_voxel_processing_{current_time}.log')

        self.logger = self.setup_logger(log_file)

        self.data_path = CONF.PATH.DATA
        self.scene_sequence = CONF.SCENE.SEQUENCE

        self.logger.info(f'Start processing DSEC point cloud to voxel')
        self.logger.info(f'Data path: {self.data_path}')


        self.filepaths = []
        self.get_filepath()

        '''
        # For DSEC dataset, globl bbox is [244, 244, 64]
        # For SemanticKITTI dataset, voxel is [256, 256, 32]
        
        self.global_min_bound = np.array([float('inf'), float('inf'), float('inf')])
        self.global_max_bound = np.array([float('-inf'), float('-inf'), float('-inf')])

        self.global_min_bound, self.global_max_bound = self.compute_global_bounding_box()
        '''

        self.x_range = [0, 51.2]
        self.y_range = [-25.6, 25.6]
        self.z_range = [-2.0, 4.4]
        self.voxel_size = 0.2

        self.voxelize_and_save(self.filepaths[0])

    def setup_logger(self, log_file):
        """Set up the logger to write logs to both the console and a file."""
        logger = logging.getLogger(__name__)
        logger.setLevel(logging.DEBUG)

        # File handler for logging to a file
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)

        # Console handler for logging to the console
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)

        # Formatter for log messages
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)

        # Add handlers to the logger
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

        return logger
    
    def get_filepath(self):
        for seq in self.scene_sequence:
            self.filepaths += glob(os.path.join(self.data_path, seq, 'velodyne', '*.bin'))

    def compute_global_bounding_box(self):
        for file in tqdm(self.filepaths, desc="Computing global bbox"):
            points = self.read_bin_file(file)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            bounding_box = pcd.get_axis_aligned_bounding_box()
            self.global_min_bound = np.minimum(self.global_min_bound, bounding_box.min_bound)
            self.global_max_bound = np.maximum(self.global_max_bound, bounding_box.max_bound)
        
        return self.global_min_bound, self.global_max_bound


    def read_bin_file(self, bin_file_path):
        point_cloud = np.fromfile(bin_file_path, dtype=np.float32).reshape(-1, 4)
        return point_cloud[:, :3]
    

    def filter_points_in_range(self, points):
        mask = (
            (points[:, 0] >= self.x_range[0]) & (points[:, 0] <= self.x_range[1]) &
            (points[:, 1] >= self.y_range[0]) & (points[:, 1] <= self.y_range[1]) &
            (points[:, 2] >= self.z_range[0]) & (points[:, 2] <= self.z_range[1])
        )
        return points[mask]


    def voxelize_and_save(self, point_cloud_file, output_folder=None):
        points = self.read_bin_file(point_cloud_file)
        points = self.filter_points_in_range(points)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=self.voxel_size)
        
        # get voxel center
        voxels = voxel_grid.get_voxels()
        print(len(voxels))
        voxel_centers = np.array([voxel_grid.get_voxel_center_coordinate(voxel.grid_index) for voxel in voxels])
    
        # 保存体素数据为 .bin 文件
        #output_file_path = os.path.join(output_folder, os.path.basename(point_cloud_file))
        voxel_centers.astype(np.float32).tofile('/media/max/GAME/MA/test.bin')
        
        return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # 平移点云，使其与全局坐标对齐
        pcd.translate(-global_min_bound)
        
        # 生成体素网格
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
        
        # 提取体素中心点
        voxel_centers = np.array([voxel.grid_index for voxel in voxel_grid.get_voxels()])
        
        # 保存体素数据为 .bin 文件
        output_file_path = os.path.join(output_folder, os.path.basename(point_cloud_file))
        voxel_centers.astype(np.float32).tofile(output_file_path)

if __name__ == "__main__":
    VOX = Voxelizer()