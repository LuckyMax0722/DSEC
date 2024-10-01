import numpy as np
import open3d as o3d
import sys
import os

from os import path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from config.config import CONF
from mayavi import mlab

class Visualizer1:
    def __init__(self, bin_filepath):

        if os.path.isdir(bin_filepath):
            bin_files = [os.path.join(bin_filepath, f) for f in sorted(os.listdir(bin_filepath)) if f.endswith('.bin')]
            for bin_file in bin_files:
                self.visualize_blf_file(bin_file)
        elif os.path.isfile(bin_filepath) and bin_filepath.endswith('.bin'):
            self.visualize_blf_file(bin_filepath)
        else:
            print("Invaid input, please input path/to/bin/dir or path/to/bin")


    def get_size(self, points):
        points = np.asarray(points)

        # compute x,y,z range
        x_range = np.ptp(points[:, 0])
        y_range = np.ptp(points[:, 1])
        z_range = np.ptp(points[:, 2])

        print(f"x_range: {x_range}")
        print(f"y_range: {y_range}")
        print(f"z_range: {z_range}")

    def visualize_blf_file(self, bin_filepath):
        # load .bin
        points = np.fromfile(bin_filepath, dtype=np.float32).reshape(-1, 4)

        pcd = o3d.geometry.PointCloud()

        # get x,y,z points
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])

        self.get_size(pcd.points)

        # vis
        o3d.visualization.draw_geometries([pcd])

class Visualizer:
    def __init__(self, pcd_bin_filepath, voxel_bin_filepath):

        self.pcd = np.fromfile(pcd_bin_filepath, dtype=np.float32).reshape(-1, 4)

        self.voxel = np.fromfile(voxel_bin_filepath, dtype=np.uint8)
        self.voxel = self.unpack(self.voxel)
        self.voxel = self.voxel.reshape((256, 256, 32))

        self.vis_mayavi()
        self.vis_o3d()

    def unpack(self, compressed):
        ''' given a bit encoded voxel grid, make a normal voxel grid out of it.  '''
        uncompressed = np.zeros(compressed.shape[0] * 8, dtype=np.uint8)
        uncompressed[::8] = compressed[:] >> 7 & 1
        uncompressed[1::8] = compressed[:] >> 6 & 1
        uncompressed[2::8] = compressed[:] >> 5 & 1
        uncompressed[3::8] = compressed[:] >> 4 & 1
        uncompressed[4::8] = compressed[:] >> 3 & 1
        uncompressed[5::8] = compressed[:] >> 2 & 1
        uncompressed[6::8] = compressed[:] >> 1 & 1
        uncompressed[7::8] = compressed[:] & 1

        return uncompressed

    def vis_mayavi(self):
        mlab.contour3d(self.voxel, contours=[0.5], opacity=0.5)

        # 计算中心坐标
        center = [0, 128, 0]

        # 在中心处放置一个点
        mlab.points3d(center[0], center[1], center[2], color=(1, 0, 0), scale_factor=5)

        # 绘制坐标轴
        axes_length = 30  # 坐标轴的长度
        mlab.quiver3d(center[0], center[1], center[2], 
                      axes_length, 0, 0, color=(1, 0, 0), scale_factor=1)  # x轴
        mlab.quiver3d(center[0], center[1], center[2], 
                      0, axes_length, 0, color=(0, 1, 0), scale_factor=1)  # y轴
        mlab.quiver3d(center[0], center[1], center[2], 
                      0, 0, axes_length, color=(0, 0, 1), scale_factor=1)  # z轴
                      
        mlab.orientation_axes()

        mlab.show()

    def vis_o3d(self):
        x = self.pcd[:, 0]
        y = self.pcd[:, 1]
        z = self.pcd[:, 2]

        mlab.points3d(x, y, z, mode="point", colormap='spectral', scale_factor=1)
        
        # 计算中心坐标
        center = [0, 0, 0]

        # 在中心处放置一个点
        mlab.points3d(center[0], center[1], center[2], color=(1, 0, 0), scale_factor=1)

        # 绘制坐标轴
        axes_length = 30  # 坐标轴的长度
        mlab.quiver3d(center[0], center[1], center[2], 
                      axes_length, 0, 0, color=(1, 0, 0), scale_factor=1)  # x轴
        mlab.quiver3d(center[0], center[1], center[2], 
                      0, axes_length, 0, color=(0, 1, 0), scale_factor=1)  # y轴
        mlab.quiver3d(center[0], center[1], center[2], 
                      0, 0, axes_length, color=(0, 0, 1), scale_factor=1)  # z轴
                      
        mlab.orientation_axes()

        mlab.show()


        '''
        pcd = o3d.geometry.PointCloud()

        # get x,y,z points
        pcd.points = o3d.utility.Vector3dVector(self.pcd[:, :3])

        # vis
        o3d.visualization.draw_geometries([pcd])
        '''


if __name__ == "__main__":
    #VIS = Visualizer('/media/max/GAME/MA/datasets/semanticKITTI/000000.bin')
    #VIS.vis_bin()

    VIS = Visualizer(
        pcd_bin_filepath = '/media/max/GAME/MA/datasets/DSEC/data/interlaken_00/velodyne/51530562321.bin',
        
        #voxel_bin_filepath = '/media/max/GAME/MA/datasets/SemanticKITTI/dataset/sequences/00/voxels/000000.bin'
        voxel_bin_filepath = '/media/max/GAME/MA/voxelizer/bin/000000.bin'
    )
