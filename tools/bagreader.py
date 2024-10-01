import rosbag
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from tqdm import tqdm
import yaml
import open3d as o3d
import logging
import sys
import os
from datetime import datetime

from os import path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from config.config import CONF
from mayavi import mlab

class BAGFileProcessor:
    def __init__(self):
        """
        Initialize BAGFileProcessor Class and Read .bag file

            Topics            Types                     Message Count   Frequency
        0   /imu/data         sensor_msgs/Imu           666450          992.063493
        1   /velodyne_points  sensor_msgs/PointCloud2   13330           19.829467

        """
        # Setup logging
        log_dir = CONF.PATH.LOG
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = os.path.join(log_dir, f'dsec_pcd_2_bin_processing_{current_time}.log')
        self.logger = self.setup_logger(log_file)


        self.topic = '/velodyne_points'
        self.data_path = CONF.PATH.DATA
        self.scene_sequence = CONF.SCENE.SEQUENCE

        self.logger.info(f'Start processing DSEC point cloud datasets')
        self.logger.info(f'Topic name: {self.topic}')
        self.logger.info(f'Data path: {self.data_path}')

        self.filepath = []
        self.get_filepath()

        self.get_bin()

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
            filepath = os.path.join(self.data_path, seq, 'lidar_imu.bag')
            self.filepath.append(filepath)

            self.logger.info(f'Added file path: {filepath}')


    def get_iter(self, bag_file_path):
        bag_data = rosbag.Bag(bag_file_path, "r")
        iter_points_data = bag_data.read_messages(topics=self.topic)
        return iter_points_data

    def get_transfer_matrix(self, yaml_filepath):
        with open(yaml_filepath, 'r') as file:
            config = yaml.safe_load(file)
        self.logger.info(f'Loaded transformation matrix from: {yaml_filepath}')
        T = np.array(config['T_lidar_camRect1'])
        return T

    def get_bin(self):
        for filepath in tqdm(self.filepath, desc="Processing files"):
            try:
                self.logger.info(f'Start processing file: {filepath}')

                # create iterator for each scene
                points_iter = self.get_iter(filepath)  

                # create output dir
                output_filedir = os.path.join(os.path.dirname(filepath), 'velodyne')
                if not os.path.exists(output_filedir):
                    os.makedirs(output_filedir)
                    self.logger.info(f'Created directory: {output_filedir}')
                else:
                    self.logger.info(f'Directory already exists: {output_filedir}')

                # get T
                yaml_filepath = os.path.join(os.path.dirname(filepath), 'cam_to_lidar.yaml')
                T = self.get_transfer_matrix(yaml_filepath)

                '''
                while True:
                    try:
                        topic, msg, timestamp = next(points_iter)  # read one frame each time
                '''     

                for topic, msg, timestamp in tqdm(points_iter, desc="Processing frames", unit="frame"):
                    try:
                        # x y z intensity ring time
                        point_list = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))

                        points = np.array(point_list, dtype=np.float32)

                        # transfer points to homogeneous (x,y,z) --> (x,y,z,1)
                        points_homogeneous = np.hstack((points[:, :3], np.ones((points.shape[0], 1))))
                        
                        # apply T
                        points_transformed = np.dot(points_homogeneous, T.T)

                        # transfer homogeneous to points (x,y,z,1) --> (x,y,z) 
                        points_transformed = points_transformed[:, :3]

                        # add intensity
                        points_transformed = np.hstack((points_transformed, points[:, 3:4]))

                        output_filepath = os.path.join(output_filedir, (str(timestamp.to_nsec() // 1000) + '.bin'))
                        points.tofile(output_filepath)

                    except StopIteration: 
                        # stop when facing the iterator
                        self.logger.info(f'Finished processing file: {filepath}')
                        break
            except Exception as e:
                    self.logger.error(f'Error processing file {filepath}: {str(e)}')


class Visualizer:
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

    def viz_mayavi(self, bin_filepath):
        # load .bin
        points = np.fromfile(bin_filepath, dtype=np.float32).reshape(-1, 4)

        x = points[:, 0]  # x position of point
        y = points[:, 1]  # y position of point
        z = points[:, 2]  # z position of point
        fig = mlab.figure(bgcolor=(0, 0, 0), size=(640, 360))
        mlab.points3d(x, y, z,
                    z,  # Values used for Color
                    mode="point",
                    colormap='spectral',  # 'bone', 'copper', 'gnuplot'
                    # color=(0, 1, 0),   # Used a fixed (r,g,b) instead
                    figure=fig,
                    )
        mlab.savefig("mayavi_visualization.png")  # Save the visualization as an image
        mlab.close(fig)  # Close the Mayavi figure

if __name__ == "__main__":
    BFP = BAGFileProcessor()
