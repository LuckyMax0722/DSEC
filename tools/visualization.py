import cv2
import matplotlib as mpl
import matplotlib.cm as cm
import numpy as np

from tqdm import tqdm

import sys
from os import path

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from utils.eventreader import EventReader
import open3d as o3d
import hdf5plugin

def show_image(image):
    cv2.namedWindow('viz', cv2.WINDOW_NORMAL)
    cv2.imshow('viz', image)
    cv2.waitKey(0)

def render(x: np.ndarray, y: np.ndarray, pol: np.ndarray, H: int, W: int) -> np.ndarray:
    assert x.size == y.size == pol.size
    assert H > 0
    assert W > 0
    img = np.full((H,W,3), fill_value=255,dtype='uint8')
    mask = np.zeros((H,W),dtype='int32')
    pol = pol.astype('int')
    pol[pol==0]=-1
    mask1 = (x>=0)&(y>=0)&(W>x)&(H>y)
    mask[y[mask1],x[mask1]]=pol[mask1]
    img[mask==0]=[255,255,255]
    img[mask==-1]=[255,0,0]
    img[mask==1]=[0,0,255]
    return img

def event_visualize(event_filepath, dt=50.0):
    height=480
    width=640

    for events in tqdm(EventReader(event_filepath, dt)):
        p = events['p']
        x = events['x']
        y = events['y']
        t = events['t']

        print(p,t,x,y)
        img = render(x, y, p, height, width)
        show_image(img)
        break

def pcd_visualize(pcd_filepath):
    # load pcd
    pcd = o3d.io.read_point_cloud(pcd_filepath)

    # visualize pcd
    o3d.visualization.draw_geometries([pcd])

    
if __name__ == "__main__":
    event_visualize("/media/max/GAME/MA/datasets/DSEC/data/interlaken_00_a/events/left/events.h5")