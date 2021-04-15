"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Online Quadric SLAM system
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
import numpy as np
import time
import cv2
import atexit
import yaml
import argparse

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

# import custom python modules
sys.dont_write_bytecode = True
from base.data_association import DataAssociation
from base.data_association import KeepAssociator
from base.data_association import MergeAssociator
from visualization.drawing import CV2Drawing
from base.containers import Trajectory, Quadrics, Detections, ObjectDetection
from base.initialisation import Initialize

from dataset_interfaces.scenenet_dataset import SceneNetDataset
from dataset_interfaces.tumrgbd_dataset import TUMDataset
from detectors.faster_rcnn import FasterRCNN

# import gtsam and extension
import gtsam
import gtsam_quadrics


import pyrealsense2 as rs
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def rs_intrinsics_to_matrix(intrinsics):
    matrix = np.zeros((3,3))
    matrix[0,0] = intrinsics.fx
    matrix[1,1] = intrinsics.fy
    matrix[0,2] = intrinsics.ppx
    matrix[1,2] = intrinsics.ppy
    matrix[2,2] = 1.0
    return matrix

    



if __name__ == '__main__': 


    np.set_printoptions(precision=4)

    pipeline = rs.pipeline()
    config = rs.config()

    r1 = config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    r2 = config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline_profile = pipeline.start(config)
    color_stream_profile = pipeline_profile.get_stream(rs.stream.color)
    intrinsics = color_stream_profile.as_video_stream_profile().get_intrinsics()

    device = pipeline_profile.get_device()
    depth_sensor = device.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    
    P = rs_intrinsics_to_matrix(intrinsics)

    odometry = cv2.rgbd.RgbdOdometry_create(P)

    n = 0
    try:

        fig = plt.figure()
        ax = fig.gca(projection='3d')


        poses = [gtsam.Pose3()]
        prev_img = None
        prev_depth = None
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # convert image to np arrays
            depth = np.asanyarray(depth_frame.get_data())
            color = np.asanyarray(color_frame.get_data())

            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            depth_meters = np.array(depth*depth_scale, np.float32)

            # calculate odometry
            if n > 50:
                mask = np.ones(color.shape[0:2], np.uint8)
                transform = np.ones((4,4))
                odometry.compute(prev_img, prev_depth, mask, gray, depth_meters, mask, transform)

                odom = gtsam.Pose3(transform).inverse()
                poses.append(poses[-1].compose(odom))
                print(odom.translation().vector())
                
                xs = [p.x() for p in poses]
                ys = [p.y() for p in poses]
                zs = [p.z() for p in poses]

                ax.clear()
                ax.plot(xs,ys,zs)
                plt.draw()
                plt.pause(0.001)


            # draw rgb+depth 
            depth = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color, depth))
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.waitKey(1)

            prev_img = gray
            prev_depth = depth_meters
            n += 1
    except KeyboardInterrupt:
        print('\nstopping')
        pipeline.stop()
    