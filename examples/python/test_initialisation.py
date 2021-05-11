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
import yaml
import argparse

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../quadricslam'))

# import custom python modules
sys.dont_write_bytecode = True
from visualization.drawing import CV2Drawing
# from base.containers import Trajectory, Quadrics, Detections, ObjectDetection
from detectors.faster_rcnn import FasterRCNN

# import gtsam and extension
import gtsam
import gtsam_quadrics

import pyrealsense2 as rs
import atexit




class RealsenseCamera(object):
    def __init__(self):
        # setup camera streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline_profile = self.pipeline.start(config)
        atexit.register(self.close_camera) # register pipeline stop on exit

        # get camera intrinsics
        color_stream_profile = pipeline_profile.get_stream(rs.stream.color)
        intrinsics = color_stream_profile.as_video_stream_profile().get_intrinsics()
        self.intrinsics = self.intrinsics_to_matrix(intrinsics)

        # get depth scale
        device = pipeline_profile.get_device()
        depth_sensor = device.first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def intrinsics_to_matrix(self, intrinsics):
        """
        intrinsics: realsense object
        matrix: 3x3 camera intrinsics
        """
        matrix = np.zeros((3,3))
        matrix[0,0] = intrinsics.fx
        matrix[1,1] = intrinsics.fy
        matrix[0,2] = intrinsics.ppx
        matrix[1,2] = intrinsics.ppy
        matrix[2,2] = 1.0
        return matrix

    def next_images(self):
        """Blocking, waits for depth and color.
        color: (h,w,3) np.uint8
        depth: (h,w) np.float32 - depth in meters
        """
        finished = False
        while not finished:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            failed = not depth_frame or not color_frame
            finished = not failed

        # convert image to np arrays
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        # convert depth to meters
        depth_meters = np.array(depth*self.depth_scale, np.float32)
        return color, depth_meters

    def close_camera(self):
        print('\nclosing camera')
        self.pipeline.stop()







def intrinsics_to_gtsam(intrinsics):
    fx = intrinsics[0,0]
    fy = intrinsics[1,1]
    cx = intrinsics[0,2]
    cy = intrinsics[1,2]
    return gtsam.Cal3_S2(fx, fy, 0, cx, cy)





def initialize_quadric(depth, box, camera_pose, calibration, object_depth=0.1):
    """
    depth: (h,w) depth image in meters
    """
    # get average box depth 
    dbox = box.vector().astype('int') # get discrete box bounds
    box_depth = depth[dbox[1]:dbox[3], dbox[0]:dbox[2]].mean()
    
    center = box.center()
    x = (center.x() - calibration.px()) * box_depth / calibration.fx()
    y = (center.y() - calibration.py()) * box_depth / calibration.fy()
    relative_point = gtsam.Point3(x, y, box_depth)
    quadric_center = camera_pose.compose(gtsam.Pose3(camera_pose.rotation(), relative_point))

    tx = (box.xmin() - calibration.px()) * box_depth / calibration.fx()
    ty = (box.ymin() - calibration.py()) * box_depth / calibration.fy()
    radii = np.array([np.abs(tx-x), np.abs(ty-y), object_depth])

    quadric = gtsam_quadrics.ConstrainedDualQuadric(quadric_center, radii)
    
    # relative_point = gtsam.Point3(x, y, 2.0)
    # quadric_center = camera_pose.compose(gtsam.Pose3(camera_pose.rotation(), relative_point))
    # quadric = gtsam_quadrics.ConstrainedDualQuadric(quadric_center, np.array([0.1]*3))


    return quadric


if __name__ == '__main__': 

    # setup video writer
    video_writer = cv2.VideoWriter('initialisation2.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 14, (640, 480))
    atexit.register(video_writer.release)

    # setup camera 
    camera = RealsenseCamera()

    # setup detector
    predictor = FasterRCNN('COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml', batch_size=5)

    # convert calibration matrix to gtsam
    calibration = intrinsics_to_gtsam(camera.intrinsics)

    while True:
        # get new images
        color, depth = camera.next_images()

        # run detector 
        detections = predictor([color])[0]
        boxes = [d.box for d in detections]

        # filter large boxes
        boxes = [b for b in boxes if b.width()*b.height() < 320*240]

        # initialize quadrics
        quadrics = [initialize_quadric(depth, box, gtsam.Pose3(), calibration, object_depth=0.1) for box in boxes]

        # draw boxes and quadrics
        image = color.copy()
        drawing = CV2Drawing(image)
        for box, quadric in zip(boxes, quadrics):
            drawing.quadric(gtsam.Pose3(), quadric, calibration, (255,0,255))
            drawing.box(box, (0,0,255))
        cv2.imshow('current view', image)
        cv2.waitKey(1)

        video_writer.write(image)


