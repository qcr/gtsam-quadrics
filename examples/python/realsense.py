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
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../quadricslam'))

# import custom python modules
sys.dont_write_bytecode = True
# from visualization.drawing import CV2Drawing
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

class OdometryWrapper(object):
    def __init__(self, intrinsics):
        """intrinsics: 3x3 camera calibration"""
        self.odometry = cv2.rgbd.RgbdOdometry_create(intrinsics)
        self.prev_image = None
        self.prev_depth = None

    def compute(self, image, depth):
        """First call will return None
        image = (h,w,1) np.uint8 - grayscale 
        depth = (h,w,1) np.float32 - depth in meters
        odom: 4x4 pose matrix, transform from current frame to previous frame
        """
        if self.prev_image is None:
            self.prev_image = image
            self.prev_depth = depth
            return None

        transform = np.ones((4,4))
        mask = np.ones(image.shape[0:2], np.uint8)
        self.odometry.compute(self.prev_image, self.prev_depth, mask, gray, depth, mask, transform)
        odom = gtsam.Pose3(transform).inverse()

        self.prev_image = image
        self.prev_depth = depth
        return odom




if __name__ == '__main__': 

    # setup camera 
    camera = RealsenseCamera()

    # setup odometry
    odometry = OdometryWrapper(camera.intrinsics)

    # setup detector
    predictor = FasterRCNN('COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml', batch_size=5)

    # -------- setup slam system -----------------------------------
    settings = quadricslam.settings() # default or load from file 
    calibration = matrix_to_cal(camera.intrinsics)
    qslam = quadricslam.QuadricSLAM(camera_settings, method_settings, optimizer_settings)
    # -----------------------------------------------------------------

    # setup associator
    associator = quadricslam.associator(settings)

    while True:
        # get new images
        color, depth = camera.next_images()

        # convert image to greyscale
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        # calculate odometry
        odom = odometry.compute(gray, depth)
        if odom is None: 
            continue

        # run detector 
        detections = predictor(color)
        boxes = [d.box for d in detections]

        # associate detections
        associated_boxes = associator.associate(color, boxes)

        # initialize new quadrics
        new_quadrics = quadricslam.initialize(measurement_history, current_quadrics, associated_boxes)
        
        # ------------------------------------------------------
        # update slam system
        qslam.update(odom, associated_detections, new_quadrics)

        # optimize
        quadrics, poses = qslam.optimize()
        
        # visualize current pose
        qslam.visualize(quadrics, poses)
        # ------------------------------------------------------


















# qslam.update()
# adds new odometry to graph
# adds new pose estimate to values
# runs measurements through data-association
# records associated measurments
# tries to initialize new landmarks
# adds new measurements/landmarks to graph/values if valid



# initializer
# NOTE: check if we have any object keys in history that are uninitialized (needs full measurement history, current quadrics)
# NOTE: check they will be constrained if init
# NOTE: check they have been initialized correctly 
# NOTE: either provide a way for user to get measurement history / quadric keys
# NOTE: OR get the user to overload the initialization method / pass the function in
# new init methods have different constraints (n views, depth info, etc)


# NOTE: cant we just make a base qslam system and derive to modify odometry / initialization
# what if the user wants to know the current pose_key?
# associator also needs copy of detections history
# QSLAM should HAVE-A associator/initializer and they can derive?


# COMPONENTS: core, odometry, detection, data-association, initialization, visualization, evaluation 



# camera (settings): 
    # .get_next_images (void): returns rgb/depth
    # .intrinsics: 3x3 matrix
# odometry (settings, intrinsics): 
    # .compute (color, depth): returns relative pose
# detector (settings): takes in image(s) and returns boxes
    # .detect (color): returns [boxes]


# new image, odom, detections!


# associator (settings)
    # .associate (history, color, boxes): returns associated boxes
    # probably stores some version of history (cnn codes / full detections) to aid future associations

    # NOTE: might need .get_past_measurements()

# initializer (settings)
    # .initialize (history, current_quadrics, new_associated_boxes): returns new_quadrics
        # tries to initialize objects that haven't been initialized before
        # checks to make sure object is constrained and initialization is valid

    # NOTE: needs .get_past_measurements(), .get_current_quadric_keys()

# system (settings)
    # .update (odom, associated_boxes, new_quadrics, history)
    # .optimize (void): quadrics, trajectory

    # only adds unused measurements to graphs
    # trusts initializer that quadrics are new, constrained, valid



# if init strategy is single-view, should be initialized immedietly and added when constrained
# does the user need to know the current pose_key? 
    # either give qslam.update() all the information and let it do the background magic
    # or only give it things it can immedietly add to graph/values and optimize

# quadricslam controls when to initialize new landmarks
# 

# trajectory: list[gtsam.Pose3]
# odometry: list[gtsam.Pose3]

