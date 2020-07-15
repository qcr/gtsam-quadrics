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

# import custom python modules
sys.dont_write_bytecode = True
from base.data_association import DataAssociation
from dataset_interfaces.scenenet_dataset import SceneNetDataset
from visualization.drawing import CV2Drawing
from base.containers import Trajectory, Quadrics, Detections, ObjectDetection

# import gtsam and extension
import gtsam
import gtsam_quadrics


class QuadricSLAM_Online(object):
    def __init__(self, config):
        self.config = config
        
        # load camera calibration
        self.calibration = self.load_calibration(config)

        # load class names
        self.class_names = self.load_class_names(config['QuadricSLAM.classes_path'])

        # create isam2 optimizer 
        self.isam = self.create_optimizer(config)
        
        # define gtsam macros 
        self.X = lambda i: int(gtsam.symbol(ord('x'), i))
        self.Q = lambda i: int(gtsam.symbol(ord('q'), i))

        # declare noise models
        self.pose_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([config['QuadricSLAM.pose_sd']]*6, dtype=np.float))
        self.bbox_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([config['QuadricSLAM.box_sd']]*4, dtype=np.float))
        self.quadric_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([config['QuadricSLAM.quad_sd']]*9, dtype=np.float))

        # set measurement storage 
        self.detections = Detections()

        # store current estimates
        self.current_trajectory = Trajectory()
        self.current_quadrics = Quadrics()

        # initialize data-association module
        self.data_association = DataAssociation(self.calibration, config)

        # prepare video capture (initialized on first use)
        self.video_writer = None

        # store processed frames to number pose_keys
        self.frames = 0

    def create_optimizer(self, config):
        if config['Optimizer.dogleg']:
            opt_params = gtsam.ISAM2DoglegParams()
        else:
            opt_params = gtsam.ISAM2GaussNewtonParams()
        parameters = gtsam.ISAM2Params()
        parameters.setOptimizationParams(opt_params)
        parameters.setEnableRelinearization(config['Optimizer.relinearization'])
        parameters.setRelinearizeThreshold(config['Optimizer.relinearize_thresh'])
        parameters.setRelinearizeSkip(config['Optimizer.relinearize_skip'])
        parameters.print_("ISAM2 Parameters")
        isam = gtsam.ISAM2(parameters)
        return isam

    def load_class_names(self, path):
        classes_fp = open(path, 'r')
        return classes_fp.read().split('\n')[:-1]

    def load_calibration(self, camera_config, no_distortion=True):
        """ Loads gtsam calibration from openvslam config format """
        camera_model = gtsam.Cal3_S2
        calibration_list = [
            camera_config['Camera.fx'],
            camera_config['Camera.fy'],
            0.0,
            camera_config['Camera.cx'],
            camera_config['Camera.cy'],
        ]

        if no_distortion:
            return camera_model(*calibration_list)

        if 'Camera.k1' in camera_config:
            camera_model = gtsam.Cal3DS2
            calibration_list += [
                camera_config['Camera.k1'],
                camera_config['Camera.k2'],
            ]

        if 'Camera.p1' in camera_config:
            calibration_list += [
                camera_config['Camera.p1'],
                camera_config['Camera.p2'],
            ]

        return camera_model(*calibration_list)
        
    def filter_detections(self, image_detections):
        indicies = [self.class_names.index(name) for name in self.config['QuadricSLAM.viable_classes']]
        return [d for d in image_detections if np.argmax(d.scores) in indicies]

    def visualize(self, image, image_detections, camera_pose):
        # draw detections
        img = image.copy()
        drawing = CV2Drawing(img)
        for detection in image_detections:
            scores = detection.scores
            text = '{}:{:.2f}'.format(self.class_names[np.argmax(scores)], np.max(scores))
            drawing.box_and_text(detection.box, (0,0,255), text, (0,0,0))

        # draw current map 
        for quadric in self.current_quadrics.values():
            drawing.quadric(camera_pose, quadric, self.calibration, (255,0,255))
        cv2.imshow('Current view', img)
        cv2.waitKey(1)

        # record map + detections
        if self.config['Recording.record']:
            if self.video_writer is None:
                self.video_writer = cv2.VideoWriter('performance.mp4', cv2.VideoWriter_fourcc(*'MP4V'), self.config['Camera.fps'], (image.shape[1], image.shape[0]))
                atexit.register(self.video_writer.release)
            self.video_writer.write(img)

    def update(self, image, image_detections, camera_pose):
        pose_key = self.frames
        self.frames += 1

        # filter object detections
        if self.config['QuadricSLAM.filter_measurements']:
            image_detections = self.filter_detections(image_detections)

        # draw current map and measurements
        self.visualize(image, image_detections, camera_pose)

        # associate new measurements with existing keys
        associated_detections = self.data_association.associate(image, image_detections, camera_pose, pose_key, self.current_quadrics, visualize=True, verbose=True)

        # store new boxes for later initialization and factor adding
        self.detections.add_detections(associated_detections)

        # add new camera pose to current estimate
        # we do this so we can access the current pose for initialization
        self.current_trajectory.add(camera_pose, pose_key)

        # create local graph and estimate
        local_graph = gtsam.NonlinearFactorGraph()
        local_estimate = gtsam.Values()
        
        # add new pose measurements to graph / estimate
        local_estimate.insert(self.X(pose_key), camera_pose)
        prior_factor = gtsam.PriorFactorPose3(self.X(pose_key), camera_pose, self.pose_noise)
        local_graph.add(prior_factor)

        # check if we can initialize any new objects
        for object_key, object_detections in self.detections.per_object():

            # no need to re-initialize objects
            if object_key in self.current_quadrics.keys():
                continue
            
            # initialize object if seen enough
            quadric = self.initialize_quadric(object_key, object_detections, self.current_trajectory, local_estimate)

            # continue if not correctly initialized 
            if quadric is None: 
                continue

            # add quadric to values 
            quadric.addToValues(local_estimate, self.Q(object_key))

            # add weak quadric prior 
            prior_factor = gtsam_quadrics.PriorFactorConstrainedDualQuadric(self.Q(object_key), quadric, self.quadric_noise)
            local_graph.add(prior_factor)

            # add quadric to current quadrics
            # we do this to avoid reinitialization and as a flag to add new measurements
            self.current_quadrics.add(quadric, object_key)


        # add measurements if unused
        for (pose_key, object_key), detection in self.detections.items():

            # only add new measurements
            if self.detections.is_used(pose_key, object_key):
                continue            

            # add measurements if initialized 
            if object_key in self.current_quadrics.keys():
                bbf = gtsam_quadrics.BoundingBoxFactor(detection.box, self.calibration, self.X(pose_key), self.Q(object_key), self.bbox_noise)
                local_graph.add(bbf)
                self.detections.set_used(True, pose_key, object_key)

        # use local graph / estimate to update isam2
        self.isam.update(local_graph, local_estimate)

        # calculate current estimate
        current_estimate = self.isam.calculateEstimate()
        
        # update current estimate 
        self.current_trajectory = Trajectory.from_values(current_estimate)
        self.current_quadrics = Quadrics.from_values(current_estimate)

    def initialize_quadric(self, object_key, object_detections, current_trajectory, local_estimate):
        """ 
        Attempts to initialize the quadric according to self.initialization_method.
        Returns None if quadric could not be initialized 
        """
        if self.config['QuadricSLAM.init_method'] == 'SVD':
            if len(object_detections) >= self.config['QuadricSLAM.min_views']:

                object_boxes = [d.box for d in object_detections.values()]
                pose_keys = object_detections.keys()
                object_poses = current_trajectory.at_keys(pose_keys)
                quadric_matrix = self.quadric_SVD(object_poses, object_boxes, self.calibration)
                quadric = gtsam_quadrics.ConstrainedDualQuadric.constrain(quadric_matrix)

                # check quadric is okay
                if self.is_okay(quadric, object_poses, self.calibration):
                    return quadric

        else:
            abox = list(object_detections.values())[0]
            apose_key = list(object_detections.keys())[0]
            apose = current_trajectory.at(apose_key)
            displacement = 0.5
            quadric_pose = apose.compose(gtsam.Pose3(gtsam.Rot3(),gtsam.Point3(0,0,displacement)))
            quadric = gtsam_quadrics.ConstrainedDualQuadric(quadric_pose, np.array([0.01]*3))
            return quadric
        return None

    def quadric_SVD(self, poses, object_boxes, calibration):
        """ calculates quadric_matrix using SVD """

        # iterate through box/pose data
        planes = []
        for box, pose in zip(object_boxes, poses):

            # calculate boxes lines
            lines = box.lines()

            # convert Vector3Vector to list
            lines = [lines.at(i) for i in range(lines.size())]

            # calculate projection matrix
            P = gtsam_quadrics.QuadricCamera.transformToImage(pose, calibration).transpose()

            # project lines to planes
            planes += [P @ line for line in lines]

        # create A matrix
        A = np.asarray([np.array([p[0]**2,  2*(p[0]*p[1]),  2*(p[0]*p[2]),  2*(p[0]*p[3]),
                                                p[1]**2,  	2*(p[1]*p[2]),  2*(p[1]*p[3]),
                                                                p[2]**2,  	2*(p[2]*p[3]),
                                                                               p[3]**2]) for p in planes])

        # solve SVD for Aq = 0, which should be equal to p'Qp = 0
        _,_,V = np.linalg.svd(A, full_matrices=True)
        q = V.T[:, -1]

        # construct quadric
        dual_quadric = np.array([[q[0], q[1], q[2], q[3]],
                                [q[1], q[4], q[5], q[6]],
                                [q[2], q[5], q[7], q[8]],
                                [q[3], q[6], q[8], q[9]]])

        return dual_quadric

    def is_okay(self, quadric, poses, calibration):
        """
        Checks quadric is valid:
            quadric constrained correctly
            paralax > threshold
            reprojections valid in each frame 
                quadric infront of camera : positive depth 
                camera outside quadric
                conic is an ellipse 
            ensure views provide enough DOF (due to edges / out of frame)
        """
        for pose in poses:

            # quadric must have positive depth
            if quadric.isBehind(pose):
                return False

            # camera pose must be outside quadric 
            if quadric.contains(pose):
                return False

            # conic must be valid and elliptical 
            conic = gtsam_quadrics.QuadricCamera.project(quadric, pose, calibration)
            if conic.isDegenerate():
                return False
            if not conic.isEllipse():
                return False
                
        return True
