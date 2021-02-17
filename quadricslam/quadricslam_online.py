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




name_to_estimator = {
    'Cauchy': gtsam.noiseModel_mEstimator_Cauchy,
    'DCS': gtsam.noiseModel_mEstimator_DCS,
    'Fair': gtsam.noiseModel_mEstimator_Fair,
    'GemanMcClure': gtsam.noiseModel_mEstimator_GemanMcClure,
    'Huber': gtsam.noiseModel_mEstimator_Huber,
    'Tukey': gtsam.noiseModel_mEstimator_Tukey,
    'Welsch': gtsam.noiseModel_mEstimator_Welsch,
}

name_to_parameter = {
    'Cauchy': 0.30,
    'DCS': 3.79,
    'Fair': 0.1,
    'GemanMcClure': 2.64,
    'Huber': 0.1,
    'Tukey': 16.24,
    'Welsch': 5.46,
}




class QuadricSLAM_Online(object):
    def __init__(self, calibration, config):
        self.config = config
        self.calibration = calibration
        
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

        # robust_estimator = name_to_estimator['Tukey'](10.0)
        # self.pose_noise = gtsam.noiseModel_Robust(robust_estimator, self.pose_noise)
        # self.bbox_noise = gtsam.noiseModel_Robust(robust_estimator, self.bbox_noise)
        # self.quadric_noise = gtsam.noiseModel_Robust(robust_estimator, self.quadric_noise)

        # set measurement storage 
        self.detections = Detections()

        # store current estimates
        self.current_trajectory = Trajectory()
        self.current_quadrics = Quadrics()

        # initialize data-association module
        # self.data_association = DataAssociation(self.calibration, config)
        self.data_association = KeepAssociator(thresh=0.8, layer='fc2')
        # self.data_association = MergeAssociator(thresh=0.8, layer='fc2')

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

    def update(self, image, associated_detections, camera_pose):
        pose_key = self.frames
        self.frames += 1

        # # filter object detections
        # if self.config['QuadricSLAM.filter_measurements']:
        #     image_detections = self.filter_detections(image_detections)

        # # draw current map and measurements
        # self.visualize(image, image_detections, camera_pose)
        self.visualize(image, associated_detections.values(), camera_pose)

        # visualize associated detections
        img = image.copy()
        drawing = CV2Drawing(img)
        for (pkey, object_key), detection in associated_detections.items():
            assert pkey == pose_key
            drawing.box_and_text(detection.box, (255,255,0), '{}'.format(object_key), (255,255,255))
        cv2.imshow('data-association', img)
        cv2.waitKey(1)

        # associate new measurements with existing keys
        # associated_detections = self.data_association.associate(image, image_detections, camera_pose, pose_key, self.current_quadrics, visualize=True, verbose=True)
        # associated_detections = self.data_association.associate(image, image_detections, pose_key)

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
            
            # extract poses-detections
            object_boxes = [d.box for d in object_detections.values()]
            pose_keys = object_detections.keys()
            object_poses = self.current_trajectory.at_keys(pose_keys)
            
            if not len(object_detections) >= self.config['QuadricSLAM.min_views']:
                continue

            # initialize object if seen enough
            if self.config['QuadricSLAM.init_method'] == 'SVD':
                if not len(object_detections) >= self.config['QuadricSLAM.min_views']:
                    continue
                quadric = Initialize.SVD(object_poses, object_boxes, self.calibration)
                
            elif self.config['QuadricSLAM.init_method'] == 'simple': 
                quadric = Initialize.simple(object_poses[-1], object_boxes[-1], self.calibration, depth=1.0, size=0.01) 

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
                bbf = gtsam_quadrics.BoundingBoxFactor(detection.box, self.calibration, self.X(pose_key), self.Q(object_key), self.bbox_noise, self.config['QuadricSLAM.measurement_model'])
                local_graph.add(bbf)
                self.detections.set_used(True, pose_key, object_key)

        # use local graph / estimate to update isam2
        self.isam.update(local_graph, local_estimate)

        # calculate current estimate
        current_estimate = self.isam.calculateEstimate()
        
        # update current estimate 
        self.current_trajectory = Trajectory.from_values(current_estimate)
        self.current_quadrics = Quadrics.from_values(current_estimate)


# if dataset has no detections, run detector
# if dataset has no odometry, run odom
# if dataset has no DA, run DA

# anyone can use this to run on their dataset 
# they can chose weather to use included odom/detections/da


def run_tum():
    # load config
    config = yaml.safe_load(open('/home/lachness/git_ws/quadricslam/quadricslam/config/online.yaml', 'r'))

    # load dataset
    dataset = TUMDataset('/media/lachness/DATA/Datasets/TUM/')

    # load detector
    predictor = FasterRCNN('COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml', batch_size=5)

    print('starting')

    for scene in dataset:

        # start SLAM
        SLAM = QuadricSLAM_Online(scene.calibration, config)
        print('testing scene')

        # iterate through timesteps and send to SLAM
        frames = 0
        for time, rgb_path in scene.aligned_rgb.items():


            # load image
            rgb_image = cv2.imread(rgb_path)

            # load odometry
            camera_pose = scene.aligned_trajectory[time]

            # calculate detections
            # detections = predictor([rgb_image])[0]
            detections = Detections()
            for object_key, d in scene.associated_detections.at_pose(time).items():
                detections.add(d, frames, object_key)
            

            SLAM.update(rgb_image, detections, camera_pose)
            frames += 1






def run_scenenet():
    config = yaml.safe_load(open('/home/lachness/git_ws/quadricslam/quadricslam/config/online.yaml', 'r'))
    dataset = SceneNetDataset(
        dataset_path = "/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/data/train",
        protobuf_folder = "/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/data/train_protobufs",
        reader_path = "/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/scenenet_pb2.py",
    )


    # dataset pose keys need to match frame_n
    for i, scene in enumerate(dataset):
        if i < 1:
            continue


        SLAM = QuadricSLAM_Online(scene.calibration, config)
        
        images = scene.images
        detections = scene.true_detections
        trajectory = scene.true_trajectory

        # filter partials
        image_bounds = gtsam_quadrics.AlignedBox2(0,0,320.,240.)
        filter_pixels = 10
        filter_bounds = image_bounds.vector() + np.array([1,1,-1,-1])*filter_pixels
        filter_bounds = gtsam_quadrics.AlignedBox2(filter_bounds)
        filtered_detections = Detections()
        for (pose_key, object_key), detection in detections.items():
            if filter_bounds.contains(detection.box):
                filtered_detections.add(detection, pose_key, object_key)

        detections = filtered_detections
        

        for pose_key, pose in trajectory.items():
            image = images[pose_key]

            # wrap back in Detections object
            image_detections = Detections()
            for object_key, detection in detections.at_pose(pose_key).items():
                image_detections.add(detection, pose_key, object_key)

            SLAM.update(image, image_detections, pose)



if __name__ == '__main__':
    run_scenenet()