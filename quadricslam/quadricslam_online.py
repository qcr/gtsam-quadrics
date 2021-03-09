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
        self.gps_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([config['QuadricSLAM.gps_sd']]*6, dtype=np.float))
        self.odom_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([config['QuadricSLAM.odom_sd']]*6, dtype=np.float))
        self.box_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([config['QuadricSLAM.box_sd']]*4, dtype=np.float))
        self.pose_prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([config['QuadricSLAM.pose_prior_sd']]*6, dtype=np.float))
        self.quad_prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([config['QuadricSLAM.quad_prior_sd']]*9, dtype=np.float))
        self.angle_factor_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([config['QuadricSLAM.angle_factor_sd']]*3, dtype=np.float))
        self.rot_prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([config['QuadricSLAM.prior_rots_sd']]*3 + [config['QuadricSLAM.inf_sd']]*6, dtype=np.float))

        # robust_estimator = name_to_estimator['Tukey'](1.0)
        # self.gps_noise = gtsam.noiseModel_Robust(robust_estimator, self.gps_noise)
        # self.odom_noise = gtsam.noiseModel_Robust(robust_estimator, self.odom_noise)
        # self.box_noise = gtsam.noiseModel_Robust(robust_estimator, self.box_noise)
        # self.pose_prior_noise = gtsam.noiseModel_Robust(robust_estimator, self.pose_prior_noise)
        # self.quad_prior_noise = gtsam.noiseModel_Robust(robust_estimator, self.quad_prior_noise)
        # self.angle_factor_noise = gtsam.noiseModel_Robust(robust_estimator, self.angle_factor_noise)

        # set measurement storage 
        self.detections = Detections()

        # store current estimates
        self.current_estimate = None
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

        # store previous pose for odom
        self.prev_pose = gtsam.Pose3()
        self.global_graph = gtsam.NonlinearFactorGraph()
        self.global_values = gtsam.Values()

    def create_optimizer(self, config):
        if config['Optimizer'] == "ISAM-D":
            opt_params = gtsam.ISAM2DoglegParams()
        else:
            opt_params = gtsam.ISAM2GaussNewtonParams()
        parameters = gtsam.ISAM2Params()
        parameters.setOptimizationParams(opt_params)
        parameters.setEnableRelinearization(config['ISAM.relinearization'])
        parameters.setRelinearizeThreshold(config['ISAM.relinearize_thresh'])
        parameters.setRelinearizeSkip(config['ISAM.relinearize_skip'])
        parameters.setFactorization(config['ISAM.factorization'])
        parameters.setEnableDetailedResults(True)
        # parameters.print_("ISAM2 Parameters")
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

    def update(self, image, associated_detections, camera_pose, initial_quadrics=None):
        pose_key = self.frames


        # # filter object detections
        # if self.config['QuadricSLAM.filter_measurements']:
        #     image_detections = self.filter_detections(image_detections)

        # # draw current map and measurements
        # self.visualize(image, image_detections, camera_pose)
        self.visualize(image, associated_detections.values(), camera_pose)

        # visualize associated detections
        # img = image.copy()
        # drawing = CV2Drawing(img)
        # for (pkey, object_key), detection in associated_detections.items():
        #     assert pkey == pose_key
        #     drawing.box_and_text(detection.box, (255,255,0), '{}'.format(object_key), (255,255,255))
        # cv2.imshow('data-association', img)
        # cv2.waitKey(1)

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
        
        # add new pose measurements to values
        local_estimate.insert(self.X(pose_key), camera_pose)

        # add gps measurement or odometry
        if self.config['QuadricSLAM.gps']:
            gps_factor = gtsam.PriorFactorPose3(self.X(pose_key), camera_pose, self.gps_noise)
            local_graph.add(gps_factor)
        else:

            # add prior for first few poses
            if self.frames < self.config['QuadricSLAM.n_priors']:
                prior_factor = gtsam.PriorFactorPose3(self.X(pose_key), camera_pose, self.pose_prior_noise)
                local_graph.add(prior_factor)

            # add odom factor 
            if self.frames > 0:
                odom = self.prev_pose.between(camera_pose)
                odom_factor = gtsam.BetweenFactorPose3(self.X(pose_key-1), self.X(pose_key), odom, self.odom_noise)
                local_graph.add(odom_factor)
            self.prev_pose = camera_pose


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
                
            elif self.config['QuadricSLAM.init_method'] == 'SIMPLE': 
                quadric = Initialize.simple(object_poses[-1], object_boxes[-1], self.calibration, depth=1.0, size=0.01) 

            elif self.config['QuadricSLAM.init_method'] == 'TRUE':
                quadric = initial_quadrics.at(object_key)


            # continue if not correctly initialized 
            if quadric is None: 
                continue

            # add quadric to values 
            quadric.addToValues(local_estimate, self.Q(object_key))

            # TODO: HOW TO CHECK IF CONSTRAINED BEFORE ADDING QUADS

            # add weak quadric prior 
            if self.config['QuadricSLAM.zero_prior']:
                prior_quad = gtsam_quadrics.ConstrainedDualQuadric()
            else:
                prior_quad = quadric

            if self.config['QuadricSLAM.quad_priors']:
                prior_factor = gtsam_quadrics.PriorFactorConstrainedDualQuadric(self.Q(object_key), prior_quad, self.quad_prior_noise)
                local_graph.add(prior_factor)

            if self.config['QuadricSLAM.prior_rots']:
                prior_factor = gtsam_quadrics.PriorFactorConstrainedDualQuadric(self.Q(object_key), prior_quad, self.rot_prior_noise)
                local_graph.add(prior_factor)
                

            if self.config['QuadricSLAM.angle_factors']:
                angle_factor = gtsam_quadrics.QuadricAngleFactor(self.Q(object_key), prior_quad.pose().rotation(), self.angle_factor_noise)
                local_graph.add(angle_factor)

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
                bbf = gtsam_quadrics.BoundingBoxFactor(detection.box, self.calibration, self.X(pose_key), self.Q(object_key), self.box_noise, self.config['QuadricSLAM.measurement_model'])
                local_graph.add(bbf)
                self.detections.set_used(True, pose_key, object_key)


        # add to global graph/values
        self.global_graph.push_back(local_graph)
        self.global_values.insert(local_estimate)

        # check global graph/values
        global_factors = [self.global_graph.at(idx) for idx in range(self.global_graph.size())]
        startkeys = [f.keys().at(0) for f in global_factors]

        bbfs = [f for f in global_factors if f.keys().size() == 2 and gtsam.symbolChr(f.keys().at(1)) == 'q']
        
        # TODO: check using only global graph/values. How to know which factor is bbf?
        if len(self.current_quadrics) > 0 and False:
            print('Checking validy of global graph/values manually')
            for key in self.current_quadrics.keys():
                bbfs = [f for f in global_factors if f.keys().size() == 2 and f.keys().at(1) == self.Q(key)]

                poses_exist = all([self.global_values.exists(bbf.keys().at(0)) for bbf in bbfs]) # each pose must have value
                poses_constrained = all([bbf.keys().at(0) in startkeys for bbf in bbfs]) # each pose must have prior/between
                quad_exists = self.global_values.exists(self.Q(key))
                quad_constrained = len(bbfs) > 3
                valid = all([poses_exist, poses_constrained, quad_exists, quad_constrained])

                # check DOF per factor
                # calculate H2 (4x9) and count nonzero rows
                bbfs = [gtsam_quadrics.dynamic_cast_BoundingBoxFactor_NonlinearFactor(f) for f in bbfs]
                dquads = [f.evaluateH2(self.global_values) for f in bbfs]
                DOFs = np.sum([np.count_nonzero((dquad != 0).sum(1)) for dquad in dquads])

                errors = [np.square(bbf.unwhitenedError(self.global_values)).sum() for bbf in bbfs]

                # check conditioning per quad
                conds = [np.linalg.cond(dquad) for dquad in dquads]

                frame = {
                    'object': key,
                    'valid': valid,
                    'DOFs': DOFs,
                    'avg cond': np.mean(conds),
                    'max cond': np.max(conds),
                    'avg error': np.mean(errors),
                    'max error': np.max(errors),
                }
                print('-----------------')
                for key, value in frame.items():
                    print(key, value)
                


        # check isam graph/values
        if self.current_estimate is not None and len(self.current_quadrics) > 0 and False:

            print('\nchecking global graph/values')
            v1 = self.valid_system(self.global_graph, self.global_values)
            
            # print('\nchecking isam graph/est')
            # v2 = self.valid_system(self.isam.getFactorsUnsafe(), self.current_estimate)

            # print('\nchecking isam graph/initial values')
            # v3 = self.valid_system(self.isam.getFactorsUnsafe(), self.global_values)

            # if not all([v1,v2,v3]):
            # if not v1:
            #     import code
            #     code.interact(local=dict(globals(),**locals()))


        try: 
            if self.config['Optimizer'] in ["ISAM", "ISAM-D"]:
                self.isam.update(local_graph, local_estimate)
                self.current_estimate = self.isam.calculateEstimate()
            elif self.config['Optimizer'] == "LVM":
                params = gtsam.LevenbergMarquardtParams()
                params.setVerbosityLM("SUMMARY")    
                params.setMaxIterations(20)
                params.setlambdaInitial(1.0e-5)
                params.setlambdaUpperBound(1.0e+10)
                params.setlambdaLowerBound(1.0e-8)
                params.setRelativeErrorTol(1.0e-5)
                params.setAbsoluteErrorTol(1.0e-5)
                optimizer = gtsam.LevenbergMarquardtOptimizer(self.global_graph, self.global_values, params)
                self.current_estimate = optimizer.optimize()
            elif self.config['Optimizer'] == "GN":
                params = gtsam.GaussNewtonParams()
                params.setVerbosity("ERROR")   
                params.setMaxIterations(20)
                params.setRelativeErrorTol(1.0e-5)
                params.setAbsoluteErrorTol(1.0e-5)
                optimizer = gtsam.GaussNewtonOptimizer(self.global_graph, self.global_values, params)
                self.current_estimate = optimizer.optimize()
        except Exception as e:
            self.frames += 1
            return False


        # update current estimate 
        self.current_trajectory = Trajectory.from_values(self.current_estimate)
        self.current_quadrics = Quadrics.from_values(self.current_estimate)
        self.frames += 1
        return True



    def valid_system(self, nlfg, values):
        gfg = nlfg.linearize(values) 
        jacobian = gfg.jacobian()[0]
        hessian = gfg.hessian()[0]
        valid = True

        # check if underdetermined
        if np.linalg.matrix_rank(jacobian) < values.dim() \
            or np.linalg.matrix_rank(hessian) < values.dim():
            print('  NOT VALID: underdetermined')
            valid = False

        # check if indefinite, i.e not positive semidefinite or negative semidefinite
        eigv = np.linalg.eigh(hessian)[0]
        if np.any(eigv<0) and np.any(eigv>0):
            print('  NOT VALID: indefinite hessian')
            valid = False

        if not np.all(eigv>0):
            print('  NOT VALID: not postive definite')
            valid = False

        # check conditioning 
        cond = np.linalg.cond(jacobian)
        print('  Conditioning: ', cond)

        # COND CHECKING:
        # check almost underconstrained variable
        # vastly different uncertainties
        return valid


    def classify_eigv(self, eigv):
        if np.all(eigv>0):
            return 'positive definite'
        if np.all(eigv>=0):
            return 'positive semidefinite'
        if np.all(eigv<0):
            return 'negative definite'
        if np.all(eigv<=0):
            return 'negative semidefinite'
        if np.any(eigv<0) and np.any(eigv>0):
            return 'indefinite'



# if dataset has no detections, run detector
# if dataset has no odometry, run odom
# if dataset has no DA, run DA

# anyone can use this to run on their dataset 
# they can chose weather to use included odom/detections/da


