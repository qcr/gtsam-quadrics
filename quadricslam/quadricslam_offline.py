"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Offline Quadric SLAM Front-end
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import configparser
import argparse
import yaml
import code

# import gtsam and extension
import gtsam
import gtsam_quadrics

# import custom python modules
sys.dont_write_bytecode = True
from dataset_interfaces.simulated_dataset import SimulatedSequence
from dataset_interfaces.scenenet_dataset import SceneNetDataset
from visualization.drawing import MPLDrawing
from base.evaluation import Evaluation
from base.containers import *
from base.initialisation import Initialize


class QuadricSLAM_Offline(object):
    """
    Python front-end to build graph/estimate from dataset. 
    """
    X = lambda i: int(gtsam.symbol(ord('x'), i))
    Q = lambda i: int(gtsam.symbol(ord('q'), i))

    def __init__(self, calibration, params, config):
        self.calibration = calibration 
        self.params = params
        self.config = config

    def run(self, noisy_trajectory, noisy_detections, initial_quadrics=None, true_trajectory=None, evaluate=False, visualize=False, robust_estimator=None, custom_noise=False):

        # initialize constrainable quadrics
        if initial_quadrics is None:
            initial_quadrics = self.initialize_quadrics(noisy_trajectory, noisy_detections, self.calibration)
        else:
            views_per_object = dict(zip(*np.unique(noisy_detections.object_keys(), return_counts=True)))
            constrained_objects = [k for k,v in views_per_object.items() if v >= self.config['QuadricSLAM.min_views']]
            initial_quadrics = initial_quadrics.keep_keys(constrained_objects)
            
        # build graph / estimate
        graph = self.build_graph(noisy_trajectory, noisy_detections, initial_quadrics, robust_estimator, custom_noise)
        initial_estimate = self.build_estimate(noisy_trajectory, initial_quadrics)

        # draw initial system
        if visualize:
            plotting = MPLDrawing('initial_problem')
            plotting.plot_system(graph, initial_estimate)

        # optimize using c++ back-end
        estimate = self.optimize(graph, initial_estimate)

        # draw estimation
        if visualize:
            plotting = MPLDrawing('final_solution')
            plotting.plot_system(graph, estimate)

        # extract quadrics / trajectory 
        estimated_trajectory = Trajectory.from_values(estimate)
        estimated_quadrics = Quadrics.from_values(estimate)

        # evaluate results
        if evaluate:
            initial_ATE_H = Evaluation.evaluate_trajectory(Trajectory.from_values(initial_estimate), true_trajectory, type='horn')[0]
            estimate_ATE_H = Evaluation.evaluate_trajectory(estimated_trajectory, true_trajectory, type='horn')[0]
            initial_ATE = Evaluation.evaluate_trajectory(Trajectory.from_values(initial_estimate), true_trajectory, type='weak')[0]
            estimate_ATE = Evaluation.evaluate_trajectory(estimated_trajectory, true_trajectory, type='weak')[0]
            print('Initial ATE w/ horn alignment: {}'.format(initial_ATE_H))
            print('Final ATE w/ horn alignment:   {}'.format(estimate_ATE_H))
            print('Initial ATE w/ weak alignment: {}'.format(initial_ATE))
            print('Final ATE w/ weak alignment:   {}'.format(estimate_ATE))

        # plot results
        if visualize:
            trajectories = [Trajectory.from_values(initial_estimate), estimated_trajectory, true_trajectory]
            maps = [Quadrics.from_values(initial_estimate), estimated_quadrics]
            colors = ['r', 'm', 'g']; names = ['initial_estimate', 'final_estimate', 'ground_truth']
            plotting.plot_result(trajectories, maps, colors, names)
           
        return estimated_trajectory, estimated_quadrics

    def optimize(self, graph, initial_estimate):
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, self.params)
        estimate = optimizer.optimize()
        return estimate

    def build_graph(self, initial_trajectory, noisy_detections, initial_quadrics, robust_estimator=None, custom_noise=False):
        # create empty graph
        graph = gtsam.NonlinearFactorGraph()

        # declare noise models
        prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([self.config['QuadricSLAM.prior_sd']]*6, dtype=np.float))
        odometry_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([self.config['QuadricSLAM.odom_sd']]*6, dtype=np.float))
        bbox_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([self.config['QuadricSLAM.box_sd']]*4, dtype=np.float))

        # convert to robust estimators
        if robust_estimator is not None:
            prior_noise = gtsam.noiseModel_Robust(robust_estimator, prior_noise)
            odometry_noise = gtsam.noiseModel_Robust(robust_estimator, odometry_noise)
            bbox_noise = gtsam.noiseModel_Robust(robust_estimator, bbox_noise)

        # calculate odometry from trajectory
        noisy_odometry = initial_trajectory.as_odometry()

        # add prior pose
        prior_factor = gtsam.PriorFactorPose3(QuadricSLAM_Offline.X(0), initial_trajectory.at(0), prior_noise)
        graph.add(prior_factor)

        # add odometry measurements
        for (start_key, end_key), rpose in noisy_odometry.items():
            odometry_factor = gtsam.BetweenFactorPose3(QuadricSLAM_Offline.X(start_key), QuadricSLAM_Offline.X(end_key), rpose, odometry_noise)
            graph.add(odometry_factor)

        # add valid box measurements
        for object_key, object_detections in noisy_detections.per_object():

            # add if quadric initialized and constrained
            if object_key in initial_quadrics.keys():
                
                # calculate width / height std
                if custom_noise:
                    dimensions = np.array([[detection.box.width(), detection.box.height()] for detection in object_detections.values()])
                    sigma = dimensions.std(0).sum()
                    bbox_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([sigma]*4, dtype=np.float))
                    if robust_estimator is not None:
                        bbox_noise = gtsam.noiseModel_Robust(robust_estimator, bbox_noise)

                # add measurements
                for pose_key, detection in object_detections.items():
                    bbf = gtsam_quadrics.BoundingBoxFactor(detection.box, self.calibration, QuadricSLAM_Offline.X(pose_key), QuadricSLAM_Offline.Q(object_key), bbox_noise, self.config['QuadricSLAM.error_type'])
                    graph.add(bbf)


        # add weak quadric priors
        # for object_key, quadric in initial_quadrics.items():
        #     quadric_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([10]*9, dtype=np.float))
        #     prior_factor = gtsam_quadrics.PriorFactorConstrainedDualQuadric(QuadricSLAM_Offline.Q(object_key), quadric, quadric_noise)
        #     graph.add(prior_factor)


        return graph

    def build_estimate(self, initial_trajectory, initial_quadrics):
        # create initial estimate
        initial_estimate = gtsam.Values()

        # add initial pose estimates
        for pose_key, pose in initial_trajectory.items():
            initial_estimate.insert(QuadricSLAM_Offline.X(pose_key), pose)

        # add initial landmark estimates if constrained and initialized
        for object_key, quadric in initial_quadrics.items():
            quadric.addToValues(initial_estimate, QuadricSLAM_Offline.Q(object_key))

        return initial_estimate

    def initialize_quadrics(self, trajectory, detections, calibration):
        """ Uses SVD to initialize quadrics from measurements """
        quadrics = Quadrics()

        # loop through object keys
        for object_key, object_detections in detections.per_object():

            # get the object boxes from each pose
            object_boxes = [d.box for d in object_detections.values()]

            # get the poses associated with each detection 
            pose_keys = list(object_detections.keys())
            poses = trajectory.at_keys(pose_keys)

            # ensure quadric seen from enough views
            if len(np.unique(pose_keys)) < self.config['QuadricSLAM.min_views']:
                continue

            # initialize quadric fomr views using svd
            quadric = Initialize.SVD(poses, object_boxes, calibration)
            # quadric = Initialize.simple(poses[0], object_boxes[0], calibration, depth=1.0, size=0.01)

            # check quadric is okay
            if quadric is not None:
                quadrics.add(quadric, object_key)

        return quadrics



def load_calibration(config):
    calibration = gtsam.Cal3_S2(
        config['Camera.fx'],
        config['Camera.fy'],
        0.0,
        config['Camera.cx'],
        config['Camera.cy'],
    )
    return calibration

def load_optimizer_parms(config):
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM(config['Optimizer.verbosity'])    
    params.setMaxIterations(config['Optimizer.max_iterations'])
    params.setlambdaInitial(config['Optimizer.lambda_initial'])
    params.setlambdaUpperBound(config['Optimizer.lambda_upper_bound'])
    params.setlambdaLowerBound(config['Optimizer.lambda_lower_bound'])
    params.setRelativeErrorTol(config['Optimizer.relative_error_tol'])
    params.setAbsoluteErrorTol(config['Optimizer.absolute_error_tol'])
    return params

def load_sequence(config):
    if config['Dataset.name'] == 'SceneNet':
        dataset = SceneNetDataset(
            dataset_path = config['Dataset.data'],
            protobuf_folder = config['Dataset.protobufs'],
            reader_path = config['Dataset.protobuf_definition'],
        )
        sequence = dataset[config['Dataset.sequence_n']]
    else:
        sequence = SimulatedSequence.sequence1()
    return sequence



if __name__ == '__main__':
    # parse input arguments for config file
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', help='path to config file', required=True)
    args = parser.parse_args()

    # load system configuration
    config = yaml.safe_load(open(args.config, 'r'))

    # validate configuration 
    if config['Dataset.name'] == 'SceneNet':

        # if SceneNet, ensure all paths are provided 
        required_settings = ['Dataset.data', 'Dataset.protobufs', 'Dataset.protobuf_definition', 'Dataset.sequence_n']
        if not all(setting in config for setting in required_settings):
            raise RuntimeError("If SceneNet selected as dataset, must provide path to SceneNet files.")

    # load data from config
    opt_params = load_optimizer_parms(config)
    calibration = load_calibration(config)
    sequence = load_sequence(config)

    # set experiment seed 
    np.random.seed(config['Noise.seed'])

    # get true trajectory and detections
    true_trajectory = sequence.true_trajectory
    true_detections = sequence.true_detections

    # inject noise into trajectory
    true_odometry = true_trajectory.as_odometry()
    noisy_odometry = true_odometry.add_noise(mu=0.0, sd=config['Noise.odom_sd'])
    noisy_trajectory = noisy_odometry.as_trajectory(true_trajectory.values()[0])
    
    # inject noise into detections
    noisy_detections = true_detections.add_noise(mu=0.0, sd=config['Noise.box_sd'])


    # run system on data
    system = QuadricSLAM_Offline(calibration, opt_params, config)
    system.run(noisy_trajectory, noisy_detections, true_trajectory=true_trajectory, evaluate=True, visualize=True)
