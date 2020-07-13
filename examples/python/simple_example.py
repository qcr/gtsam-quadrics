"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: example problem using python interface 
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# import gtsam and extension
import gtsam
import gtsam_quadrics

if __name__ == '__main__':

    # declare noise estimates
    ODOM_SIGMA = 0.01
    BOX_SIGMA = 3

    # define shortcuts for symbols
    X = lambda i: int(gtsam.symbol(ord('x'), i))
    Q = lambda i: int(gtsam.symbol(ord('q'), i))

    # create empty graph / estimate
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # define calibration 
    calibration = gtsam.Cal3_S2(525.0, 525.0, 0.0, 160.0, 120.0)

    # define noise models
    prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([1e-1]*6, dtype=np.float))
    odometry_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([ODOM_SIGMA]*3 + [ODOM_SIGMA]*3, dtype=np.float))
    bbox_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([BOX_SIGMA]*4, dtype=np.float))

    # define poses 
    poses = []
    poses.append(gtsam.SimpleCamera.Lookat(gtsam.Point3(10,0,0), gtsam.Point3(), gtsam.Point3(0,0,1)).pose())
    poses.append(gtsam.SimpleCamera.Lookat(gtsam.Point3(0,-10,0), gtsam.Point3(), gtsam.Point3(0,0,1)).pose())
    poses.append(gtsam.SimpleCamera.Lookat(gtsam.Point3(-10,0,0), gtsam.Point3(), gtsam.Point3(0,0,1)).pose())
    poses.append(gtsam.SimpleCamera.Lookat(gtsam.Point3(0,10,0), gtsam.Point3(), gtsam.Point3(0,0,1)).pose())
    poses.append(gtsam.SimpleCamera.Lookat(gtsam.Point3(10,0,0), gtsam.Point3(), gtsam.Point3(0,0,1)).pose())

    # define quadrics
    quadrics = []
    quadrics.append(gtsam_quadrics.ConstrainedDualQuadric(gtsam.Pose3(), np.array([1.,2.,3.])))
    quadrics.append(gtsam_quadrics.ConstrainedDualQuadric(gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.1,0.1,0.1)), np.array([1.,2.,3.])))

    # add prior on first pose
    prior_factor = gtsam.PriorFactorPose3(X(0), poses[0], prior_noise)
    graph.add(prior_factor)

    # add trajectory estimate
    for i, pose in enumerate(poses):

        # add a perturbation to initial pose estimates to simulate noise
        perturbed_pose = poses[i].compose(gtsam.Pose3(gtsam.Rot3.Rodrigues(0.1,0.1,0.1), gtsam.Point3(0.1,0.2,0.3)))
        initial_estimate.insert(X(i), perturbed_pose)

    # add quadric estimate
    for i, quadric in enumerate(quadrics):
        quadric.addToValues(initial_estimate, Q(i))

    # add relative poses to graph as odometry
    for i in range(len(poses)-1):
        relative_pose = poses[i].between(poses[i+1])
        odometry_factor = gtsam.BetweenFactorPose3(X(i), X(i+1), relative_pose, odometry_noise)
        graph.add(odometry_factor)

    # reproject true quadrics into each true pose
    for j, quadric in enumerate(quadrics):
        for i, pose in enumerate(poses):
            conic = gtsam_quadrics.QuadricCamera.project(quadric, pose, calibration)
            bounds = conic.bounds()
            bbf = gtsam_quadrics.BoundingBoxFactor(bounds, calibration, X(i), Q(j), bbox_noise)
            graph.add(bbf)

    # define lm parameters
    parameters = gtsam.LevenbergMarquardtParams()
    parameters.setVerbosityLM("SUMMARY") # SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA : VALUES, ERROR 
    parameters.setMaxIterations(100)
    parameters.setlambdaInitial(1e-5)
    parameters.setlambdaUpperBound(1e10)
    parameters.setlambdaLowerBound(1e-8)
    parameters.setRelativeErrorTol(1e-5)
    parameters.setAbsoluteErrorTol(1e-5)

    # create optimizer
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, parameters)

    # run optimizer
    result = optimizer.optimize()
