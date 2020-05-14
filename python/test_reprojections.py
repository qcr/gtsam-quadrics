"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Runs the system through datasets and investigates how often reprojection fails
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# import gtsam and extension
import gtsam
import quadricslam

# import custom python modules
sys.dont_write_bytecode = True
from simulated_dataset import ManualSequence
from scenenet_dataset import SceneNetDataset
from containers import *
from drawing import Drawing
from system import System


def draw_factors(graph, estimate, calibration):

    # get variables and factors
    trajectory = Trajectory.from_values(estimate)
    quadrics = Quadrics.from_values(estimate)
    box_factors = [quadricslam.BoundingBoxFactor.getFromGraph(graph, i) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
    odom_factors = [graph.at(i) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'x']

    # make figure
    fig = plt.figure('Trajectory')
    fig.clf(); ax = fig.gca()
    
    # draw trajectory
    xy = np.array([[pose.x(), pose.y()] for pose in trajectory.data()])
    plt.plot(xy[:,0], xy[:,1], linestyle='-', c='c', zorder=1)
    plt.scatter(xy[:,0], xy[:,1], s=5, c='b', zorder=9)
    
    # draw quadrics
    for quadric in quadrics.data():
        # conic = quadric.matrix()[[0,1,3],:][:,[0,1,3]]
        # Drawing.mpl_draw_conic(conic)
        # bounds = quadric.bounds()
        # xyxy = quadricslam.DualConic(conic).bounds().vector()
        # rect = patches.Rectangle(xyxy[0:2],xyxy[2],xyxy[3],linewidth=1,edgecolor='g',facecolor='none')
        # ax.add_patch(rect)

        plt.plot(quadric.getPose().x(), quadric.getPose().y(), marker='o', markersize=5, c='m', fillstyle='none')
        plt.plot(quadric.getPose().x(), quadric.getPose().y(), marker='o', markersize=5, c='m', alpha=0.5)


    # draw a bar graph of factor error and color factor type
    # GREEN: good
    # RED: quad behind camera
    # ORANGE: camera inside quadric

    errors = []
    classifications = []

    for factor in box_factors:

        ## NOTE: using bbf to evaluate error will not hit reprojection errors

        pose_key = gtsam.symbolIndex(factor.poseKey())
        object_key = gtsam.symbolIndex(factor.objectKey())
        pose = trajectory.at(pose_key)
        quadric = quadrics.at(object_key)
        conic = quadricslam.QuadricCamera.project(quadric, pose, calibration)
        bounds = conic.bounds()
        measured_box = factor.measurement()
        error = (measured_box.vector() - bounds.vector())
        # error = factor.unwhitenedError(estimate)
        errors.append( np.square(error).sum() )

        isBehind = quadric.isBehind(pose)
        isInside = quadric.contains(pose)
        isEllipse = conic.isEllipse()
        classifications.append([isBehind, isInside, isEllipse])

    c = np.array(classifications)
    print('isBehind: {}/{}'.format(c[:,0].sum(), len(c)))
    print('isInside: {}/{}'.format(c[:,1].sum(), len(c)))
    print('notElips: {}/{}'.format(np.logical_not(c[:,2]).sum(), len(c)))
    print('')

    # fig = plt.figure("factors")
    # ax = plt.subplot(3,1,1)
    # plt.bar(range(len(box_factors)), errors, color= ['green' if not c[0] else 'red' for c in classifications])
    # ax = plt.subplot(3,1,2)
    # plt.bar(range(len(box_factors)), errors, color= ['green' if not c[1] else 'red' for c in classifications])
    # ax = plt.subplot(3,1,3)
    # plt.bar(range(len(box_factors)), errors, color= ['green' if c[0] else 'red' for c in classifications])
    # plt.scatter([0]*len(box_factors), errors, c=['green' if not c[0] else 'red' for c in classifications])
    # plt.scatter([1]*len(box_factors), errors, c=['green' if not c[1] else 'red' for c in classifications])
    # plt.scatter([2]*len(box_factors), errors, c=['green' if c[2] else 'red' for c in classifications])
    # plt.yscale('log')
    plt.show()

        


def test_sequence(sequence):

    # build initial estimate + graph
    graph, initial_estimate = System.build_graph(sequence)

    # check system makes sense
    System.check_problem(graph, initial_estimate)

    # create optimizer parameters
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")    # SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA : VALUES, ERROR 
    params.setMaxIterations(100)
    params.setlambdaInitial(1e-5)       # defaults to 1e5
    params.setlambdaUpperBound(1e10)     # defaults to 1e5
    params.setlambdaLowerBound(1e-8)    # defaults to 0.0
    params.setRelativeErrorTol(1e-10)   # stop iterating when change in error between steps is less than this
    params.setAbsoluteErrorTol(1e-8)    # stop when cost-costchange < tol

    # create optimizer
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)


    # plot initial estimate
    draw_factors(graph, initial_estimate, sequence.calibration)

    # iterate manually 
    for i in range(100):

        # take a step
        optimizer.iterate()

        # get the estimate
        estimate = optimizer.values()

        # draw current iteration
        draw_factors(graph, estimate, sequence.calibration)




if __name__ == '__main__':
    np.random.seed(121)

    trainval = 'train'
    dataset_path = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/data/{}'.format(trainval)
    protobuf_folder = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/data/{}_protobufs'.format(trainval)
    reader_path = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/scenenet_pb2.py'
    dataset = SceneNetDataset(dataset_path, protobuf_folder, reader_path)
    test_sequence(dataset[0])

    # sequence = ManualSequence.sequence1()
    # System.run(sequence)

