"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Drawing interface
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt

# import gtsam and extension
import gtsam

# import custom python modules
sys.dont_write_bytecode = True
from containers import Trajectory
from containers import Odometry
from containers import Quadrics
from containers import Boxes

symbolChr = lambda i: chr(gtsam.symbolChr(i))

class Drawing(object):

    @staticmethod
    # TODO: draw camera fov
    # TODO: project quadric shape orthographically
    def draw_problem(graph, estimate):
        """ assumes z axis is up """

        estimated_trajectory = Trajectory.from_values(estimate)
        estimated_quadrics = Quadrics.from_values(estimate)

        for pose_key, pose in estimated_trajectory.items():
            plt.plot(pose.x(), pose.y(), marker='o', markersize=3, color='c')
            plt.text(pose.x(), pose.y(), 'x{}'.format(pose_key))

        for object_key, quadric in estimated_quadrics.items():
            plt.plot(quadric.getPose().x(), quadric.getPose().y(), marker='o', markersize=3, color='m')
            plt.text(quadric.getPose().x(), quadric.getPose().y(), 'q{}'.format(object_key))

        plt.show()

        # for quadric in 


        # graph_keys = [graph.keys().at(i) for i in range(graph.keys().size())]
        # draw quadric variables

        # draw odometry factors

        # draw bbox factors

    @staticmethod
    def draw_results(trajectories, quadrics, colors):
        for i, trajectory in enumerate(trajectories):
            xy = np.array([[pose.x(), pose.y()] for pose in trajectory.data()])
            plt.plot(xy[:,0], xy[:,1], marker='o', markersize=3, c=colors[i], label='traj {}'.format(i))
        
        for i, _quadrics in enumerate(quadrics):
            for quadric in _quadrics.data():
                plt.plot(quadric.getPose().x(), quadric.getPose().y(), marker='o', markersize=3, c=colors[i], label='quads {}'.format(i))

        plt.show()