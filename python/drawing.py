"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Drawing interface
Author: Lachlan Nicholson (Python)
"""

import sys
import cv2
import numpy as np
from containers import Trajectory
from containers import Odometry
from containers import Quadrics
from containers import Boxes

import matplotlib.pyplot as plt

sys.dont_write_bytecode = True

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
