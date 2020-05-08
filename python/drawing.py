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

sys.dont_write_bytecode = True

class Drawing(object):

    @staticmethod
    # TODO: draw camera fov
    # TODO: project quadric shape orthographically
    def draw_problem(graph, estimate):

        graph_keys = [graph.keys().at(i) for i in range(graph.keys().size())]
        estimate_keys = [estimate.keys().at(i) for i in range(estimate.keys().size())]

        # draw trajectory variables
        # for key in graph_keys:


        # draw quadric variables

        # draw odometry factors

        # draw bbox factors
