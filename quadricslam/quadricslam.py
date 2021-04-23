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




class System(object):
    def __init__(self,
                 calibration,
                 optimizer, # how to handle lvm/gn/isam? 
                 initializer,
                 measurement_mode,
                 prior_noise,
                 odom_noise,
                 box_noise,
                 online = True,
                 ):
        pass

    def update()



# Initializers: SVD, SIMPLE, DEPTH, TRUE



class Initializer(ABC):
    def __init__(self):
        pass

    def initialize_quadric(self, poses, boxes):
        pass

class Associator(ABC):
    def __init__(self, calibration):
        pass

    def associate(self, pose, boxes):
        pass


class QuadricSLAM(object):
    def __init__(self, settings, initializer, associator):
        pass

    def update(self, odom, associated_boxes):
        """
        Updates graph/values with new measurements.
        Stores boxes for later use
        Initializes new landmarks if valid
        """
        pass