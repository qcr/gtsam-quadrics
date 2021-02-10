"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Dataset interface 
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import cv2
import sys
import math
import importlib
import numpy as np
from PIL import Image
from abc import ABC, abstractmethod

# import gtsam and extension
import gtsam
import gtsam_quadrics

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

# import custom python modules
sys.dont_write_bytecode = True
from base.containers import ObjectDetection
from base.containers import Detections
from base.containers import Trajectory
from base.containers import Odometry
from base.containers import Quadrics
from visualization.drawing import CV2Drawing
from visualization.interactive_player import InteractivePlayer

class Dataset(ABC):
    """
    This class loads the dataset and iterably returns scenes.
    """
    @abstractmethod
    def __len__(self):
        pass

    @abstractmethod
    def __getitem__(self, index):
        pass


class Scene(object):
    """
    This class contains all the information for a single video/scene. 
    """

    def __init__(self, **kwargs): 
        # self._valid_fields = ['trajectory', 'images', 'detections', 'data_association', 'map']
        self._fields = {}
        for k, v in kwargs.items():
            self.set(k, v)

    def __setattr__(self, name, value):
        if name.startswith('_'):
            super().__setattr__(name, value)
        else:
            self.set(name, value)

    def __getattr__(self, name):
        return self._fields[name]

    def set(self, name, value):
        self._fields[name] = value

    def has(self, name):
        return name in self._fields

    def remove(self, name):
        del self._fields[name]

    def __len__(self):
        raise NotImplementedError('Scene does not support __len__')


class SmartImages(object):
    """ 
    Acts similarly to a list of cv2 images 
    With the advantage that images are only loaded when accessed. 
    """
    def __init__(self, image_paths):
        self.image_paths = image_paths
        
    def __getitem__(self, index):
        image_path = self.image_paths[index]
        image = cv2.imread(image_path)
        return image

    def __len__(self):
        return len(self.image_paths)

    