
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
from visualization.drawing import CV2Drawing
from base.containers import Trajectory, Quadrics, Detections, ObjectDetection

from dataset_interfaces.scenenet_dataset import SceneNetDataset
from dataset_interfaces.tumrgbd_dataset import TUMDataset
from detectors.faster_rcnn import FasterRCNN

# import gtsam and extension
import gtsam
import gtsam_quadrics

if __name__ == '__main__':

    # load dataset
    dataset = TUMDataset('/media/lachness/DATA/Datasets/TUM/')

    for scene in dataset:

        for image in scene.rgb_images:
            print(type(image))