
import os
import sys
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')
sys.path.append('/home/lachness/git_ws/quadricslam/ros/src/py_detector/py_detector')

import time
import datetime
import argparse

from models import *
from utils.utils import *
from utils.datasets import *

from PIL import Image

import torch
from torch.utils.data import DataLoader
from torchvision import datasets
from torchvision import transforms
from torch.autograd import Variable

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import NullLocator

import cv2

import numpy as np
import torch
from torchvision import transforms
import torch.nn.functional as F


class Detector(object):
    def __init__(self):
        self.code_path = '/home/lachness/git_ws/quadricslam/ros/src/py_detector/py_detector'
        self.weights_path = self.code_path + '/weights/yolov3.weights'
        self.config_path = self.code_path + '/config/yolov3.cfg'
        self.classes_path = self.code_path + '/data/coco.names'
        self.conf_thres = 0.8
        self.nms_thres = 0.4
        self.img_size = 416

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = Darknet(self.config_path, img_size=self.img_size).to(self.device)
        self.model.load_darknet_weights(self.weights_path)
        self.model.eval()  # Set in evaluation mode
        self.classes = load_classes(self.classes_path)
        self.Tensor = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor

    def forward(self, image):
        """
        image: cv2 format, i.e, HxWxC 0-255 BGR format. 
        Returns Nx85 where each row has: x1,y1,x2,y2,objectness,scores[80]
        """

        # record original image shape
        image_shape = image.shape

        # convert HWC|0-255|BGR into HWC|0-255|RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # convert to CHW|0-1|RGB
        image = transforms.ToTensor()(image)

        # pad to square and resize
        image, _ = pad_to_square(image, 0)
        image = resize(image, self.img_size)

        # add first dimension NCHW
        input = image.unsqueeze(0)

        # convert to gpu 
        input = Variable(input.type(self.Tensor))

        # Get detections
        with torch.no_grad():
            outputs = self.model(input)

            # use a modified nms that keeps class scores
            detections = full_non_max_suppression(outputs, self.conf_thres, self.nms_thres)

            # nms returns list [torch(n,7)]*N where N is images and n is detections
            detections = detections[0]

            # nms returns None for each image if no detections
            if detections is None:
                return None

            # rescale to original image
            detections = rescale_boxes(detections, self.img_size, image_shape[:2])

        return detections.cpu().numpy()
