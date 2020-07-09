"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Trajectory, Quadrics, Boxes, Odometry containers
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')
import numpy as np
from PIL import Image
import cv2
import torch
from torch.utils.data import DataLoader
from torchvision import datasets
from torchvision import transforms
from torch.autograd import Variable
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import NullLocator

# import custom modules
sys.dont_write_bytecode = True
import importlib
from models import *
from utils.utils import *
from utils.datasets import *




class Detector(object):
    def __init__(self, weights_path, config_path, classes_path):
        self.weights_path = weights_path
        self.config_path = config_path
        self.classes_path = classes_path
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
            detections = self.full_non_max_suppression(outputs, self.conf_thres, self.nms_thres)

            # nms returns list [torch(n,85)]*N where N is images and n is detections
            detections = detections[0]

            # nms returns None for each image if no detections
            if detections is None:
                return np.array([])

            # rescale to original image
            detections = rescale_boxes(detections, self.img_size, image_shape[:2])

        # detatch detections
        detections = detections.cpu().numpy()
        
        return detections


    def full_non_max_suppression(self, prediction, conf_thres=0.5, nms_thres=0.4, scores=False):
        """
        We have enriched PyTorch-YOLOv3 to perform non maximum supression and store
        the full class scores, instead of just the winning class probability. 

        Removes detections with lower object confidence score than 'conf_thres' and performs
        Non-Maximum Suppression to further filter detections.
        
        Inputs: prediction [N_images, 10657, 85] tensor 
        Returns: detections with shape [N_images, N_detections, 85]:
        Each 85 long detection is stored as: (x1, y1, x2, y2, object_conf, 80_class_scores)
        """

        # From (center x, center y, width, height) to (x1, y1, x2, y2)
        prediction[..., :4] = xywh2xyxy(prediction[..., :4])

        output = [None for _ in range(len(prediction))]
        for image_i, image_pred in enumerate(prediction):

            # append index into prediction
            image_pred = torch.cat((image_pred, torch.arange(len(image_pred)).unsqueeze(1).float()), 1)

            # convert local image_pred to probabilities
            if (scores):
                image_pred[:, 4:-1] = torch.sigmoid(image_pred[:, 4:-1])

            # Filter out confidence scores below threshold
            image_pred = image_pred[image_pred[:, 4] >= conf_thres]

            # If none are remaining => process next image
            if not image_pred.size(0):
                continue

            # Object confidence times class confidence
            score = image_pred[:, 4] * image_pred[:, 5:-1].max(1)[0]

            # Sort by it
            image_pred = image_pred[(-score).argsort()]
            class_confs, class_preds = image_pred[:, 5:-1].max(1, keepdim=True)

            # added index into image predictions
            detections = torch.cat((image_pred[:, :5], class_confs.float(), class_preds.float(), image_pred[:, -1:]), 1)

            # get only the detections where boxes are valid
            detections = detections[((detections[:,:4]<1e10).sum(1)>=4), :]
            # print(detections.shape)

            # Perform non-maximum suppression
            indicies_per_observation = []
            while detections.size(0):
                ious = bbox_iou(detections[0, :4].unsqueeze(0), detections[:, :4])
                # if (torch.isclose(ious[0], torch.FloatTensor([0])) or torch.isnan(ious).any() or torch.isinf(ious).any()):
                # 	print('WARNING, inf IOU encountered, skipping\n')#, detections[:,:4])
                # 	detections = detections[1:]
                # 	continue
                large_overlap = ious > nms_thres
                label_match = detections[0, -2] == detections[:, -2]

                # Indices of boxes with lower confidence scores, large IOUs and matching labels
                invalid = large_overlap & label_match

                # store indicies of overlapping boxes
                indicies_per_observation += [detections[invalid,-1]]
                detections = detections[~invalid]

            # merge overlapping boxes
            keep_boxes = []
            for indicies in indicies_per_observation:
                _detections = prediction[image_i, indicies.long(), :]

                weights = _detections[:,4:5]
                if (scores):
                    weights = torch.sigmoid(weights)

                _detection = _detections[0]
                _detection[0:4] = (weights * _detections[:, 0:4]).sum(0) / weights.sum()
                keep_boxes.append(_detection)

            if keep_boxes:
                output[image_i] = torch.stack(keep_boxes)

        return output

