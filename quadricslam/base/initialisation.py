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

# import custom python modules
sys.dont_write_bytecode = True
from base.data_association import DataAssociation
from dataset_interfaces.scenenet_dataset import SceneNetDataset
from visualization.drawing import CV2Drawing
from base.containers import Trajectory, Quadrics, Detections, ObjectDetection

# import gtsam and extension
import gtsam
import gtsam_quadrics

class Initialize(object):

    @staticmethod
    def simple(pose, box, calibration, depth=1.0, size=0.01):
        """
        Create quadric @ depth along ray of bbox centre
        """
        center = box.center()
        x = (center.x() - calibration.px()) * depth / calibration.fx()
        y = (center.y() - calibration.py()) * depth / calibration.fy()
        relative_point = gtsam.Point3(x,y,depth)
        global_pose = pose.compose(gtsam.Pose3(gtsam.Rot3(), relative_point))
        quadric = gtsam_quadrics.ConstrainedDualQuadric(global_pose, np.array([size]*3))
        return quadric

    @staticmethod
    def SVD(poses, boxes, calibration):
        """ 
        poses = [gtsam.Pose3]
        boxes = [gtsam_quadrics.AlignedBox3]
        returns: quadric as a 4x4 matrix
        """

        # iterate through box/pose data
        planes = []
        for box, pose in zip(boxes, poses):

            # calculate boxes lines
            lines = box.lines()

            # convert Vector3Vector to list
            lines = [lines.at(i) for i in range(lines.size())]

            # calculate projection matrix
            P = gtsam_quadrics.QuadricCamera.transformToImage(pose, calibration).transpose()

            # project lines to planes
            planes += [P @ line for line in lines]

        # create A matrix
        A = np.asarray([np.array([p[0]**2,  2*(p[0]*p[1]),  2*(p[0]*p[2]),  2*(p[0]*p[3]),
                                                p[1]**2,  	2*(p[1]*p[2]),  2*(p[1]*p[3]),
                                                                p[2]**2,  	2*(p[2]*p[3]),
                                                                               p[3]**2]) for p in planes])

        # solve SVD for Aq = 0, which should be equal to p'Qp = 0
        _,_,V = np.linalg.svd(A, full_matrices=True)
        q = V.T[:, -1]

        # construct quadric
        quadric_matrix = np.array([[q[0], q[1], q[2], q[3]],
                                [q[1], q[4], q[5], q[6]],
                                [q[2], q[5], q[7], q[8]],
                                [q[3], q[6], q[8], q[9]]])

        # constrain quadric matrix -> ellipsoid
        quadric = gtsam_quadrics.ConstrainedDualQuadric.constrain(quadric_matrix)
        
        # check if valid
        if not Initialize.is_okay(quadric, poses, calibration):
            return None
        return quadric

    @staticmethod
    def is_okay(quadric, poses, calibration):
        """
        Checks quadric is valid:
            quadric constrained correctly
            paralax > threshold
            reprojections valid in each frame 
                quadric infront of camera : positive depth 
                camera outside quadric
                conic is an ellipse 
            ensure views provide enough DOF (due to edges / out of frame)
        """
        for pose in poses:

            # quadric must have positive depth
            if quadric.isBehind(pose):
                return False

            # camera pose must be outside quadric 
            if quadric.contains(pose):
                return False

            # conic must be valid and elliptical 
            conic = gtsam_quadrics.QuadricCamera.project(quadric, pose, calibration)
            if conic.isDegenerate():
                return False
            if not conic.isEllipse():
                return False
                
        return True
