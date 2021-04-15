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


# developer interface:
# * create qslam instance
# * specify odometry/detections/da/init sources
# * update with new info
# * optimize when needed






# NOTE: offline system keeps consistent .graph / .estimate
# NOTE: online system clears .graph / .estimate each .optimize() call
# because we only add local->global @optimize, offline system wont error about 
#   duplicate keys until you optimize
# tf. offline: graph/estimate = current problem | .local = changes since last update
# tf. online: graph is unused. estimate = last estimate | .local = changes



# PROBLEMS:
# exposing .add_pose, etc, as the only interface requires the user to keep track of pose/quadric keys
# if we add quick methods to calculate keys for them, it's innefecient 
# only effecient way is to keep a history
# How can we provide both a low-level and high-level interface
# How do we know who should store the histories? (previous pose/key, detections, used quads, used detections)




# define gtsam macros 
X = lambda i: int(gtsam.symbol(ord('x'), i))
Q = lambda i: int(gtsam.symbol(ord('q'), i))

class QuadricSLAM(object):
 
    def __init__(self, settings, calibration):
        self.settings = settings
        self.calibration = calibration

        self.optimizer = self.create_optimizer(settings.optimizer)
        self.graph = gtsam.NonlinearFactorGraph()
        self.estimate = gtsam.Values() 

        # empty if offline
        self.local_graph = gtsam.NonlinearFactorGraph()
        self.local_estimate = gtsam.Values() 

        self.trajectory = Trajectory() 
        self.quadrics = Quadrics()

        # record measurements for DA
        # macros
        # noise models
        self.odom_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([settings['QuadricSLAM.odom_sd']]*6, dtype=np.float))
        self.box_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([settings['QuadricSLAM.box_sd']]*4, dtype=np.float))
        self.measurement_model = settings['QuadricSLAM.measurement_model']


    def _get_largest_pose_key(self):
        existing_keys = pose_keys_from_values(self.estimate) + pose_keys_from_values(self.local_estimate)
        return np.max(existing_keys)

    def _get_previous_pose(self):
        previous_key = self._get_largest_pose_key()
        if self.estimate.exists(previous_key):
            previous_pose = self.estimate.atPose3(X(previous_key))
        else:
            previous_pose = self.local_estimate.atPose3(X(previous_key))
        return previous_pose

 
    def add_pose(self, 
                 pose: gtsam.Pose3, 
                 rpose: bool = False,
                 key: int = None
                 ) -> int:
        """Adds pose to values. Uses next sequential key if none provided.
        If a rpose is True, pose is treated as a relative pose (odometry). 
        
        Converts to global pose by composing relative pose onto last pose.
        The most recent pose is assumed to have the largest key.
        """
        if key is None:
            key = self._get_largest_pose_key() + 1
        if rpose:
            previous_pose = self._get_previous_pose()
            pose = previous_pose.compose(pose)
            
        self.local_estimate.insert(X(key), pose)
        return key
 
    def add_odometry(self, 
                     odom: gtsam.Pose3, 
                     rpose: bool = True,
                     start_key: int = None,
                     end_key: int = None,
                     ) -> tuple[int, int]:
        """Adds odometry to graph as BetweenFactor. 
        Extends previous pose if no keys provided."""
        # NOTE: should they be able to specify custom noise?

        if start_key is None and end_key is None:
            previous_key = self._get_largest_pose_key()
            start_key = previous_key
            end_key = start_key + 1

        # convert global to rpose using last pose
        if not rpose:
            previous_pose = self._get_previous_pose()
            odom = previous_pose.between(odom)

        odom_factor = gtsam.BetweenFactorPose3(X(start_key), X(end_key), odom, self.odom_noise)
        self.local_graph.add(odom_factor)
        return (start_key, end_key)

 
    def add_quadric(self,
                    quadric: gtsam_quadrics.ConstrainedDualQuadric,
                    key: int,
                    ) -> int:
        """Adds quadric to graph. Ensure quadric is constrained (more than 3 views)."""
        quadric.addToValues(self.local_estimate, Q(key))
 
    def add_detection(self,
                      box: gtsam_quadrics.AlignedBox2,
                      pose_key: int,
                      object_key: int
                      ):
        """Adds box to graph as BoundingBoxFactor. Ensure quadric is constrained, initialized, and measurement is new."""
        # NOTE: should they be able to use custom MM here?
        bbf = gtsam_quadrics.BoundingBoxFactor(box, self.calibration, X(pose_key), Q(object_key), self.box_noise, self.measurement_model)
        self.local_graph.add(bbf)

 
    def optimize(self) -> tuple[dict[int, gtsam.Pose3], dict[int, gtsam_quadrics.ConstrainedDualQuadric]]:
        """Optimize the current graph.
        If system.offline, performs full bundle adjustment.
        If system.online, updates isam2 with the factors and values since last call
        Returns Trajectory, Map.
        """

        self.graph.push_back(self.local_graph)
        self.estimate.insert(self.local_estimate)

        # NOTE: can create more cohesian between offline/online
        # by converting opt process to func(graph, values)
        if settings.offline:
            optimizer = self.create_optimizer(self.graph, self.estimate, settings, opt_params)
            estimate = optimizer.optimize()
        if settings.online:
            self.isam.update(self.local_graph, self.local_estimate)
            estimate = self.isam.calculateEstimate()

        self.local_graph.resize(0)
        self.local_estimate.clear()

        self.current_trajectory = Trajectory.from_values(estimate)
        self.current_quadrics = Quadrics.from_values(estimate)

 
 
    # -------------------------------------------------------------------
    # Fast interfaces (useful when using our own odom/detector/DA/init) 
    # -------------------------------------------------------------------
 
    def update(self, pose, detections): 
        """Update the graph/values with all single-view measurements. 
        Essentially performs DA/init then calls .add_pose(), .add_odom(), .add_quad(), .add_detection()
        """

        
 
    def batch_update(self, trajectory, pose_stamped_detections):
        """Updates the graph/values for multi-views."""
 
    def full_update(self, image): 
        """Calculates odometry, detections, DA, init, then builds graph."""
 


# utilities
def cvec_to_list(cvec):
    return [keys.at(i) for i in range(keys.size())]

def pose_keys_from_values(values):
    keys = cvec_to_list(values.keys()) # mangled
    pose_keys = [gtsam.SymbolIndex(key) for key in keys if gtsam.symbolChr(key) == 'x']
    return pose_keys
    
