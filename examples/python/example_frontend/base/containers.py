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
import numpy as np
from collections import defaultdict

# import gtsam and extension
import gtsam
import quadricslam

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))








class Container1(dict):
    """
    Container1 is an un-ordered single-key dict.
    You cannot store two values at the same key.
    """
    def add(self, value, key):
        self[key] = value
    
    def keys(self):
        return list(super().keys())

    def values(self):
        return list(super().values())

    def at(self, key):
        return self[key]

    def at_keys(self, keys):
        return [self.at(key) for key in keys]


class Trajectory(Container1):
    def as_odometry(self):
        """ Converts the trajectory to a sequence of relative poses """
        odometry = Odometry()

        pose_keys = self.keys()
        poses = self.values()
        for i in range(len(self)-1):
            start_key = pose_keys[i]
            end_key = pose_keys[i+1]
            rpose = poses.at(start_key).between(poses.at(end_key))
            odometry.add(rpose, start_key, end_key)
            
        return odometry

    def applyTransform(self, reference):
        """ Transforms trajectory from local coordinates to reference coordinates """
        transformed_trajectory = Trajectory()
        for key, pose in self.items():
            transformed_trajectory.add(reference.transformPoseFrom(pose), key)
        return transformed_trajectory

    @staticmethod
    def from_values(values):
        trajectory = Trajectory()
        for i in range(values.keys().size()):
            key = values.keys().at(i)
            if chr(gtsam.symbolChr(key)) == 'x':
                trajectory.add(values.atPose3(key), gtsam.symbolIndex(key))

        return trajectory


class Quadrics(Container1):
    @staticmethod
    def from_values(values):
        quadrics = Quadrics()
        for i in range(values.keys().size()):
            key = values.keys().at(i)
            if chr(gtsam.symbolChr(key)) == 'q':
                quadric = quadricslam.ConstrainedDualQuadric.getFromValues(values, key)
                quadrics.add(quadric, gtsam.symbolIndex(key))
        return quadrics



class ObjectDetection(object):
    def __init__(self, box, objectness, scores):
        self.box = box
        self.objectness = objectness
        self.scores = scores




class Container2_Map(object):
    """
    Container2_Map acts as an 2-key map.
    """
    def __init__(self):
        self.key1_map = defaultdict(dict)
        self.key2_map = defaultdict(dict)

    def __len__(self):
        len(self.key1_map)

    def add(self, value, key1, key2):
        self.key1_map[key1][key2] = value
        self.key2_map[key2][key1] = value

    def keys(self):
        """ returns [(key1, key2)]*len(values) """
        return [(k1,k2) for k1,k2map in self.key1_map.items() for k2 in k2map.keys()]

    def key1s(self):
        """ returns [key1]*len(values) """
        return [keypair[0] for keypair in self.keys()]

    def key2s(self):
        """ returns [key1]*len(values) """
        return [keypair[1] for keypair in self.keys()]

    def ukey1s(self):
        return list(self.key1_map.keys())

    def ukey2s(self):
        return list(self.key2_map.keys())

    def at(self, key1, key2):
        """ returns value @ key1,key2 """
        return self.key1_map[key1][key2]

    def at_key1(self, key1):
        """ returns {key2: value} @ key1 """
        return self.key1_map[key1]

    def at_key2(self, key2):
        """ returns {key1: value} @ key2 """
        return self.key2_map[key2]

    def per_key1(self):
        """ [key1, {key2: value}]  """ 
        return self.key1_map.items()

    def per_key2(self):
        """ [key2, {key1: value}]  """ 
        return self.key2_map.items()

    def values(self):
        """ [value] """
        return (v for k2map in self.key1_map.values() for v in k2map.values())

    def items(self):
        """ [(key1,key2),value] """
        return (((k1,k2),v) for k1,k2map in self.key1_map.items() for k2,v in k2map.items())
    

class Detections(Container2_Map):
    def __init__(self):
        super().__init__()
        self.used = dict()

    def add(self, detection, pose_key, object_key):
        super().add(detection, pose_key, object_key)
        self.used[pose_key, object_key] = False

    def add_detections(self, detections):
        for (key1, key2), value in detections.items():
            self.add(value, key1, key2)

    def pose_keys(self):
        return self.key1s()

    def object_keys(self):
        return self.key2s()

    def at_pose(self, pose_key):
        return self.at_key1(pose_key)

    def at_object(self, object_key):
        return self.at_key2(object_key)

    def per_pose(self):
        # return ((pose_key, object_map.values()) for pose_key,object_map in self.key1_map.items())
        return self.per_key1()

    def per_object(self):
        # return ((object_key, pose_map.values()) for object_key,pose_map in self.key2_map.items())
        return self.per_key2()

    def is_used(self, pose_key, object_key):
        return self.used[pose_key, object_key]

    def set_used(self, value, pose_key, object_key):
        self.used[pose_key, object_key] = value

    def add_noise(self, mu, sd):
        """ 
        compose noisevec onto relative poses 
        NOTE: assumes type(value) == AlignedBox2
        """
        noisy_boxes = Detections()
        for (pose_key, object_key), box in self.items():

            # get normally distributed doubles 
            noise_vector = np.random.normal(mu, sd, 4)

            # construct noisey perturbation
            noisy_box = quadricslam.AlignedBox2(box.vector() + noise_vector)

            # add box to collection
            noisy_boxes.add(noisy_box, pose_key, object_key)
        return noisy_boxes

    def add_noise_strict(self, mu, sd, image_dimensions):
        """ Take care to ensure boxes have positive width/height
        And they do not extend past the image dimensions 
        NOTE: assumes type(value) == AlignedBox2
        """
        image_box = quadricslam.AlignedBox2(0,0,image_dimensions[0],image_dimensions[1])
        noisy_boxes = Detections()
        for (pose_key, object_key), box in self.items():

            # get normally distributed doubles 
            noise_vector = np.random.normal(mu, sd, 4)

            # construct noisey perturbation
            nbox = quadricslam.AlignedBox2(box.vector() + noise_vector)

            # ensure nbox right way around
            if nbox.width() < 0:
                nbox = quadricslam.AlignedBox2(nbox.xmax(), nbox.ymin(), nbox.xmin(), nbox.ymax())
            if nbox.height() < 0:
                nbox = quadricslam.AlignedBox2(nbox.xmin(), nbox.ymax(), nbox.xmax(), nbox.ymin())

            # make sure box is inside image dimensions
            if nbox.intersects(image_box):
                nbox = nbox.vector()
                nbox[[0,2]] = np.clip(nbox[[0,2]], image_box.xmin(), image_box.xmax())
                nbox[[1,3]] = np.clip(nbox[[1,3]], image_box.ymin(), image_box.ymax())
                nbox = quadricslam.AlignedBox2(nbox)

            # make sure nbox has width/height
            if nbox.width() >= 0.0 and nbox.width() < 1.0:
                continue
            if nbox.height() >= 0.0 and nbox.height() < 1.0:
                continue


            # add nbox to collection
            noisy_boxes.add(nbox, pose_key, object_key)
        return noisy_boxes


class Odometry(Container2_Map):
    def as_trajectory(self, reference=gtsam.Pose3()):
        """ Converts relative poses to global trajectory 
        reference = first pose of global trajectory
        NOTE: assumes odometry keys are ascending 
        """
        trajectory = Trajectory()
        for (start_key, end_key), rpose in self.items():
            if len(trajectory) == 0:
                trajectory.add(reference, start_key)
            trajectory.add( trajectory.values()[-1].compose(rpose), end_key )
        return trajectory

    def add_noise(self, mu, sd):
        """ 
        compose Pose3::retract(noisevec) onto relative poses 
        NOTE: assumes type(value) is gtsam.Pose3
        """
        noisy_rposes = Odometry()
        for (start_key, end_key), rpose in self.items():

            # get normally distributed doubles 
            noise_vector = np.random.normal(mu, sd, 6)

            # construct noisey perturbation
            delta = gtsam.Pose3.retract(gtsam.Pose3(), noise_vector)

            # compose the noise
            noisy_rposes.add(rpose.compose(delta), start_key, end_key)
        return noisy_rposes

        