"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Trajectory, Quadrics, Boxes, Odometry containers
Author: Lachlan Nicholson (Python)
"""

import sys
import gtsam
from collections import defaultdict
import numpy as np
import quadricslam

sys.dont_write_bytecode = True

X = lambda i: int(gtsam.symbol(ord('x'), i))
Q = lambda i: int(gtsam.symbol(ord('q'), i))

class Trajectory(object):
    """ 
    Implicit data association. 
    Keys stored in index of pose.
    Cast to gtsamKey when converted to factor.
    """
    def __init__(self, poses):
        self._poses = poses

    def __len__(self):
        return len(self._poses)

    def __getitem__(self, index):
        """ return in order with images[index] """
        return self._poses[index]

    def items(self):
        """ [(key, pose)] """
        return zip(range(len(self._poses)), self._poses)

    def data(self):
        """ returns list of poses """
        return self._poses

    def at_keys(self, keys):
        """ returns a new trajectory only with these keys """
        return Trajectory([self._poses[key] for key in keys])

    def as_odometry(self):
        relative_poses = [self._poses[i].between(self._poses[i+1]) for i in range(len(self._poses)-1)]
        return Odometry(relative_poses)

    def add_prior(self, graph, noisemodel):
        """ add prior X(0) to graph """
        prior_factor = gtsam.PriorFactorPose3(X(0), self._poses[0], noisemodel)
        graph.add(prior_factor)

    def add_estimates(self, values):
        """ add poses X(i) to values """
        for index, pose in enumerate(self._poses):
            values.insert(X(index), pose)

    @staticmethod
    def from_values(values):
        estimate_keys = [values.keys().at(i) for i in range(values.keys().size())]
        pose_keys = [k for k in estimate_keys if chr(gtsam.symbolChr(k)) == 'x']
        poses = [values.atPose3(k) for k in pose_keys]
        return Trajectory(poses)


            

            


class Odometry(object):
    """ 
    Implicit data association. 
    Keys stored in index of rpose.
    Cast to gtsamKey when converted to factor.
    """
    def __init__(self, rposes):
        self._rposes = rposes

    def __len__(self):
        return len(self._rposes)

    def __getitem__(self, index):
        return self._rposes[index]

    def data(self):
        return self._rposes

    def as_trajectory(self, reference=gtsam.Pose3()):
        global_poses = [reference]
        for rpose in self._rposes:
            global_poses.append( global_poses[-1].compose(rpose) )
        return Trajectory(global_poses)

    def add_factors(self, graph, noisemodel):
        """ add odom factors X(i) -> X(i+1) to graph """
        for index, rpose in enumerate(self._rposes):
            odometry_factor = gtsam.BetweenFactorPose3(X(index), X(index+1), rpose, noisemodel)
            graph.add(odometry_factor)

    def add_noise(self, mu, sd):
        """ compose Pose3::retract(noisevec) onto relative poses """
        noisy_rposes = []
        for rpose in self._rposes:

            # get normally distributed doubles 
            noise_vector = np.random.normal(mu, sd, 6)

            # construct noisey perturbation
            delta = gtsam.Pose3.retract(gtsam.Pose3(), noise_vector)

            # compose the noise
            noisy_rposes.append(rpose.compose(delta))
        return Odometry(noisy_rposes)



class Quadrics(object):
    """
    Explicit data association.
    Keys stored as dictionary keys
    Mapped to gtsamKey when added to estimate.
    """
    def __init__(self, quadrics=None):
        self._quadrics = quadrics
        if quadrics is None:
            self._quadrics = dict()

    def add(self, quadric, key):
        """ inline """
        self._quadrics[key] = quadric

    def items(self):
        return list(self._quadrics.items())

    def keys(self):
        return list(self._quadrics.keys())

    # def add_estimates(self, values):
    #     """ add q if n bbfs > 3 """
    #     for key, quadric in self._quadrics.items():
    #         quadric.addToValues(values, Q(key))
    #         # quadricslam.insertConstrainedDualQuadric(values, Q(key), quadric)
    #         # values.insert(Q(key), quadric)

    @staticmethod
    def from_values(values):
        estimate_keys = [values.keys().at(i) for i in range(values.keys().size())]
        quadric_keys = [k for k in estimate_keys if chr(gtsam.symbolChr(k)) == 'q']
        quadrics = {int(gtsam.symbolIndex(k)): quadricslam.ConstrainedDualQuadric.getFromValues(values, k) for k in quadric_keys}
        return Quadrics(quadrics)



# TODO: allow multiple boxes at same key set
# TODO: allow boxes with undefined object key
# TODO: provide fast lookup for boxes.at(pose) and boxes.at(quadric)
# TODO: provide fast access to list[boxes]
# TODO: provide fast access to boxes.pose_keys(), boxes.object_keys()
# TODO: add groupby_object() and groupby_pose()
class Boxes(object):
    def __init__(self, boxes=None):
        self._boxes = boxes
        if boxes is None:
            self._boxes = dict() # acts as a mock sparse 2D array

    def add(self, box, pose_key, object_key):
        """ inline add """
        self._boxes[pose_key, object_key] = box

    def add_boxes(self, boxes):
        """ inline add """
        # mashed_keys = len([keypair for keypair in self.keypairs() if keypair in boxes.keypairs()])
        self._boxes.update(boxes._boxes)

    def __len__(self):
        return len(self._boxes.values())

    # def __getitem__(self, index):
    #     """ returns ((pose_key, object_key), box) | O(1)"""
    #     return list(self._boxes.items())[index]
    
    def items(self):
        return list(self._boxes.items())

    def data(self):
        """ returns [boxes] | O(1)"""
        return list(self._boxes.values())

    def keypairs(self):
        return list(self._boxes.keys())

    def pose_keys(self):
        """ return in order with .data() the pose_keys | O(n)"""
        return [keypair[0] for keypair in self._boxes.keys()]

    def object_keys(self):
        """ return in order with .data() the object_keys | O(n)"""
        return [keypair[1] for keypair in self._boxes.keys()]

    def at(self, pose_key, object_key):
        return self._boxes[pose_key, object_key]

    def at_pose(self, pose_key):
        """ returns a Boxes object of boxes at pose_key | O(n)"""
        return Boxes({k:v for k,v in self._boxes.items() if k[0] == pose_key})

    def at_object(self, object_key):
        """ returns a Boxes object of boxes at object_key | O(n)"""
        return Boxes({k:v for k,v in self._boxes.items() if k[1] == object_key})

    # def add_factors(self, graph, noisemodel, calibration, image_dimensions, valid_quadrics):
    #     """ add bbf if q initialized and n > 3 """
        
    #     for object_key in np.unique(self.object_keys()):
    #         if object_key not in valid_quadrics:
    #             continue
            
    #         object_boxes = self.at_object(object_key)

    #         if len(object_boxes) > 3:
    #             for (pose_key, t), box in object_boxes.items():
    #                 bbf = quadricslam.BoundingBoxFactor(box, calibration, image_dimensions, X(pose_key), Q(object_key), noisemodel)
    #                 bbf.addToGraph(graph)
        
    #     # for (pose_key, object_key), box in self._boxes.items():
    #     #     bbf = quadricslam.BoundingBoxFactor(box, calibration, image_dimensions, X(pose_key), Q(object_key), noisemodel)
    #     #     bbf.addToGraph(graph)
    #         # graph.add(bbf)

    def add_noise(self, mu, sd):
        """ compose noisevec onto relative poses """
        noisy_boxes = Boxes()
        for (pose_key, object_key), box in list(self._boxes.items()):

            # get normally distributed doubles 
            noise_vector = np.random.normal(mu, sd, 4)

            # construct noisey perturbation
            noisy_box = quadricslam.AlignedBox2(box.vector() + noise_vector)

            # add box to collection
            noisy_boxes.add(noisy_box, pose_key, object_key)
        return noisy_boxes

    def prints(self):
        for object_key in np.unique(self.object_keys()):
            object_boxes = self.at_object(object_key)

            print('Object: q{} | {} views'.format(object_key, len(object_boxes)))
            # for (pose_key, t), box in object_boxes.items():
            #     print('    x{} -> q{}'.format(pose_key, object_key))
