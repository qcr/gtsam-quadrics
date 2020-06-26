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


X = lambda i: int(gtsam.symbol(ord('x'), i))
Q = lambda i: int(gtsam.symbol(ord('q'), i))

class Trajectory(object):
    """ 
    Class for storing a sequence of 3D poses. 
    Data association is implicit where the pose_key is represented as index of pose.
    """
    def __init__(self, poses):
        self._poses = poses

    def __len__(self):
        return len(self._poses)

    def __getitem__(self, index):
        return self._poses[index]

    def items(self):
        """ Returns key-value pairs """
        return list(zip(range(len(self._poses)), self._poses))

    def data(self):
        """ Returns a list of Pose3 objects """
        return self._poses

    def keys(self):
        """ Returns a list of pose_keys """
        return list(range(len(self._poses)))

    def at(self, key):
        """ Get the pose at key """
        return self._poses[key]

    def at_keys(self, keys):
        """ Returns a new trajectory containing the poses at associated with keys """
        return Trajectory([self._poses[key] for key in keys])

    def as_odometry(self):
        """ Converts the trajectory to a sequence of relative poses """
        relative_poses = [self._poses[i].between(self._poses[i+1]) for i in range(len(self._poses)-1)]
        return Odometry(relative_poses)

    def applyTransform(self, reference):
        """ Transforms trajectory from local coordinates to reference coordinates """
        poses = [reference.transformPoseFrom(pose) for pose in self._poses]
        return Trajectory(poses)

    @staticmethod
    def from_values(values):
        estimate_keys = [values.keys().at(i) for i in range(values.keys().size())]
        pose_keys = [k for k in estimate_keys if chr(gtsam.symbolChr(k)) == 'x']
        poses = [values.atPose3(k) for k in pose_keys]
        return Trajectory(poses)


            

            


class Odometry(object):
    """ 
    Class for storing a sequence of relative pose measurements.
    Data association is implicit in the index of the relative pose.
    """
    def __init__(self, rposes):
        self._rposes = rposes

    def __len__(self):
        return len(self._rposes)

    def __getitem__(self, index):
        return self._rposes[index]

    def items(self):
        """ Returns [(start_key, end_key), rpose] for each relative pose """
        return list(zip(zip(range(len(self._rposes)), range(1,len(self._rposes)+1)), self._rposes))

    def data(self):
        """ Returns list of relative poses """
        return self._rposes

    def as_trajectory(self, reference=gtsam.Pose3()):
        """ Converts relative poses to global trajectory 
        reference = first pose of global trajectory
        """
        global_poses = [reference]
        for rpose in self._rposes:
            global_poses.append( global_poses[-1].compose(rpose) )
        return Trajectory(global_poses)

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

    def __len__(self):
        return len(self._quadrics)

    def add(self, quadric, key):
        """ inline """
        self._quadrics[key] = quadric

    def items(self):
        return list(self._quadrics.items())

    def data(self):
        return list(self._quadrics.values())

    def keys(self):
        return list(self._quadrics.keys())

    def at(self, key):
        return self._quadrics[key]

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
    
    def items(self):
        """ Returns [(pose_key, object_key), box] for each box """
        return list(self._boxes.items())

    def data(self):
        """ returns [boxes] | O(1)"""
        return list(self._boxes.values())

    def remove(self, pose_key, object_key):
        self._boxes.pop((pose_key, object_key))

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

    def add_noise_strict(self, mu, sd, image_dimensions):
        """ Take care to ensure boxes have positive width/height
        And they do not extend past the image dimensions """
        image_box = quadricslam.AlignedBox2(0,0,image_dimensions[0],image_dimensions[1])
        noisy_boxes = Boxes()
        for (pose_key, object_key), box in list(self._boxes.items()):

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

    def prints(self, detailed=False):
        for object_key in np.unique(self.object_keys()):
            object_boxes = self.at_object(object_key)

            print('Object: q{} | {} views'.format(object_key, len(object_boxes)))

            if detailed:
                for (pose_key, t), box in object_boxes.items():
                    print('    x{} -> q{}'.format(pose_key, object_key))
