"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Simulated dataset interface 
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
import numpy as np

# import gtsam and extension
import gtsam
import quadricslam

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

# import custom python modules
sys.dont_write_bytecode = True
from base.containers import Trajectory
from base.containers import Quadrics
from base.containers import Boxes


# TODO: ensure calibration / dimensions are linked correctly
class SimulatedSequence(object):
    """
    Automatically generate dataset information given a set of camera positions and quadrics. 
    """
    calibration = gtsam.Cal3_S2(525.0, 525.0, 0.0, 160.0, 120.0)

    def __init__(self, points, quadrics, n_interpolate=50):
        """
        :param - points: list of camera positions (Point3)
        :param - quadrics: list of ConstrainedDualQuadrics
        :param - n_interpolate: number of points to be generated between 'points'. 
        """

        # generate camera poses looking at a quadric 
        target = quadrics[0].pose().translation()
        poses = [gtsam.SimpleCamera.Lookat(point, target, gtsam.Point3(0,0,1)).pose() for point in points]

        # interpolate poses into trajectory
        self.true_trajectory = self.interpolate_poses(poses, n_interpolate)

        # create quadrics from list
        self.true_quadrics = Quadrics(dict(zip(range(len(quadrics)), quadrics)))

        # create box measurements
        self.true_boxes = self.reproject_quadrics(self.true_quadrics, self.true_trajectory)


    def reproject_quadrics(self, quadrics, trajectory):
        image_box = quadricslam.AlignedBox2(0,0,self.calibration.px()*2.0, self.calibration.py()*2.0)
        boxes = Boxes()
        for pose_key, pose in trajectory.items():
            for object_key, quadric in quadrics.items():

                # get raw box from projection
                dual_conic = quadricslam.QuadricCamera.project(quadric, pose, self.calibration)
                raw_box = dual_conic.bounds()
                # print(raw_box.vector())

                # only add boxes that project inside fov completely
                if image_box.contains(raw_box):
                    boxes.add(raw_box, pose_key, object_key)

                # correctly project partials to screen dimensions
                # elif image_box.intersects(raw_box):
                #     v = raw_box.vector()
                #     v[[0,2]] = np.clip(v[[0,2]], 0, self.image_dimensions[0])
                #     v[[1,3]] = np.clip(v[[1,3]], 0, self.image_dimensions[1])
                #     raw_box = AlignedBox2(v)
                #     boxes.add(box, pose_key, object_key)
        return boxes

    def interpolate_poses(self, poses, n_between):
        new_poses = []
        for i in range(len(poses)-1):

            # add real pose
            new_poses.append(poses[i])

            # interpolate new poses
            for j in range(n_between):
                percentage = (j+1) / float(n_between+1)
                ipose = quadricslam.interpolate(poses[i], poses[i+1], percentage)
                new_poses.append(ipose)

        # add final pose
        new_poses.append(poses[-1])
        return Trajectory(new_poses)


    @staticmethod
    def sequence1():
        """ a manually generated dataset sequence """
        points = []
        points.append(gtsam.Point3(10,0,0))
        points.append(gtsam.Point3(0,-10,0))
        points.append(gtsam.Point3(-10,0,0))
        points.append(gtsam.Point3(0,10,0))
        points.append(gtsam.Point3(10,0,0))

        quadrics = []
        quadrics.append(quadricslam.ConstrainedDualQuadric(gtsam.Pose3(), np.array([0.2,0.3,0.4])))
        quadrics.append(quadricslam.ConstrainedDualQuadric(gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.2,0.2,0.2)), np.array([0.2,0.3,0.4])))
        sequence = SimulatedSequence(points, quadrics)
        return sequence
