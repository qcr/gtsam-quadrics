"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Runs the system through datasets and investigates how often reprojection fails
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# import gtsam and extension
import gtsam
import quadricslam

# import custom python modules
sys.dont_write_bytecode = True
from simulated_dataset import ManualSequence
from scenenet_dataset import SceneNetDataset
from containers import *
from drawing import Drawing
from system import System


if __name__ == '__main__':
    """ 
    We want to classify quadrics as being:
    - entirely viewable
    - partially viewable
    - not visible at all
    As to the volume overlap between fov envolope and quadric

    Noting that: p.T * Q * p = 0
    - is negative when the plane intersects the quadric
    ^ only when the quadric is normalized

    Noting that: p.T * X = 0
    - for fov planes usually we expect [+,+,-,-]
        which indicates the point is inside the fov
    - if any sign is not correct, 
        it means the point is on the other side of the plane
    
    """

    camera_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0,0,-5))
    qpose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.5,0,0))
    qradii = np.array([0.3,0.3,0.3])
    quadric = quadricslam.ConstrainedDualQuadric(qpose, qradii)
    qmatrix = quadric.matrix()
    qmatrix = qmatrix/qmatrix[-1,-1]

    # print('qmatrix\n', qmatrix)

    fov = quadricslam.AlignedBox2(0,0,320,240)
    fov_lines = [fov.lines().at(i) for i in range(fov.lines().size())]

    calibration = gtsam.Cal3_S2(525.0, 525.0, 0.0, 160.0, 120.0)
    P = quadricslam.QuadricCamera.transformToImage(camera_pose, calibration).transpose()
    fov_planes = [P @ line for line in fov_lines]
    errors = [plane.transpose() @ qmatrix @ plane for plane in fov_planes]
    # print('fov_planes', fov_planes)
    # print('errors', errors)

    conic = quadricslam.QuadricCamera.project(quadric, camera_pose, calibration)
    print('conic bounds', conic.bounds().vector())

    p = np.array([3,0,0,1.])
    point_errors = [plane.transpose() @ p for plane in fov_planes]
    # print('point_errors', point_errors)

    dC = conic.matrix()
    C = np.linalg.inv(dC)
    lines = []
    lines.append( np.array([1., 0., -0.]) )
    lines.append( np.array([0., 1., -0.]) )
    lines.append( np.array([1., 0., -320.]) )
    lines.append( np.array([0., 1., -240.]) )
    for l in lines:
        X = l.transpose() @ dC
        print(X.T @ C, l.T)
        print(X.T @ C @ X,  l.T @ X)
        # print(l.transpose() @ X)
        # print(X[0:2]/X[-1])



    # sequence = ManualSequence.sequence1()
    # System.run(sequence)

