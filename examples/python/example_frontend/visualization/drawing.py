"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Drawing interface
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# import gtsam and extension
import gtsam
import quadricslam

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

# import custom python modules
sys.dont_write_bytecode = True
from base.containers import Trajectory
from base.containers import Odometry
from base.containers import Quadrics
from base.containers import Boxes

symbolChr = lambda i: chr(gtsam.symbolChr(i))

class Drawing(object):

    @staticmethod
    # TODO: draw camera fov
    # TODO: project quadric shape orthographically
    def draw_problem(graph, estimate):
        """ assumes z axis is up """

        estimated_trajectory = Trajectory.from_values(estimate)
        estimated_quadrics = Quadrics.from_values(estimate)

        for pose_key, pose in estimated_trajectory.items():
            plt.plot(pose.x(), pose.y(), marker='o', markersize=3, color='c')
            plt.text(pose.x(), pose.y(), 'x{}'.format(pose_key))

        for object_key, quadric in estimated_quadrics.items():
            plt.plot(quadric.getPose().x(), quadric.getPose().y(), marker='o', markersize=3, color='m')
            plt.text(quadric.getPose().x(), quadric.getPose().y(), 'q{}'.format(object_key))

        plt.show()

        # for quadric in 


        # graph_keys = [graph.keys().at(i) for i in range(graph.keys().size())]
        # draw quadric variables

        # draw odometry factors

        # draw bbox factors

    @staticmethod
    def draw_results(trajectories, quadrics, colors):
        for i, trajectory in enumerate(trajectories):
            xy = np.array([[pose.x(), pose.y()] for pose in trajectory.data()])
            plt.plot(xy[:,0], xy[:,1], marker='o', markersize=3, c=colors[i], label='traj {}'.format(i))
        
        for i, _quadrics in enumerate(quadrics):
            for quadric in _quadrics.data():
                plt.plot(quadric.getPose().x(), quadric.getPose().y(), marker='o', markersize=3, c=colors[i], label='quads {}'.format(i))

        plt.show()

    
    @staticmethod
    def plot_problem(graph, estimate, calibration):

        # extract variables and factors
        trajectory = Trajectory.from_values(estimate)
        quadrics = Quadrics.from_values(estimate)

        # collect nonlinear factors from graph
        box_factors = [quadricslam.BoundingBoxFactor.getFromGraph(graph, i) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
        # box_factors = [graph.at(i) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
        odom_factors = [graph.at(i) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'x']

        # calculate x,y, error per pose
        xy = []
        errors = []
        for pose_key in trajectory.keys():
            pose = trajectory.at(pose_key)
            xy.append([pose.x(), pose.y()])

            bbfs = [f for f in box_factors if gtsam.symbolIndex(f.keys().at(0)) == pose_key]

            # get sum of each bbf error at pose 
            # bbf error = || x-x' ||^2
            error = np.sum([np.square(bbf.unwhitenedError(estimate)).sum() for bbf in bbfs])
            # error = np.sum([bbf.error(estimate) for bbf in bbfs])
            errors.append(error)

        xy = np.array(xy)
        errors = np.array(errors)


        for pose_key in trajectory.keys():

            # plot full trajectory
            fig = plt.figure('Trajectory')
            fig.clf()
            plt.plot(xy[:,0], xy[:,1], linestyle='-', c='c', zorder=1)
            plt.scatter(xy[:,0], xy[:,1], s=25, c=errors, zorder=9)
            plt.colorbar()
            
            # plot quadrics
            for quadric in quadrics.data():
                plt.plot(quadric.getPose().x(), quadric.getPose().y(), marker='o', markersize=5, c='m', fillstyle='none')
                plt.plot(quadric.getPose().x(), quadric.getPose().y(), marker='o', markersize=5, c='m', alpha=0.5)

            # highlight specific pose
            # pose = trajectory.at(pose_key)
            # plt.scatter(pose.x(), pose.y(), s=45, facecolors='none', edgecolors='r', zorder=10)

            # # draw camera fov
            # point = pose.compose(gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0,0,2))).translation()
            # plt.plot((pose.x(), point.x()), (pose.y(), point.y()), linestyle='-', marker='o', markersize=5, c='g', zorder=10)

            # # draw fpv
            # fig = plt.figure('FPV Conic')
            # fig.clf()
            # for quadric in quadrics.data():
                # dual_conic = quadricslam.QuadricCamera.project(quadric, pose, calibration).matrix()

            #     Drawing.mpl_draw_conic(dual_conic)
            # plt.draw()
            # plt.pause(0.001)
            # # plt.waitforbuttonpress()
            break

        # draw fpv on mouseover 
        # empty_image = np.zeros((240, 320, 3), dtype=np.uint8) + 255
        # for pose_key in trajectory.keys():
        #     pose = trajectory.at(pose_key)
        #     # boxes = boxes.at_pose(pose_key)
        #     image = empty_image.copy()
        #     Drawing.draw_fpv(image, pose, quadrics, calibration)
        #     cv2.imshow('test', image)
        #     cv2.waitKey(0)


        plt.show()

    @staticmethod
    def draw_fpv(image, pose, quadrics, calibration):
        for quadric in quadrics.data():
            Drawing.cv2_draw_quadric(image, pose, quadric, calibration)

        # for box in boxes.data():
        #     Drawing.cv2_draw_box(image, box)
        
    @staticmethod
    def cv2_draw_box(image, box, color=(0,0,255), thickness=2):
        cv2.rectangle(image, (int(box.xmin()),int(box.ymin())), (int(box.xmax()),int(box.ymax())), color, thickness)

    @staticmethod
    def cv2_draw_quadric(image, pose, quadric, calibration, color=(255,0,0), alpha=1):
        points_2D = Drawing.generate_uv_spherical(quadric, pose, calibration, 10, 10)
        points_2D = np.round(points_2D).astype('int')
        # color = (0,0,255)
        color = (color[2], color[1], color[0]) # rgb to bgr

        if alpha!=1:
            full_image = image.copy()

        for i in range(points_2D.shape[0]):
            for j in range(points_2D.shape[1]-1):
                point_2D = points_2D[i,j]
                nextpoint_2D = points_2D[i,j+1]
                cv2.line(image, (point_2D[0], point_2D[1]), (nextpoint_2D[0], nextpoint_2D[1]), color, 1, cv2.LINE_AA)

        for j in range(points_2D.shape[1]):
            for i in range(points_2D.shape[0]-1):
                point_2D = points_2D[i,j]
                nextpoint_2D = points_2D[i+1,j]
                cv2.line(image, (point_2D[0], point_2D[1]), (nextpoint_2D[0], nextpoint_2D[1]), color, 1, cv2.LINE_AA)

        if alpha!=1:
            cv2.addWeighted(image, alpha, full_image, 1-alpha, 0, image)

    @staticmethod
    def generate_uv_spherical(quadric, pose, calibration, theta_points=30, phi_points=30):
        rotation = quadric.getPose().rotation().matrix()
        translation = quadric.getPose().translation().vector()

        # Radii corresponding to the coefficients:
        rx, ry, rz = quadric.getRadii()

        # Set of all spherical angles:
        u = np.linspace(0, 2 * np.pi, theta_points)
        v = np.linspace(0, np.pi, phi_points)

        # Cartesian coordinates that correspond to the spherical angles:
        x = rx * np.outer(np.cos(u), np.sin(v))
        y = ry * np.outer(np.sin(u), np.sin(v))
        z = rz * np.outer(np.ones_like(u), np.cos(v))
        # w = np.ones(x.shape)
        points = np.stack((x,y,z))

        points_2D = np.zeros((points.shape[1], points.shape[2], 2))
        transform_to_image = quadricslam.QuadricCamera.transformToImage(pose, calibration)

        # warp points to quadric (3D) and project to image (2D)
        for i in range(points.shape[1]):
            for j in range(points.shape[2]):
                # point = [x[i,j], y[i,j], z[i,j], 1.0]
                point = points[:,i,j]
                warped_point = point.dot(np.linalg.inv(rotation))
                warped_point += translation

                point_3D = np.array([warped_point[0], warped_point[1], warped_point[2], 1.0])
                point_2D = transform_to_image.dot(point_3D)
                point_2D = point_2D[:-1]/point_2D[-1]
                points_2D[i,j] = point_2D

        return points_2D

    @staticmethod
    def mpl_draw_conic(dual_conic):
        conic = np.linalg.inv(dual_conic)

        irange = 2000
        x = np.linspace(-irange, irange, 1000)
        y = np.linspace(-irange, irange, 1000)
        x, y = np.meshgrid(x, y)

        a = conic[0,0]; 
        b = conic[1,0]*2.0; 
        c = conic[1,1]
        d = conic[2,0]*2.0; 
        e = conic[2,1]*2.0; 
        f = conic[2,2]

        # conic_descriminant = b**2 - 4*a*c
        # print('conic_descriminant: {}'.format(conic_descriminant))

        plt.contour(x, y, (a*x**2 + b*x*y + c*y**2 + d*x + e*y + f), [0], colors='r')

        fig = plt.gcf()
        ax = fig.gca()
        ax.invert_yaxis()
        # rect = patches.Rectangle((0,0),320,240,linewidth=1,edgecolor='r',facecolor='none')
        # ax.add_patch(rect)

        # dual_conic = dual_conic/dual_conic[-1,-1]
        # conic_point = [dual_conic[0,2], dual_conic[1,2]]

        # plt.scatter(conic_point[0], conic_point[1], c='r')
        # plt.xlim((0,320))
        # plt.ylim((0,240))