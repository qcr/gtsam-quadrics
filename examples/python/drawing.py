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
from matplotlib.lines import Line2D

# import gtsam and extension
import gtsam
import gtsam_quadrics

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

# import custom python modules
sys.dont_write_bytecode = True

class CV2Drawing(object):
    def __init__(self, image):
        self._image = image
        self.image_width = self._image.shape[1]
        self.image_height = self._image.shape[0]
    
    def box(self, box, color=(0,0,255), thickness=2):
        cv2.rectangle(self._image, (int(box.xmin()),int(box.ymin())), (int(box.xmax()),int(box.ymax())), color, thickness)

    def text(self, text, lower_left, color=(255,255,255), thickness=1, background=False, background_color=(0,0,255), background_margin=3):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5

        lower_left = [lower_left[0]+background_margin, lower_left[1]-background_margin]
        text_size = cv2.getTextSize(text, font, font_scale, thickness)
        text_width = text_size[0][0] + background_margin * 2
        text_height = text_size[0][1] + background_margin * 2


        # lower_left = [upper_left[0], upper_left[1]+text_width]
        final_position = list(lower_left)
        final_position[1] = int(np.clip(lower_left[1], text_height, self.image_height))
        final_position[0] = int(np.clip(lower_left[0], 0, self.image_width-text_width))
        final_position = tuple(final_position)

        if (background):
            upper_left = [final_position[0], final_position[1]-text_height]
            xmin = upper_left[0]-background_margin+1
            ymin = upper_left[1]-background_margin+4
            xmax = upper_left[0]+text_width+background_margin
            ymax = upper_left[1]+text_height+background_margin+1
            cv2.rectangle(self._image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), background_color, cv2.FILLED)
        cv2.putText(self._image, text, final_position, font, font_scale, color, thickness, cv2.LINE_AA)

    def box_and_text(self, box, box_color, text, text_color):
        box_thickness = 2
        self.box(box, box_color, thickness=box_thickness)
        self.text(text, (box.xmin()-box_thickness,box.ymin()-box_thickness), text_color, 1, True, box_color)

    def quadric(self, pose, quadric, calibration, color=(255,0,0), alpha=1):
        """ 
        Draws a wireframe quadric at camera position.
        Will not draw lines if both ends project outside image border.
        Will not draw if quadric is behind camera 
        """
        if quadric.isBehind(pose):
            return
        
        image_box = gtsam_quadrics.AlignedBox2(0,0,self.image_width, self.image_height)
        points_2D = self.generate_uv_spherical(quadric, pose, calibration, 20, 20)
        points_2D = np.round(points_2D).astype('int')
        # color = (0,0,255)
        color = (color[2], color[1], color[0]) # rgb to bgr

        if alpha!=1:
            full_image = self._image.copy()

        for i in range(points_2D.shape[0]):
            for j in range(points_2D.shape[1]-1):
                point_2D = points_2D[i,j]
                nextpoint_2D = points_2D[i,j+1]
                if image_box.contains(gtsam.Point2(*point_2D)) or image_box.contains(gtsam.Point2(*nextpoint_2D)):
                    cv2.line(self._image, (point_2D[0], point_2D[1]), (nextpoint_2D[0], nextpoint_2D[1]), color, 1, cv2.LINE_AA)

        for j in range(points_2D.shape[1]):
            for i in range(points_2D.shape[0]-1):
                point_2D = points_2D[i,j]
                nextpoint_2D = points_2D[i+1,j]
                if image_box.contains(gtsam.Point2(*point_2D)) or image_box.contains(gtsam.Point2(*nextpoint_2D)):
                    cv2.line(self._image, (point_2D[0], point_2D[1]), (nextpoint_2D[0], nextpoint_2D[1]), color, 1, cv2.LINE_AA)

        if alpha!=1:
            cv2.addWeighted(self._image, alpha, full_image, 1-alpha, 0, self._image)

    def generate_uv_spherical(self, quadric, pose, calibration, theta_points=30, phi_points=30):
        rotation = quadric.pose().rotation().matrix()
        translation = quadric.pose().translation().vector()

        # Radii corresponding to the coefficients:
        rx, ry, rz = quadric.radii()

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
        transform_to_image = gtsam_quadrics.QuadricCamera.transformToImage(pose, calibration)

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