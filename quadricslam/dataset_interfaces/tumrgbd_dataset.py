"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Dataset interface 
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import cv2
import sys
import math
import code
import json
import importlib
import numpy as np
from PIL import Image
from abc import ABC, abstractmethod
from scipy.optimize import linear_sum_assignment

# import gtsam and extension
import gtsam
import gtsam_quadrics

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

# import custom python modules
sys.dont_write_bytecode = True
from base.containers import ObjectDetection
from base.containers import Detections
from base.containers import Trajectory
from base.containers import Odometry
from base.containers import Quadrics
from dataset_interfaces.dataset import Dataset
from dataset_interfaces.dataset import Scene
from dataset_interfaces.dataset import SmartImages



# by default provides:
# - gt trajectory
# - accelerometer 
# - rgb
# - depth
# we can extend with precomputed
# - odometry (orb, orbvo, fovis, openvslam)
# - detections (yolov3, fasterrcnn)
# HOWEVER
# - GT trajectory is not aligned to images
# - depth is not aligned with RGB times


class TUMDataset(Dataset):

    def __init__(self, dataset_path):
        self.dataset_path = dataset_path
        self.extension_path = '/media/lachness/DATA/Datasets/TUM_extension/associated_tum_detections/associated_detections'
        
        # https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
        self.calibrations = {
            'freiburg1': gtsam.Cal3_S2(517.3, 516.5, 0.0, 318.6, 255.3),
            'freiburg2': gtsam.Cal3_S2(520.9, 521.0, 0.0, 325.1, 249.7),
            'freiburg3': gtsam.Cal3_S2(535.4, 539.2, 0.0, 320.1, 247.6),
        }

        self.scenes = self.load_scenes(dataset_path)

    def __getitem__(self, index):
        return self.scenes[index]

    def __len__(self):
        return len(self.scenes)

    def load_scenes(self, dataset_path):
        scenes = []
        for scene_name in os.listdir(dataset_path):
            if not os.path.isdir(os.path.join(dataset_path, scene_name)):
                continue
            
            scene_path = os.path.join(dataset_path, scene_name)
            scene = Scene(scene_path=scene_path)
            scene.calibration = self.load_calibration(scene_path)

            # load gt trajectory w/ time-keys
            scene.trajectory = self.load_trajectory(scene_path)
            
            # load rgb and depth
            scene.rgb_images = self.load_images(scene_path, 'rgb')
            scene.depth_images = self.load_images(scene_path, 'depth')

            # align trajectory->rgb
            scene.aligned_trajectory, scene.aligned_rgb = self.align_trajectory(scene.trajectory, scene.rgb_images)

            # load associated detections
            ad = self.load_annotated_detections(scene_name)

            # align times
            scene.associated_detections = Detections()
            for (pose_key, object_key), detection in ad.items():
                if pose_key in scene.aligned_trajectory.keys():
                    scene.associated_detections.add(detection, pose_key, object_key)


            scenes.append(scene)
        return scenes


    def load_annotated_detections(self, scene_name):
        data = json.load(open(os.path.join(self.extension_path, scene_name+'.json')))

        # convert each measurement to detection
        str_times = []
        detections = Detections()
        for frame in data:

            # extract frame
            object_key = int(frame['object_key'])
            objectness = float(frame['objectness'])
            box = gtsam_quadrics.AlignedBox2(*frame['box'])
            distribution = np.array(frame['scores'])
            str_time = frame['image_path'].split('/')[-1].replace('.png','')
            # pose_key = int(frame['image_key'])

            # create detection
            detection = ObjectDetection(box, 1.0, distribution)
            detections.add(detection, str_time, object_key)
        return detections

    def align_trajectory(self, trajectory, rgb_images):
        # do we change the times to match, or keep them seperate and match indicies/keys
        matches = self.align_times(list(rgb_images.keys()), list(trajectory.keys()))

        # set trajectory to use matching rgb_time
        aligned = Trajectory()
        for rkey, tkey in matches:
            aligned.add(trajectory.at(tkey), rkey)

        # take matched rgb_images
        rgb_images = {rkey:rgb_images[rkey] for rkey, tkey in matches}

        return aligned, rgb_images
        

    def align_times(self, first, second, offset=0.0, max_difference=0.02):
        """
        Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim
        to find the closest match for every input tuple.

        Input:
        first -- first list of timestamps
        second -- second list of timestamps
        offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
        max_difference -- search radius for candidate generation

        Output:
        matches -- list of matched tuples ((first_stamp,second_stamp),(first_stamp,second_stamp))

        """
            
        firstM = np.tile(np.array(first, dtype='float'), (len(second),1))
        secondM = np.tile(np.expand_dims(np.array(second, dtype='float'),0).T, (1,len(first)))
        cost = np.abs(firstM - (secondM + offset))
        assignment = linear_sum_assignment(cost)
        valid = cost[assignment] < max_difference

        secondI = assignment[0][valid]
        firstI = assignment[1][valid]

        matches = [(first[a], second[b]) for a, b in zip(firstI, secondI)]
        matches.sort()
        return matches

    def load_images(self, scene_path, folder='rgb'):
        image_names = sorted(os.listdir(os.path.join(scene_path, folder)))
        image_paths = [os.path.join(scene_path, folder, f) for f in image_names]
        image_times = [f.replace('.png','') for f in image_names]
        return dict(zip(image_times, image_paths))

    def load_calibration(self, scene_path):
        key = 'freiburg' + scene_path.split('freiburg')[1][0]
        return self.calibrations[key]

    def load_trajectory(self, scene_path):
        path = os.path.join(scene_path, 'groundtruth.txt')

        # load data
        text_file = open(path, 'r')
        text = text_file.read()
        lines = text.split('\n')

        # ignore comment lines
        lines = [line for line in lines if line and line[0] != '#']

        # convert each line to pose, key, time
        trajectory = Trajectory()
        for line in lines:
            time = line.split(' ')[0]; 
            numbers = line.split(' ')[1:]
            precision = len(time.split('.')[-1])

            if len(numbers) != 7:
                print('warning, invalid line: [', line, ']')
                continue

            # initialize gtsam pose
            numbers = np.array(numbers).astype('float')
            trans = gtsam.Point3(numbers[0], numbers[1], numbers[2])
            rot = gtsam.Rot3.Quaternion(numbers[6], numbers[3], numbers[4], numbers[5]) # Quaternion(double w, double x, double y, double z)
            gtsam_pose = gtsam.Pose3(rot, trans)

            # construct pose
            trajectory.add(gtsam_pose, time)

        return trajectory



if __name__ == '__main__':

    dataset = TUMDataset('/media/lachness/DATA/Datasets/TUM/')
    scene = dataset[0]

    for time, path in scene.rgb_images.items():
        rgb_image = cv2.imread(rgb_image)
        cv2.imshow('test', rgb_image)
        cv2.waitKey(0)