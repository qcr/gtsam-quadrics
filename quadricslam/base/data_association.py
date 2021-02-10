"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Contains DataAssociation class for association ObjectDetections with object trackers or existing quadrics
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
import numpy as np
from enum import Enum
import cv2

# import custom libraries
sys.dont_write_bytecode = True
from base.containers import Detections
from visualization.drawing import CV2Drawing

# import gtsam and extension
import gtsam
import gtsam_quadrics


class BoxTracker(object):
    """
    Wrapper around cv2.tracker to convert box types and control tracker type.
    that allows us to stop using the tracker
    if it's failed in the past. 
    """

    def __init__(self, image, box, tracker_type):
        self.tracker_type = tracker_type
        self.tracker = self.new_tracker()
        try:
            self.tracker.init(image, self.to_cvbox(box))
        except:
            print('tracker wont init, box: ', self.to_cvbox(box))
            exit()
    
    @staticmethod
    def to_cvbox(box):
        return (box.xmin(), box.ymin(), box.xmax()-box.xmin(), box.ymax()-box.ymin())

    @staticmethod
    def from_cvbox(tlwh):
        return gtsam_quadrics.AlignedBox2(tlwh[0], tlwh[1], tlwh[0]+tlwh[2], tlwh[1]+tlwh[3])

    def new_tracker(self):
        if self.tracker_type == 'BOOSTING':
            return cv2.TrackerBoosting_create()
        elif self.tracker_type == 'MIL':
            return cv2.TrackerMIL_create()
        elif self.tracker_type == 'KCF':
            return cv2.TrackerKCF_create()
        elif self.tracker_type == 'TLD':
            return cv2.TrackerTLD_create()
        elif self.tracker_type == 'MEDIANFLOW':
            return cv2.TrackerMedianFlow_create()
        elif self.tracker_type == 'GOTURN':
            return cv2.TrackerGOTURN_create()
        elif self.tracker_type == 'MOSSE':
            return cv2.TrackerMOSSE_create()
        elif self.tracker_type == 'CSRT':
            return cv2.TrackerCSRT_create()
        else:
            raise ValueError('BoxTracker new_tracker called with unknown self.tracker_type')

    def update(self, image):    
        ok, cvbox = self.tracker.update(image)
        box = self.from_cvbox(cvbox)
        return ok, box

class ObjectTracker(object):
    """
    ObjectTracker will track objects frame to frame,
    Automatically becomes inactive if:
        - not associated in n updates 
        - not ok for n updates 
        - too many trackers
    """
    def __init__(self, image, box, tracker_type):

        # settings
        self.tracker_type = tracker_type
        self.n_active_trackers = 1
        
        # create a new box tracker 
        self.trackers = [BoxTracker(image, box, self.tracker_type)]
        self.predictions = []
        self.alive = True

    def update(self, image):
        # clear previous predictions
        self.predictions = []
        
        # update most recent trackers
        for tracker in self.trackers[-self.n_active_trackers:]:
            ok, box = tracker.update(image)
            if ok:
                self.predictions.append(box)

    def compatability(self, box):
        """ Returns (best_iou, best_prediction) """
        if len(self.predictions) == 0:
            return (0.0, None)
        ious = [[box.iou(predicted_box), predicted_box] for predicted_box in self.predictions]
        return max(ious, key=lambda x: x[0])

    def add_tracker(self, box, image):
        tracker = BoxTracker(image, box, self.tracker_type)
        self.trackers.append(tracker)






class DataAssociation(object):

    """
    We attempt to track objects until an object can be initialized.
    From then we use the existing map first to associate new measurements. 
    TODO: turn trackers off 
    TODO: enable multiple box trackers
    TODO: pass updateable version of map to constructor
    """

    def __init__(self, calibration, config):
        self.calibration = calibration
        self.object_trackers = {}
        self.object_count = 0

        # settings 
        self.iou_thresh = config['DataAssociation.iou_thresh']
        self.object_limit = config['DataAssociation.object_limit']
        self.tracker_type = config['DataAssociation.tracker_type']


    def associate(self, image, image_detections, camera_pose, pose_key, map, visualize=False, verbose=False):
        associated_detections = Detections()

        # update active trackers with new image 
        for object_key, object_tracker in self.object_trackers.items():
            if object_tracker.alive:

                # update image
                object_tracker.update(image)

        if visualize:
            img = image.copy()
            drawing = CV2Drawing(img)

        if verbose:
            associations = []

        for detection in image_detections:

            # associate single measurement
            object_key = self.associate_detection(image, detection, camera_pose, map)
            
            # append association detection
            associated_detections.add(detection, pose_key, object_key)

            # draw associated detection
            if visualize:
                drawing.box_and_text(detection.box, (255,255,0), '{}'.format(object_key), (255,255,255))

        if visualize:
            cv2.imshow('data-association', img)
            cv2.waitKey(1)

        # TODO: get actual estimate for number of active trackers
        if verbose:
            print('\n --- Data-association --- ')
            print('  active_trackers: {}'.format(np.sum([t.n_active_trackers for t in self.object_trackers.values() if t.alive])))
            print('  map_objects:     {}'.format(len(map)))
            print('  measurements:    {}'.format(len(image_detections)))
            print('      tracker: {}'.format(len([t for t in associations if t=='tracker'])))
            print('      map:     {}'.format(len([t for t in associations if t=='map'])))
            print('      new:     {}'.format(len([t for t in associations if t=='new'])))
            
        return associated_detections


    def associate_detection(self, image, detection, pose, map):
        """ 
            Tries to associate detection with map and object trackers.
            If associated with tracker, updates tracker with new measurement. 
            returns (associated_key, association_type) 

            association_types: [map, tracker, new, failed]
            TODO: remove nasty list(dict.keys())
            TODO: use a datatype that will help us get the best predicted box
        """
        compatabilities = []

        # calculate compatability with current map 
        for object_key, quadric in map.items():

            # TODO: catch projection failures 
            dual_conic = gtsam_quadrics.QuadricCamera.project(quadric, pose, self.calibration)
            predicted_box = dual_conic.bounds()
            iou = detection.box.iou(predicted_box)

            # append to compatabilities
            compatabilities.append({
                'compatability': iou,
                'object_key': object_key,
                'type': 'map',
            })

        # calculate compatability with object trackers 
        for object_key, object_tracker in self.object_trackers.items():
            if object_tracker.alive:
                comp, predicted_box = object_tracker.compatability(detection.box)

                # append to compatabilities
                compatabilities.append({
                    'compatability': comp,
                    'object_key': object_key,
                    'type': 'tracker',
                })

        if len(compatabilities) > 0:

            # get the best association
            best_frame = max(compatabilities, key=lambda x: x['compatability'])
            
            # associate with map
            if best_frame['compatability'] >= self.iou_thresh and best_frame['type'] == 'map':
                return best_frame['object_key']
                
            # associate with tracker
            elif best_frame['compatability'] >= self.iou_thresh and best_frame['type'] == 'tracker':
                best_tracker = self.object_trackers[best_frame['object_key']]
                best_tracker.add_tracker(detection.box, image)
                return best_frame['object_key']

        # otherwise create new tracker
        # get a new object key
        object_key = self.object_count
        self.object_count += 1

        # create a new tracker
        new_tracker = ObjectTracker(image, detection.box, self.tracker_type)
        self.object_trackers[object_key] = new_tracker

        return object_key



import torch


class KeepAssociator(object):
    def __init__(self, thresh=0.8, layer='fc2'):
        self.landmarks = {} # stores a list of [activations] for each key
        self.cosine_threshold = thresh
        self.activation_layer = layer
        self.operation = 'online'
        self.combine = 'keepall'
        self.name = 'Keeper({:.1f},{})'.format(self.cosine_threshold, self.activation_layer)

    def associate(self, image, detections, pose_key):
        associated = Detections()
        for detection in detections:
            object_key = self.associate_single(image, detection)
            associated.add(detection, pose_key, object_key)
        print('-- DataAssociation --')
        print('n-landmarks: {}'.format(len(self.landmarks)))
        print('total codes: {}'.format(np.sum([len(activations) for activations in self.landmarks.values()])))
        return associated

    def associate_single(self, image, detection):
        if self.activation_layer == 'fc1':
            activation = detection.fc1
        elif self.activation_layer == 'fc2':
            activation = detection.fc2
        
        # calculate measurements compatability against existing landmarks
        compatability = []
        for key in self.landmarks.keys():
            landmark = self.landmarks[key]

            # calculate compatability against previous measurements
            landmark_similarities = [torch.cosine_similarity(activation, past_activation).cpu().numpy() for past_activation in landmark]
            compatability.append(np.max(landmark_similarities))

        # associate with existing landmark
        if len(compatability) and np.max(compatability) > self.cosine_threshold:
            key = list(self.landmarks.keys())[np.argmax(compatability)]
            self.landmarks[key].append(activation)

        else: # create new landmark
            key = len(self.landmarks)
            self.landmarks[key] = [activation]
        return key

    def clear(self):
        self.landmarks = {}





class MergeAssociator(object):
    def __init__(self, thresh=0.8, layer='fc2'):
        self.landmarks = {} # stores a list of [activations] for each key
        self.cosine_threshold = thresh
        self.activation_layer = layer
        self.operation = 'online'
        self.combine = 'average'
        self.name = 'Merger({:.1f},{})'.format(self.cosine_threshold, self.activation_layer)

    def associate(self, image, detections, pose_key):
        img = image.copy()
        drawing = CV2Drawing(img)
        
        associated = Detections()
        for detection in detections:
            object_key = self.associate_single(image, detection)
            associated.add(detection, pose_key, object_key)

            # draw associated detection
            drawing.box_and_text(detection.box, (255,255,0), '{}'.format(object_key), (255,255,255))

        cv2.imshow('data-association', img)
        cv2.waitKey(1)
            
        print('-- DataAssociation --')
        print('n-landmarks: {}'.format(len(self.landmarks)))
        return associated

    def associate_single(self, image, detection):
        if self.activation_layer == 'fc1':
            activation = detection.fc1
        elif self.activation_layer == 'fc2':
            activation = detection.fc2
        
        # calculate measurements compatability against existing landmarks
        compatability = []
        for key in self.landmarks.keys():
            combined_activation = self.landmarks[key]['activation']

            # calculate compatability against previous measurements
            landmark_similarities = torch.cosine_similarity(activation, combined_activation).cpu().numpy() 
            compatability.append(np.max(landmark_similarities))

        # associate with existing landmark
        if len(compatability) and np.max(compatability) > self.cosine_threshold:
            key = list(self.landmarks.keys())[np.argmax(compatability)]
            self.update_landmark(key, activation)
        else: # create new landmark
            key = len(self.landmarks)
            self.new_landmark(key, activation)
        return key

    def update_landmark(self, key, activation):
        self.landmarks[key]['activation'] = ((self.landmarks[key]['activation'] * self.landmarks[key]['n']) + activation) / (self.landmarks[key]['n']+1)
        self.landmarks[key]['n'] += 1

    def new_landmark(self, key, activation): 
        self.landmarks[key] = {'activation': activation, 'n':1}

    def clear(self):
        self.landmarks = {}

