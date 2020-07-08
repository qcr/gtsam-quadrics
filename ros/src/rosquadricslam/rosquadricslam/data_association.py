# import standard libraries
import os
import sys
import numpy as np
from enum import Enum
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')
import cv2

# import custom libraries
sys.path.append('/home/lachness/git_ws/quadricslam/examples/python/example_frontend/')
sys.dont_write_bytecode = True
from base.containers import Detections
from visualization.drawing import CV2Drawing

# import gtsam and extension
import gtsam
import quadricslam


class TrackerType(Enum):
    BOOSTING = 1
    MIL = 2
    KCF = 3
    TLD = 4
    MEDIANFLOW = 5
    GOTURN = 6
    MOSSE = 7
    CSRT = 8

class BoxTracker(object):
    """
    Wrapper around cv2.tracker to convert box types and control tracker type.
    that allows us to stop using the tracker
    if it's failed in the past. 
    """

    def __init__(self, image, box, tracker_type=TrackerType.KCF):
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
        return quadricslam.AlignedBox2(tlwh[0], tlwh[1], tlwh[0]+tlwh[2], tlwh[1]+tlwh[3])

    def new_tracker(self):
        if self.tracker_type == TrackerType.BOOSTING:
            return cv2.TrackerBoosting_create() #'BOOSTING'
        elif self.tracker_type == TrackerType.MIL:
            return cv2.TrackerMIL_create() #'MIL'
        elif self.tracker_type == TrackerType.KCF:
            return cv2.TrackerKCF_create() #'KCF'
        elif self.tracker_type == TrackerType.TLD:
            return cv2.TrackerTLD_create() #'TLD'
        elif self.tracker_type == TrackerType.MEDIANFLOW:
            return cv2.TrackerMedianFlow_create() #'MEDIANFLOW'
        elif self.tracker_type == TrackerType.GOTURN:
            return cv2.TrackerGOTURN_create() #'GOTURN'
        elif self.tracker_type == TrackerType.MOSSE:
            return cv2.TrackerMOSSE_create() #'MOSSE'
        elif self.tracker_type == TrackerType.CSRT:
            return cv2.TrackerCSRT_create() #"CSRT"
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
    def __init__(self, image, box):

        # settings
        self.n_active_trackers = 1
        
        # create a new box tracker 
        self.trackers = [BoxTracker(image, box)]
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
        tracker = BoxTracker(image, box)
        self.trackers.append(tracker)



class DataAssociation(object):

    """
    We attempt to track objects until an object can be initialized.
    From then we use the existing map first to associate new measurements. 
    TODO: turn trackers off 
    TODO: enable multiple box trackers
    TODO: pass updateable version of map to constructor
    """

    def __init__(self, map, calibration):
        self.map = map
        self.calibration = calibration
        self.object_trackers = {}
        self.object_count = 0

        # settings 
        self.IOU_THRESH = 0.4
        self.object_limit = 100


    def associate(self, image, image_detections, camera_pose, pose_key, visualize=False, verbose=False):
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
            object_key, association_type, predicted_box = self.associate_detection(image, detection, camera_pose)
            if association_type == 'failed':
                continue
            
            # append association detection
            associated_detections.add(detection, pose_key, object_key)

            if visualize:
                if association_type == 'map' or association_type == 'tracker':
                    drawing.box(detection.box, (0,0,255))
                    color = (0,255,0) if association_type == 'map' else (255,255,0)
                    drawing.box_and_text(predicted_box, (0,255,0), association_type, (0,0,0))
                if association_type == 'new':
                    drawing.box(detection.box, (0,0,255))

            if verbose:
                associations.append(association_type)

        if visualize:
            cv2.imshow('data-association', img)
            cv2.waitKey(1)

        # TODO: get actual estimate for number of active trackers
        if verbose:
            print('\n --- Data-association --- ')
            print('  active_trackers: {}'.format(np.sum([t.n_active_trackers for t in self.object_trackers.values() if t.alive])))
            print('  map_objects:     {}'.format(len(self.map)))
            print('  measurements:    {}'.format(len(image_detections)))
            print('      tracker: {}'.format(len([t for t in associations if t=='tracker'])))
            print('      map:     {}'.format(len([t for t in associations if t=='map'])))
            print('      new:     {}'.format(len([t for t in associations if t=='new'])))
            
        return associated_detections


    def associate_detection(self, image, detection, pose):
        """ 
            Tries to associate detection with map and object trackers.
            If associated with tracker, updates tracker with new measurement. 
            returns (associated_key, association_type) 

            association_types: [map, tracker, new, failed]
            TODO: remove nasty list(dict.keys())
            TODO: use a datatype that will help us get the best predicted box
        """
        
        # calculate compatability with current map 
        map_ious = []
        for object_key, quadric in self.map.items():

            # TODO: catch projection failures 
            dual_conic = quadricslam.QuadricCamera.project(quadric, pose, self.calibration)
            predicted_box = dual_conic.bounds()
            map_ious.append([detection.box.iou(predicted_box), object_key, predicted_box])

        # calculate compatability with object trackers 
        tracker_ious = []
        for object_key, object_tracker in self.object_trackers.items():
            if object_tracker.alive:
                iou, predicted_box = object_tracker.compatability(detection.box)
                tracker_ious.append([iou, object_key, predicted_box])

        # attempt to associate detection to map/trackers
        best_map_iou = 0.0 if not map_ious else max(map_ious, key=lambda x: x[0])[0]
        best_tracker_iou = 0.0 if not tracker_ious else max(tracker_ious, key=lambda x: x[0])[0]
        
        if best_map_iou >= best_tracker_iou and best_map_iou > self.IOU_THRESH:
            object_key, predicted_box = max(map_ious, key=lambda x: x[0])[1:3]

            # turn off tracker to favor map
            if object_key in self.object_trackers:
                self.object_trackers[object_key].alive = False
            return (object_key, 'map', predicted_box)

        elif best_map_iou < best_tracker_iou and best_tracker_iou > self.IOU_THRESH:
            object_key, predicted_box = max(tracker_ious, key=lambda x: x[0])[1:3]
            best_tracker = self.object_trackers[object_key]
            best_tracker.add_tracker(detection.box, image)
            return (object_key, 'tracker', predicted_box)

        # create a new landmark for unassociated detections
        # TODO: check only active landmarks
        object_key = self.object_count
        self.object_count += 1
        if len(self.object_trackers) >= self.object_limit:
            return (object_key, 'new', None)
        new_tracker = ObjectTracker(image, detection.box)
        self.object_trackers[object_key] = new_tracker
        return (object_key, 'new', None)