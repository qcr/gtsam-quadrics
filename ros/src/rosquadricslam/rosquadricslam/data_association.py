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
    Ensure you call update(image) before checking compatability. 
    """
    def __init__(self, object_key, image, box):
        self.object_key = object_key
        
        # define parameters
        self.n_active_trackers = 1
        
        # store trackers
        tracker = BoxTracker(image, box)
        self.trackers = [tracker]

        # store last prediction
        self.predictions = []

    def update(self, image):
        # clear previous predictions
        self.predictions = []
        
        # update most recent trackers
        for tracker in self.trackers[-self.n_active_trackers:]:
            ok, box = tracker.update(image)
            if ok:
                self.predictions.append(box)

    def compatability(self, box):
        """
        Checks how compatible new box is with trackers
        """
        if len(self.predictions) == 0:
            return 0.0
        ious = [self._iou(predicted_box, box) for predicted_box in self.predictions]
        return np.max(ious)

    def add_tracker(self, box, image):
        tracker = BoxTracker(image, box)
        self.trackers.append(tracker)

    @staticmethod
    def _iou(boxA, boxB):
        # determine the (x, y)-coordinates of the intersection rectangle
        xA = max(boxA.xmin(), boxB.xmin())
        yA = max(boxA.ymin(), boxB.ymin())
        xB = min(boxA.xmax(), boxB.xmax())
        yB = min(boxA.ymax(), boxB.ymax())

        # compute the area of intersection rectangle
        interArea = max(0, max(0,(xB - xA + 1)) * max(0,(yB - yA + 1)))

        # compute the area of both the prediction and ground-truth boxes
        boxAArea = (boxA.xmax() - boxA.xmin() + 1) * (boxA.ymax() - boxA.ymin() + 1)
        boxBArea = (boxB.xmax() - boxB.xmin() + 1) * (boxB.ymax() - boxB.ymin() + 1)

        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        iou = interArea / float(boxAArea + boxBArea - interArea)
        return iou

class DataAssociation(object):
    """
    Detections are assigned to an object key as they come in. 
    They should not be stored without a key as we never go back to update / arange them.
    """
    def __init__(self):
        self.object_trackers = []
        self.IOU_THRESH = 0.4

    def track(self, image, image_detections, pose_key):
        # store associated detections
        associated_detections = Detections()
        
        # debug variables
        new_objects = 0
        tracked_objects = 0
        
        # update object trackers with new image
        # should also turn inactive if not matched in X frames
        for object_tracker in self.object_trackers:
            object_tracker.update(image)

        # check compatability between each detection and active trackers
        for detection in image_detections:
            compatabilities = [tracker.compatability(detection.box) for tracker in self.object_trackers]

            # associate with existing tracker
            if len(compatabilities) > 0 and np.max(compatabilities) > self.IOU_THRESH:
                self.object_trackers[np.argmax(compatabilities)].add_tracker(detection.box, image)
                object_key = self.object_trackers[np.argmax(compatabilities)].object_key
                tracked_objects += 1

            # or create new tracker 
            else:
                object_key = len(self.object_trackers)
                self.object_trackers.append(ObjectTracker(object_key, image, detection.box))
                new_objects += 1

            # add associated detection
            associated_detections.add(detection, pose_key, object_key)

        # print 
        print('DataAssociation: image had {} detections. {} associated | {} new | Total trackers: {}'.format(
            len(image_detections),
            tracked_objects,
            new_objects,
            len(self.object_trackers),
        ))
        return associated_detections

    def prints(self):
        print('DataAssociation: {} objects and {} trackers'.format(
            len(self.object_trackers),
            len(self.object_trackers)
        ))
    
