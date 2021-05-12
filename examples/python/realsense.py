"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Online Quadric SLAM system
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
import numpy as np
import time
import cv2
import yaml
import argparse

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../quadricslam'))

# import custom python modules
sys.dont_write_bytecode = True
from drawing import CV2Drawing

# detectron
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg

# import gtsam and extension
import gtsam
import gtsam_quadrics

import pyrealsense2 as rs
import atexit
from scipy.optimize import linear_sum_assignment



class RealsenseCamera(object):
    def __init__(self):
        # setup camera streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline_profile = self.pipeline.start(config)
        atexit.register(self.close_camera) # register pipeline stop on exit

        # get camera intrinsics
        color_stream_profile = pipeline_profile.get_stream(rs.stream.color)
        intrinsics = color_stream_profile.as_video_stream_profile().get_intrinsics()
        self.intrinsics = self.intrinsics_to_matrix(intrinsics)

        # get depth scale
        device = pipeline_profile.get_device()
        depth_sensor = device.first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def intrinsics_to_matrix(self, intrinsics):
        """
        intrinsics: realsense object
        matrix: 3x3 camera intrinsics
        """
        matrix = np.zeros((3,3))
        matrix[0,0] = intrinsics.fx
        matrix[1,1] = intrinsics.fy
        matrix[0,2] = intrinsics.ppx
        matrix[1,2] = intrinsics.ppy
        matrix[2,2] = 1.0
        return matrix

    def next_images(self):
        """Blocking, waits for depth and color.
        color: (h,w,3) np.uint8
        depth: (h,w) np.float32 - depth in meters
        """
        finished = False
        while not finished:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            failed = not depth_frame or not color_frame
            finished = not failed

        # convert image to np arrays
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        # convert depth to meters
        depth_meters = np.array(depth*self.depth_scale, np.float32)
        return color, depth_meters

    def close_camera(self):
        print('\nclosing camera')
        self.pipeline.stop()

class OdometryWrapper(object):
    def __init__(self, intrinsics):
        """intrinsics: 3x3 camera calibration"""
        self.odometry = cv2.rgbd.RgbdOdometry_create(intrinsics)
        self.prev_image = None
        self.prev_depth = None

    def compute(self, image, depth):
        """First call will return None
        image = (h,w,1) np.uint8 - grayscale 
        depth = (h,w,1) np.float32 - depth in meters
        odom: 4x4 pose matrix, transform from current frame to previous frame
        """
        if self.prev_image is None:
            self.prev_image = image
            self.prev_depth = depth
            return None

        transform = np.ones((4,4))
        mask = np.ones(image.shape[0:2], np.uint8)
        self.odometry.compute(self.prev_image, self.prev_depth, mask, gray, depth, mask, transform)
        odom = gtsam.Pose3(transform).inverse()

        self.prev_image = image
        self.prev_depth = depth
        return odom

class Associator(object):
    def __init__(self, iou_thresh, calibration):
        self.iou_thresh = iou_thresh
        self.calibration = calibration
        self.n_landmarks = 0

    def associate(self, boxes, quadrics, pose):
        """
        boxes: [gtsam_quadrics.AlignedBox2]
        quadrics: {"quadric_key": gtsam_quadrics.ConstrainedDualQuadric}
        pose: gtsam.Pose3

        returns [quadric_key]
        """

        # handle situation with no quadrics
        if len(quadrics) == 0:
            keys = np.arange(len(boxes)) + self.n_landmarks
            self.n_landmarks + len(boxes)
            return keys
        
        quadric_keys = list(quadrics.keys())
        quadric_values = list(quadrics.values())
        ious = np.zeros((len(boxes), len(quadrics)))

        # calculate iou for each box-quad pair
        for i, box in enumerate(boxes):
            for j, quadric in enumerate(quadric_values):
                dual_conic = gtsam_quadrics.QuadricCamera.project(quadric, pose, self.calibration)
                conic_box = dual_conic.bounds() # NOTE: we can use smartBounds here for more accuracy
                ious[i,j] = box.iou(conic_box)

        # solve optimal assignment problem
        box_indices, quadric_indices = linear_sum_assignment(-ious)

        # check validity of each association
        associated_keys = []
        for i,j in zip(box_indices, quadric_indices):

            # create new landmark
            if ious[i,j] < self.iou_thresh:
                associated_keys.append(self.n_landmarks)
                self.n_landmarks += 1
            else:
                associated_keys.append(quadric_keys[j])

        return associated_keys

class PredictorWrapper(object):
    def __init__(self, model_zoo_path, keep_classes=None):
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file(model_zoo_path))
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5 
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(model_zoo_path)
        self.predictor = DefaultPredictor(cfg)
        self.keep_classes = keep_classes

    def __call__(self, image):
        prediction = self.predictor(image)
        instances = prediction['instances']

        # convert instances to boxes
        boxes = []
        for i in range(len(instances)):
            instance = instances[i]
            class_index = instance.pred_classes.item()
            if self.keep_classes is None or class_index in self.keep_classes:
                box = instance.pred_boxes.tensor.squeeze(0).cpu().numpy()
                gtsam_box = gtsam_quadrics.AlignedBox2(*box)
                boxes.append(gtsam_box)

        # filter partials
        image_bounds = gtsam_quadrics.AlignedBox2(0,0,640,480)
        filter_pixels = 15
        filter_bounds = image_bounds.vector() + np.array([1,1,-1,-1])*filter_pixels
        filter_bounds = gtsam_quadrics.AlignedBox2(filter_bounds)
        boxes = [box for box in boxes if filter_bounds.contains(box)]

        return boxes



def intrinsics_to_gtsam(intrinsics):
    fx = intrinsics[0,0]
    fy = intrinsics[1,1]
    cx = intrinsics[0,2]
    cy = intrinsics[1,2]
    return gtsam.Cal3_S2(fx, fy, 0, cx, cy)

def initialize_quadric(depth, box, camera_pose, calibration, object_depth=0.1):
    """
    depth: (h,w) depth image in meters
    """
    # get average box depth 
    dbox = box.vector().astype('int') # get discrete box bounds
    box_depth = depth[dbox[1]:dbox[3], dbox[0]:dbox[2]].mean()
    
    center = box.center()
    x = (center.x() - calibration.px()) * box_depth / calibration.fx()
    y = (center.y() - calibration.py()) * box_depth / calibration.fy()
    relative_point = gtsam.Point3(x, y, box_depth)
    quadric_center = camera_pose.compose(gtsam.Pose3(camera_pose.rotation(), relative_point))

    tx = (box.xmin() - calibration.px()) * box_depth / calibration.fx()
    ty = (box.ymin() - calibration.py()) * box_depth / calibration.fy()
    radii = np.array([np.abs(tx-x), np.abs(ty-y), object_depth])

    quadric = gtsam_quadrics.ConstrainedDualQuadric(quadric_center, radii)
    return quadric





if __name__ == '__main__': 

    # setup camera 
    camera = RealsenseCamera()

    # setup odometry
    odometry = OdometryWrapper(camera.intrinsics)

    # setup detector
    predictor = PredictorWrapper('COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml') # 41 is mug

    # -------- setup slam system -----------------------------------

    # convert calibration matrix to gtsam
    calibration = intrinsics_to_gtsam(camera.intrinsics)

    # setup associator
    associator = Associator(iou_thresh=0.2, calibration=calibration)

    # setup initializer
    object_depth = 0.1

    # create isam optimizer 
    opt_params = gtsam.ISAM2DoglegParams()
    params = gtsam.ISAM2Params()
    params.setOptimizationParams(opt_params)
    params.setEnableRelinearization(True)
    # params.setRelinearizeThreshold(0.01)
    params.setRelinearizeThreshold(0.1)
    params.setRelinearizeSkip(1)
    params.setCacheLinearizedFactors(False)
    isam = gtsam.ISAM2(params)

    # declare symbol shortcuts
    X = lambda i: int(gtsam.symbol(ord('x'), i))
    Q = lambda i: int(gtsam.symbol(ord('q'), i))

    # declare noise models
    odom_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.1]*6, dtype=np.float))
    box_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([60]*4, dtype=np.float))
    pose_prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.00001]*6, dtype=np.float))

    # create local graph and estimate
    # these variables are cleared every time we update isam
    local_graph = gtsam.NonlinearFactorGraph()
    local_estimate = gtsam.Values()

    # create estimates
    # these variables consistently reflect the best estimate for every variable
    # including variables that have not yet been optimized or constrained
    # while not strictly necessary, this saves us having to check both local_estimate and estimate
    # when we don't optimize every step, and it saves us searching the c++ values
    current_trajectory = {}
    current_quadrics = {}

    # store quadrics until they have been viewed enough times to be constrained
    unconstrained_quadrics = {}

    # add initial pose/prior to estimate
    prior_factor = gtsam.PriorFactorPose3(X(0), gtsam.Pose3(), pose_prior_noise)
    local_graph.add(prior_factor)
    local_estimate.insert(X(0), gtsam.Pose3())
    current_trajectory[0] = gtsam.Pose3()

    # stores all past measurements 
    # so we can add measurements once a quadric is constrained
    # and to recall measurements for initialization purposes
    detection_history = []

    step = 1

    # -----------------------------------------------------------------


    while True:
        # get new images
        color, depth = camera.next_images()

        # convert image to greyscale
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        # calculate odometry
        odom = odometry.compute(gray, depth)
        if odom is None: 
            continue
        odom = gtsam.Pose3(odom)

        # run detector 
        boxes = predictor(color)

        # ------------------------------------------------------

        # get the current pose_key
        pose_key = step

        previous_pose = current_trajectory[pose_key-1] # get previous pose from storage
        camera_pose = previous_pose.compose(odom) # compound odometry to global pose
        local_estimate.insert(X(pose_key), camera_pose) # add pose estimate to values
        current_trajectory[pose_key] = camera_pose # add pose to current estimate 
        odom_factor = gtsam.BetweenFactorPose3(X(pose_key-1), X(pose_key), odom, odom_noise)
        local_graph.add(odom_factor) # add odometry factor to graph

        # associate detections
        associated_keys = associator.associate(boxes, current_quadrics, camera_pose)

        # wrap boxes with keys 
        associated_boxes = []
        for box, quadric_key in zip(boxes, associated_keys):
            associated_boxes.append({
                'box': box,
                'quadric_key': quadric_key,
                'pose_key': pose_key,
                'used': False,
            })

        # add associated measurements to history
        detection_history += associated_boxes

        # initialize new landmarks
        new_boxes = [f for f in associated_boxes if f['quadric_key'] not in current_quadrics.keys()]
        for detection in new_boxes:
            box = detection['box']
            quadric_key = detection['quadric_key']
            quadric = initialize_quadric(depth, box, camera_pose, calibration, object_depth)
            current_quadrics[quadric_key] = quadric

            # queue quadric to be added to problem when constrained
            unconstrained_quadrics[quadric_key] = quadric 

        # add initialized landmarks to values (if constrained)
        temporary_copy = unconstrained_quadrics.copy()
        for quadric_key, quadric in unconstrained_quadrics.items():
            quadric_measurements = [d for d in detection_history if d['quadric_key'] == quadric_key]
            if len(quadric_measurements) > 3:
                quadric.addToValues(local_estimate, Q(quadric_key))
                temporary_copy.pop(quadric_key)
        unconstrained_quadrics = temporary_copy

        # add measurements to graph if quadric is initialized and constrained
        for detection in detection_history:
            if detection['used']:
                continue

            box = detection['box']
            quadric_key = detection['quadric_key']
            pose_key = detection['pose_key']

            # add to graph if quadric is constrained
            if quadric_key not in unconstrained_quadrics.keys():
                detection['used'] = True
                bbf = gtsam_quadrics.BoundingBoxFactor(box, calibration, X(pose_key), Q(quadric_key), box_noise, "STANDARD")
                local_graph.add(bbf)


        # if not step%10:
        if True:

            # add local graph and estimate to isam
            isam.update(local_graph, local_estimate)
            estimate = isam.calculateEstimate()

            # clear graph/estimate
            local_graph.resize(0)
            local_estimate.clear()

            # update the estimated quadrics and trajectory
            for i in range(estimate.keys().size()):
                key = estimate.keys().at(i)
                if chr(gtsam.symbolChr(key)) == 'q':
                    quadric = gtsam_quadrics.ConstrainedDualQuadric.getFromValues(estimate, key)
                    current_quadrics[gtsam.symbolIndex(key)] = quadric
                elif chr(gtsam.symbolChr(key)) == 'x':
                    current_trajectory[gtsam.symbolIndex(key)] = estimate.atPose3(key)


        # visualize current view into map
        image = color.copy()
        drawing = CV2Drawing(image)
        for frame in associated_boxes:
            text = '{}'.format(frame['quadric_key'])
            drawing.box_and_text(frame['box'], (0,0,255), text, (0,0,0))

        # NOTE: get newest pose estimate
        camera_pose = current_trajectory[max(current_trajectory.keys())]
        for quadric in current_quadrics.values():
            drawing.quadric(camera_pose, quadric, calibration, (255,0,255))

        cv2.imshow('current view', image)
        cv2.waitKey(1)

        step += 1
        # ------------------------------------------------------




# CHOICES:
# 1. offline/online
# 2. class-based vs classless
# 3. optimize/update every step? or update every x steps
# 4. single-view init vs SVD
#       - SVD requires a) DA that can associate without map
#       - b) checking enough measurements to initialize 
#       - c) assumes there may be some measurements associated to quadrics we've seen before that we didn't initialize
#       - d) requires storing all measurements for initialization
#       - e) single-view requires: storing quadrics before adding to values


# UPDATE:
# - filter
# - data association
# - compound odom onto last pose 
# - add odom/prior onto last pose
# - check which objects can be initialized
# - add new quadrics to graph (if constrained+initialized)
# - add measurements (if not previously added) if quadric is in values
# - optimize


# required:
# - store detections to add after quad is initialized and constrained 
# - store initialized quadric keys (or lookup from last estimate)
# - store previous pose (or lookup from last estimate)
# - store frame_n / pose_key (or count pose values)


# NOTES:
# can do purely map based DA if single-view init and only add quads to values/graph when constrained
















# qslam.update()
# adds new odometry to graph
# adds new pose estimate to values
# runs measurements through data-association
# records associated measurments
# tries to initialize new landmarks
# adds new measurements/landmarks to graph/values if valid



# initializer
# NOTE: check if we have any object keys in history that are uninitialized (needs full measurement history, current quadrics)
# NOTE: check they will be constrained if init
# NOTE: check they have been initialized correctly 
# NOTE: either provide a way for user to get measurement history / quadric keys
# NOTE: OR get the user to overload the initialization method / pass the function in
# new init methods have different constraints (n views, depth info, etc)


# NOTE: cant we just make a base qslam system and derive to modify odometry / initialization
# what if the user wants to know the current pose_key?
# associator also needs copy of detections history
# QSLAM should HAVE-A associator/initializer and they can derive?


# COMPONENTS: core, odometry, detection, data-association, initialization, visualization, evaluation 



# camera (settings): 
    # .get_next_images (void): returns rgb/depth
    # .intrinsics: 3x3 matrix
# odometry (settings, intrinsics): 
    # .compute (color, depth): returns relative pose
# detector (settings): takes in image(s) and returns boxes
    # .detect (color): returns [boxes]


# new image, odom, detections!


# associator (settings)
    # .associate (history, color, boxes): returns associated boxes
    # probably stores some version of history (cnn codes / full detections) to aid future associations

    # NOTE: might need .get_past_measurements()

# initializer (settings)
    # .initialize (history, current_quadrics, new_associated_boxes): returns new_quadrics
        # tries to initialize objects that haven't been initialized before
        # checks to make sure object is constrained and initialization is valid

    # NOTE: needs .get_past_measurements(), .get_current_quadric_keys()

# system (settings)
    # .update (odom, associated_boxes, new_quadrics, history)
    # .optimize (void): quadrics, trajectory

    # only adds unused measurements to graphs
    # trusts initializer that quadrics are new, constrained, valid



# if init strategy is single-view, should be initialized immedietly and added when constrained
# does the user need to know the current pose_key? 
    # either give qslam.update() all the information and let it do the background magic
    # or only give it things it can immedietly add to graph/values and optimize

# quadricslam controls when to initialize new landmarks
# 

# trajectory: list[gtsam.Pose3]
# odometry: list[gtsam.Pose3]

