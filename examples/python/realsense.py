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
import atexit
import pyrealsense2 as rs
from scipy.optimize import linear_sum_assignment

# detectron
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../quadricslam'))

# import custom python modules
sys.dont_write_bytecode = True
from drawing import CV2Drawing

# import gtsam and extension
import gtsam
import gtsam_quadrics




class RealsenseCamera(object):
    """
    Example of a typical camera object. 

    Attributes
    ----------
    intrinsics : 3x3 intrinsics calibration matrix
        
    """
    
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
        """Converts the pyrealsense2 intrinsics object to 3x3 matrix.

        Parameters
        ----------
        intrinsics : pyrealsense2.intrinsics

        Returns
        -------
        ndarray
            3x3 calibtration intrinsics
        """
        matrix = np.zeros((3,3))
        matrix[0,0] = intrinsics.fx
        matrix[1,1] = intrinsics.fy
        matrix[0,2] = intrinsics.ppx
        matrix[1,2] = intrinsics.ppy
        matrix[2,2] = 1.0
        return matrix

    def next_images(self):
        """Waits for the next camera images.
        
        Returns
        -------
        color : ndarray (h,w,3) np.uint8
            Color image from camera
        depth : ndarray (h,w) np.float32
            Depth image in meters.
        
        Notes
        -----
        This function is blocking and will wait for the next images.
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
    """Example odometry system. 
    
    Parameters
    ----------
    intrinsics: ndarray
        the 3x3 intrinsic calibration matrix

    """
    
    def __init__(self, intrinsics):
        self.odometry = cv2.rgbd.RgbdOdometry_create(intrinsics)
        self.prev_image = None
        self.prev_depth = None

    def compute(self, gray, depth):
        """Estimate odometry from new rgb+depth images. 

        Parameters
        ----------
        gray : ndarray (h,w) np.uint8
            Grayscale image from camera.
        depth : ndarray (h,w) np.float32
            Depth image in meters.

        Returns
        -------
        odom: gtsam.pose3
            Transform from current frame to previous frame.

        Notes
        -----
        First call to .compute() will set initial images and return None.
        """
        if self.prev_image is None:
            self.prev_image = gray
            self.prev_depth = depth
            return None

        transform = np.ones((4,4))
        mask = np.ones(gray.shape[0:2], np.uint8)
        self.odometry.compute(self.prev_image, self.prev_depth, mask, gray, depth, mask, transform)
        odom = gtsam.Pose3(transform).inverse()

        self.prev_image = gray
        self.prev_depth = depth
        return odom

class Associator(object):
    """Example data-association for bounding-box measurements.

    This data-association object projects the map into the current pose,
    and looks for 2Dboxes that overlap with the projected quadrics. 

    Parameters
    ----------
    iou_thresh : float
        Only measurements with iou > iou_thresh, will be associated with an existing landmark.
    calibration : gtsam.Cal3_S2
        Intrinsic camera calibration.

    """
    
    def __init__(self, iou_thresh, calibration):
        self.iou_thresh = iou_thresh
        self.calibration = calibration
        self.n_landmarks = 0

    def associate(self, boxes, quadrics, pose):
        """Associates 2Dboxes to quadric landmarks.

        Each 2Dbox is associated with an object key,
        either an existing quadric landmark, or a new landmark. 

        Parameters
        ----------
        boxes : List[gtsam_quadrics.AlignedBox2]
            A list of 2D bounding boxes from the current view.
        quadrics : Dict[int, gtsam_quadrics.ConstrainedDualQuadric]
            A dictionary of key-quadric pairs representing the current map.
        pose : gtsam.Pose3
            The current camera pose. 

        Returns
        -------
        List[int] 
            The associated object-key for each of the input bounding boxes.

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
    """Example object-detection system. 

    Parameters
    ----------
    model_zoo_path : str
        The path to the detector weights and config in model_zoo.
    keep_classes : List[int]
        List of specific object classes to keep based on their index. 
        I.e, a coco-trained detector index 41 is cup

    """
    
    def __init__(self, model_zoo_path, keep_classes=None):
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file(model_zoo_path))
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5 
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(model_zoo_path)
        self.predictor = DefaultPredictor(cfg)
        self.keep_classes = keep_classes

    def __call__(self, image):
        """Predict bounding-boxes from image. 

        Returns
        -------
        List[gtsam_quadrics.AlignedBox2]
            A list of 2D boxes representing the bounds of each detected object. 

        Notes
        -----
        By default, we also eliminate detections close to the image boundaries. 
        """
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
    """Converts 3x3 calibration matrix to gtsam.Cal3_S2 object."""
    fx = intrinsics[0,0]
    fy = intrinsics[1,1]
    cx = intrinsics[0,2]
    cy = intrinsics[1,2]
    return gtsam.Cal3_S2(fx, fy, 0, cx, cy)

def initialize_quadric(depth, box, camera_pose, calibration, object_depth=0.1):
    """Initialize a quadric from a single view.

    Estimates an inital quadric from a pose, box and depth image. 

    Parameters
    ----------
    depth : (h,w)
        Depth image in meters. 
    box : gtsam_quadrics.AlignedBox2
        The 2D box outlining the object.
    camera_pose : gtsam.Pose3
        The current camera pose.
    calibration : gtsam.Cal3_S2
        The intrinsic camera calibration.
    object_depth : float
        Approximate length of the object along the camera axis.
    """
    # get average box depth 
    dbox = box.vector().astype('int') # get discrete box bounds
    box_depth = depth[dbox[1]:dbox[3], dbox[0]:dbox[2]].mean()

    # compute the 3D point corrosponding to the box center 
    center = box.center()
    x = (center.x() - calibration.px()) * box_depth / calibration.fx()
    y = (center.y() - calibration.py()) * box_depth / calibration.fy()
    relative_point = gtsam.Point3(x, y, box_depth)
    quadric_center = camera_pose.transformFrom(relative_point)

    # compute quadric rotation using .Lookat
    up_vector = camera_pose.transformFrom(gtsam.Point3(0,-1,0))
    quadric_rotation = gtsam.SimpleCamera.Lookat(camera_pose.translation(), quadric_center, up_vector).pose().rotation()
    quadric_pose = gtsam.Pose3(quadric_rotation, quadric_center)

    # compute the quadric radii from the box shape
    tx = (box.xmin() - calibration.px()) * box_depth / calibration.fx()
    ty = (box.ymin() - calibration.py()) * box_depth / calibration.fy()
    radii = np.array([np.abs(tx-x), np.abs(ty-y), object_depth])

    quadric = gtsam_quadrics.ConstrainedDualQuadric(quadric_pose, radii)
    return quadric





if __name__ == '__main__': 

    # setup the camera feed 
    camera = RealsenseCamera()

    # setup odometry system for inference 
    odometry = OdometryWrapper(camera.intrinsics)

    # setup object detectory
    predictor = PredictorWrapper('COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml') 


    # -------- setup slam system -----------------------------------

    # convert calibration matrix to gtsam
    calibration = intrinsics_to_gtsam(camera.intrinsics)

    # setup associator
    associator = Associator(iou_thresh=0.2, calibration=calibration)

    # setup quadric initializer
    object_depth = 0.1

    # create isam optimizer 
    opt_params = gtsam.ISAM2DoglegParams()
    params = gtsam.ISAM2Params()
    params.setOptimizationParams(opt_params)
    params.setEnableRelinearization(True)
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

    # create local graph and estimate (cleared when we update isam)
    local_graph = gtsam.NonlinearFactorGraph()
    local_estimate = gtsam.Values()

    # create storage for traj/map estimates
    current_trajectory = {}
    current_quadrics = {}

    # store quadrics until they have been viewed enough times to be constrained (>3)
    unconstrained_quadrics = {}

    # add initial pose/prior to estimate
    prior_factor = gtsam.PriorFactorPose3(X(0), gtsam.Pose3(), pose_prior_noise)
    local_graph.add(prior_factor)
    local_estimate.insert(X(0), gtsam.Pose3())
    current_trajectory[0] = gtsam.Pose3()

    # stores all measurements to be recalled when a quadric new quadric is constrained
    detection_history = []

    step = 1

    # -----------------------------------------------------------------


    while True:

        # get new images
        color, depth = camera.next_images()

        # calculate odometry from greyscale
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        odom = odometry.compute(gray, depth)
        if odom is None: 
            continue
        odom = gtsam.Pose3(odom)

        # run detector 
        boxes = predictor(color)

        # ------------------------------------------------------

        # get the current pose_key
        pose_key = step
        


        # get previous pose from storage
        previous_pose = current_trajectory[pose_key-1] 

        # compound odometry to global pose
        camera_pose = previous_pose.compose(odom) 

        # add pose estimate to values and current estimateo (for initialization)
        local_estimate.insert(X(pose_key), camera_pose) 
        current_trajectory[pose_key] = camera_pose 

        # add odometry factor to graph
        odom_factor = gtsam.BetweenFactorPose3(X(pose_key-1), X(pose_key), odom, odom_noise)
        local_graph.add(odom_factor) 


        # associate boxes -> quadrics 
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


        # draw the current object detections
        image = color.copy()
        drawing = CV2Drawing(image)
        for frame in associated_boxes:
            text = '{}'.format(frame['quadric_key'])
            drawing.box_and_text(frame['box'], (0,0,255), text, (0,0,0))

        # visualize current view into map
        camera_pose = current_trajectory[max(current_trajectory.keys())]
        for quadric in current_quadrics.values():
            drawing.quadric(camera_pose, quadric, calibration, (255,0,255))
        cv2.imshow('current view', image)
        cv2.waitKey(1)

        step += 1
        # ------------------------------------------------------

