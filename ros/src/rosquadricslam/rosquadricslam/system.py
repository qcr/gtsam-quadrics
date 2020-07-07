# import standard libraries
import os
import sys
import numpy as np
import time
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')
import cv2
import atexit
import yaml
import argparse

# import ros libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from detection_msgs.msg import ObjectDetectionArray
from cv_bridge import CvBridge
import message_filters

# import custom python modules
sys.path.append('/home/lachness/git_ws/quadricslam/ros/src/rosquadricslam/rosquadricslam')
sys.path.append('/home/lachness/git_ws/quadricslam/examples/python/example_frontend')
sys.dont_write_bytecode = True
from data_association import DataAssociation
from dataset_interfaces.scenenet_dataset import SceneNetDataset
from visualization.drawing import CV2Drawing
from base.containers import Trajectory, Quadrics, Detections, ObjectDetection

# import gtsam and extension
import gtsam
import quadricslam







class ROSQuadricSLAM(Node):
    def __init__(self, args):
        # set node name
        super().__init__('ROSQuadricSLAM')

        # settings
        self.config_path = args.config_path
        self.depth = args.depth
        self.record = args.record
        self.minimum_views = args.minimum_views
        self.initialization_method = args.initialization_method

        POSE_SIGMA = 0.001
        BOX_SIGMA = 10.0

        # parse settings
        if self.config_path is None:
            raise RuntimeError('ros parameter "config" is required')

        # load camera calibration
        self.calibration = self.load_camera_calibration(self.config_path)

        # load class names
        classes_path = '/home/lachness/git_ws/PyTorch-YOLOv3/data/coco.names'
        classes_fp = open(classes_path, "r")
        self.class_names = classes_fp.read().split("\n")[:-1]

        # store constants
        self.X = lambda i: int(gtsam.symbol(ord('x'), i))
        self.Q = lambda i: int(gtsam.symbol(ord('q'), i))

        # start subscriptions
        self.pose_subscription = message_filters.Subscriber(self, PoseStamped, 'poses')
        self.detection_subscription = message_filters.Subscriber(self, ObjectDetectionArray, 'detections')
        self.image_subscription = message_filters.Subscriber(self, Image, 'image')
        self.time_synchronizer = message_filters.TimeSynchronizer([self.image_subscription, self.pose_subscription, self.detection_subscription], self.depth)
        self.time_synchronizer.registerCallback(self.update)

        # store Image->msg converter
        self.bridge = CvBridge()

        # set noise models
        self.prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([POSE_SIGMA]*6, dtype=np.float))
        self.bbox_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([BOX_SIGMA]*4, dtype=np.float))

        # set graph / estimate
        self.graph = gtsam.NonlinearFactorGraph()
        self.estimate = gtsam.Values()

        # set dogleg parameters
        opt_params = gtsam.ISAM2DoglegParams()
        # opt_params = gtsam.ISAM2GaussNewtonParams()

        # set isam parameters
        parameters = gtsam.ISAM2Params()
        parameters.setOptimizationParams(opt_params)
        parameters.setRelinearizeSkip(1)
        parameters.setRelinearizeThreshold(0.01)
        # parameters.setEnableRelinearization(False)
        parameters.print_("ISAM2 Parameters")

        # create isam2 optimizer
        self.isam = gtsam.ISAM2(parameters)

        # set measurement storage 
        self.images = dict()
        self.poses = Trajectory()
        self.detections = Detections()
        self.initial_quadrics = Quadrics()

        # convert from time stamp to pose_keys
        self.pose_keys = dict()

        # store current estimates to draw each frame
        self.current_trajectory = Trajectory()
        self.current_quadrics = Quadrics()

        # initialize data-association module
        self.data_association = DataAssociation(self.current_quadrics, self.calibration)

        # store update count 
        self.count = 0

        # prepare video capture
        if self.record:
            self.video_writer = cv2.VideoWriter('good_performance.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 12.0, (640, 480))
            atexit.register(self.video_writer.release)
        print('\n~ Awaiting Measurements ~')

        # debugging 
        # atexit.register(self.debug)

    def load_camera_calibration(self, path, no_distortion=True):
        """ Loads gtsam calibration from openvslam config format """
        camera_config = yaml.safe_load(open(path, 'r'))

        camera_model = gtsam.Cal3_S2
        calibration_list = [
            camera_config['Camera.fx'],
            camera_config['Camera.fy'],
            0.0,
            camera_config['Camera.cx'],
            camera_config['Camera.cy'],
        ]

        if no_distortion:
            return camera_model(*calibration_list)

        if 'Camera.k1' in camera_config:
            camera_model = gtsam.Cal3DS2
            calibration_list += [
                camera_config['Camera.k1'],
                camera_config['Camera.k2'],
            ]

        if 'Camera.p1' in camera_config:
            calibration_list += [
                camera_config['Camera.p1'],
                camera_config['Camera.p2'],
            ]

        return camera_model(*calibration_list)

    def debug(self):
        for quadric_key, quadric in self.current_quadrics.items():
            for pose_key, pose in self.current_trajectory.items():
                conic = quadricslam.QuadricCamera.project(quadric, pose, self.calibration)

                if conic.isEllipse():
                    add = 'an ellipse'
                else:
                    add = 'not elliptical!'
                print('(x{},q{}) conic is {}'.format(pose_key, quadric_key, add))
                print('quadric.isBehind',quadric.isBehind(pose))
                print('quadric.contains',quadric.contains(pose))

        good_quadric = self.current_quadrics.at(1)
        bad_quadric = self.current_quadrics.at(0)

        print('good_quadric_pose', good_quadric.pose().matrix())
        print('good_quadric_radii', good_quadric.radii())
        print('bad_quadric_pose', bad_quadric.pose().matrix())
        print('bad_quadric_radii', bad_quadric.radii())
        
        # far_pose = gtsam.SimpleCamera.Lookat(gtsam.Point3(5,5,5), bad_quadric.pose().translation(),  gtsam.Point3(0,0,1)).pose()
        # conic = quadricslam.QuadricCamera.project(bad_quadric, far_pose, self.calibration)
        # print('test:', conic.isEllipse())
        # print('test:', conic.bounds().vector())
        # img = self.images[pose_key]
        # drawing = CV2Drawing(img)
        # drawing.quadric(far_pose, bad_quadric, self.calibration, (255,0,255))
        # cv2.imshow('Current view', img)
        # cv2.waitKey(0)

        
    def msg2detections(self, msg, filters=None):
        detections = []
        for detection in msg.detections:
            if filters is not None:
                filter_indicies = [self.class_names.index(filter) for filter in filters]
            if filters is None or np.argmax(detection.scores) in filter_indicies:
                box = quadricslam.AlignedBox2(detection.box.xmin, detection.box.ymin, detection.box.xmax, detection.box.ymax) 
                detection = ObjectDetection(box, detection.objectness, detection.scores)
                detections.append(detection)
        return detections

    def msg2pose(self, msg):
        point = gtsam.Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        rot = gtsam.Rot3.Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        return gtsam.Pose3(rot, point)

    def msg2image(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
    def msg2time(self, msg):
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec)*1e-9

    def time2key(self, float_time):
        try:
            return self.pose_keys[float_time]
        except KeyError as key_error:
            pose_key = len(self.pose_keys)
            self.pose_keys[float_time] = pose_key
            return pose_key







    def update(self, image_msg, pose_msg, detections_msg):
        self.count += 1
        update_start = time.time()
        self.get_logger().info('Update started')

        # convert msgs to data
        image = self.msg2image(image_msg)
        camera_pose = self.msg2pose(pose_msg).inverse()
        float_time = self.msg2time(detections_msg)
        pose_key = self.time2key(float_time)
        image_detections = self.msg2detections(detections_msg, filters=['cup', 'bowl'])
        # image_detections = self.msg2detections(detections_msg)

        # draw detections
        img = image.copy()
        drawing = CV2Drawing(img)
        for detection in image_detections:
            scores = detection.scores
            text = '{}:{:.2f}'.format(self.class_names[np.argmax(scores)], np.max(scores))
            drawing.box_and_text(detection.box, (0,0,255), text, (0,0,0))

        # draw current map 
        for quadric in self.current_quadrics.values():
            drawing.quadric(camera_pose, quadric, self.calibration, (255,0,255))
        cv2.imshow('Current view', img)
        cv2.waitKey(1)
        if self.record:
            self.video_writer.write(img)






        # associate new measurements with existing keys
        da_start = time.time()
        associated_detections = self.data_association.associate(image, image_detections, camera_pose, pose_key, visualize=True, verbose=True)
        da_end = time.time()






        # store new boxes and pose for later initialization and factor adding
        self.detections.add_detections(associated_detections)
        self.poses.add(camera_pose, pose_key)
        self.images[pose_key] = image

        # create local graph and estimate
        local_graph = gtsam.NonlinearFactorGraph()
        local_estimate = gtsam.Values()
        
        # add new pose measurements to graph / estimate
        local_estimate.insert(self.X(pose_key), camera_pose)
        prior_factor = gtsam.PriorFactorPose3(self.X(pose_key), camera_pose, self.prior_noise)
        local_graph.add(prior_factor)

        # check if we can initialize any new objects
        for object_key, object_detections in self.detections.per_object():

            # no need to re-initialize objects
            # TODO: use keys from current estimate 
            if object_key in self.initial_quadrics.keys():
                continue
            
            # initialize object if seen enough
            # TODO: use current trajectory instead of initial poses?
            quadric = self.initialize_quadric(object_key, object_detections, self.poses, local_estimate)

            # continue if not correctly initialized 
            if quadric is None: 
                continue

            # add quadric to values 
            quadric.addToValues(local_estimate, self.Q(object_key))

            # add quadric to storage (not needed in future)
            self.initial_quadrics.add(quadric, object_key)


        # add measurements if unused
        for (pose_key, object_key), detection in self.detections.items():

            # only add new measurements
            if self.detections.is_used(pose_key, object_key):
                continue            

            # add measurements if initialized 
            # TODO: use keys from current estimate
            if object_key in self.initial_quadrics.keys():
                bbf = quadricslam.BoundingBoxFactor(detection.box, self.calibration, self.X(pose_key), self.Q(object_key), self.bbox_noise)
                bbf.addToGraph(local_graph)
                self.detections.set_used(True, pose_key, object_key)

        # append local graph / estimate to full graph
        self.graph.push_back(local_graph)
        self.estimate.insert(local_estimate)

        # check problem
        # self.check_reprojection_errors(self.graph, self.estimate)
        # self.check_problem(self.graph, self.estimate)
        # self.print_dof(self.graph, self.estimate)

        # use local graph / estimate to update isam2
        self.isam.update(local_graph, local_estimate)

        # calculate current estimate
        current_estimate = self.isam.calculateEstimate()

        # check if estimate has changed
        # new_trajectory = Trajectory.from_values(current_estimate)
        # new_quadrics = Quadrics.from_values(current_estimate)
        # for object_key in new_quadrics.keys():
        #     if object_key in self.current_quadrics.keys():
        #         new_quadric = new_quadrics.at(object_key)
        #         old_quadric = self.current_quadrics.at(object_key)
        #         local_diff = np.sum(new_quadric.localCoordinates(old_quadric))
        #         if local_diff > 0:
        #             print('object {} changed by {}'.format(object_key, local_diff))
        update_end = time.time()

        
        
        
        
        # update current estimate 
        self.current_trajectory = Trajectory.from_values(current_estimate)
        self.current_quadrics.clear()
        self.current_quadrics.update(Quadrics.from_values(current_estimate))
        extracting_end = time.time()


        # print timings
        # self.get_logger().info('Update lasted {:.3f} s'.format(extracting_end-update_start))
        # print('pre-da:  {:.3f} s'.format(da_start-update_start))
        # print('da:      {:.3f} s'.format(da_end-da_start))
        # print('opt:     {:.3f} s'.format(update_end-da_end))
        # print('extract: {:.3f} s'.format(extracting_end-update_end))
        # print('')
            


    def initialize_quadric(self, object_key, object_detections, current_trajectory, local_estimate):
        """ 
        Attempts to initialize the quadric according to self.initialization_method.
        Returns None if quadric could not be initialized 
        """
        if self.initialization_method == 'SVD':
            if len(object_detections) >= self.minimum_views:

                object_boxes = [d.box for d in object_detections.values()]
                pose_keys = object_detections.keys()
                object_poses = current_trajectory.at_keys(pose_keys)
                quadric_matrix = self.quadric_SVD(object_poses, object_boxes, self.calibration)
                quadric = quadricslam.ConstrainedDualQuadric.constrain(quadric_matrix)

                # check quadric is okay
                if self.is_okay(quadric, object_poses, self.calibration):
                    return quadric

        else:
            if len(object_detections) >= self.minimum_views:
                abox = list(object_detections.values())[0]
                apose_key = list(object_detections.keys())[0]
                apose = current_trajectory.at(apose_key)
                quadric_pose = apose.compose(gtsam.Pose3(gtsam.Rot3(),gtsam.Point3(0,0,0.1)))
                quadric = quadricslam.ConstrainedDualQuadric(quadric_pose, np.array([0.01]*3))
                return quadric
                # self.initial_quadrics.add(quadric, object_key)
                # quadric.addToValues(local_estimate, self.Q(object_key))
        return None



    def print_dof(self, graph, estimate):
        print('-------------printing factor dofs')
        box_factors = [quadricslam.BoundingBoxFactor.getFromGraph(graph, i) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
        quadrics = Quadrics.from_values(estimate)
        poses = Trajectory.from_values(estimate)

        for factor in box_factors:
            error = factor.error(estimate)
            quadric = quadrics.at(gtsam.symbolIndex(factor.objectKey()))
            pose = poses.at(gtsam.symbolIndex(factor.poseKey()))
            error = factor.evaluateError(pose, quadric)
            H1 = factor.evaluateH1(pose, quadric)
            H2 = factor.evaluateH2(pose, quadric)
            print(H1)
        print('-------------printing factor dofs--------------')




    def check_reprojection_errors(self, graph, estimate):
        box_factors = [quadricslam.BoundingBoxFactor.getFromGraph(graph, i) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
        quadrics = Quadrics.from_values(estimate)
        poses = Poses.from_values(estimate)

        if len(box_factors)==0:
            return
        
        pose_keys = []
        object_keys = []
        errors = []
        for bbf in box_factors:
            pose_key = gtsam.symbolIndex(bbf.keys().at(0))
            object_key = gtsam.symbolIndex(bbf.keys().at(1))
            pose = poses.at(pose_key)
            quadric = quadrics.at(object_key)
            error = bbf.evaluateError(pose, quadric).sum()

            pose_keys.append(pose_key)
            object_keys.append(object_key)
            errors.append(error)

        max_index = np.argmax(errors)
        pose = poses.at(pose_keys[max_index])[0]
        quadric = quadrics.at(object_keys[max_index])
        bbf = box_factors[max_index]
        box = self.boxes.at(pose_keys[max_index], object_keys[max_index])[0]
        image = self.images[pose_keys[max_index]].copy()


        drawing = CV2Drawing(image)
        drawing.quadric(pose, quadric, self.calibration, (0,0,255))
        drawing.box_and_text(box, (0,255,255), 'err', (0,0,0))
        cv2.imshow('high error', image)
        cv2.waitKey(1)
            
    def check_problem(self, graph, estimate): 

        # extract variables from estimate
        quadrics = Quadrics.from_values(estimate)
        box_factors = [quadricslam.BoundingBoxFactor.getFromGraph(graph, i) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
        box_factor_keys = [int(gtsam.symbolIndex(graph.at(i).keys().at(1))) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
        # pose_factors = [graph.at(i) for i in range(graph.size()) if graph.at(i).keys().size() == 1 and chr(gtsam.symbolChr(graph.at(i).keys().at(0))) == 'x']
        full_poses = Trajectory.from_values(estimate)

        pose_factor_keys = [int(gtsam.symbolIndex(graph.at(i).keys().at(0))) for i in range(graph.size()) if graph.at(i).keys().size() == 1 and chr(gtsam.symbolChr(graph.at(i).keys().at(0))) == 'x']
        pose_estimate_keys = [int(gtsam.symbolIndex(estimate.keys().at(i))) for i in range(estimate.keys().size()) if chr(gtsam.symbolChr(estimate.keys().at(i))) == 'x']

        # ensure each pose factor has estimate
        leftover_keys = list(set(pose_factor_keys).symmetric_difference(pose_estimate_keys))
        if len(leftover_keys) > 0:
            print('pose keys {} exist as factor or estimate'.format(leftover_keys))
            exit()
            
        # ensure each object_key has a quadric and at least 3 measurements
        object_keys = np.unique(quadrics.keys() + box_factor_keys)
        for object_key in object_keys:
            if object_key not in quadrics.keys():
                print('object_key {} not in quadrics'.format(object_key))
                exit()
            if len([k for k in box_factor_keys if k==object_key]) < 5:
                print('object_key {} not as 5+ factors'.format(object_key))
                exit()

            
        # check each quadric has enough DOF to be constrained 
        for object_key, quadric in quadrics.items():
        

            # extract bbfs targeting this object
            bbfs = [f for f in box_factors if gtsam.symbolIndex(f.keys().at(1)) == object_key]

            # get associated poses
            pose_keys = [gtsam.symbolIndex(f.keys().at(0)) for f in bbfs]
            poses = [full_poses.at(pose_key) for pose_key in pose_keys]

            # calculate error with jacobian 
            n_H2_rows = []
            for bbf, pose, pose_key in zip(bbfs, poses, pose_keys):
                error = bbf.evaluateError(pose, quadric)
                H1 = bbf.evaluateH1(pose, quadric)
                H2 = bbf.evaluateH2(pose, quadric)

                H1dof = np.sum(H1.any(1))
                H2dof = np.sum(H2.any(1))
                n_H2_rows.append(H2dof)

                # print('  factor q{}->x{} has {},{} DOF'.format(
                #     object_key, pose_key,
                #     H1dof, H2dof
                # ))

            n_dof = np.sum(n_H2_rows)
            n_dof_expected = len(n_H2_rows)*4
            if n_dof != n_dof_expected:
                print('Object {} is missing {} dof'.format(object_key, n_dof_expected-n_dof))

    def quadric_SVD(self, poses, object_boxes, calibration):
        """ calculates quadric_matrix using SVD """

        # iterate through box/pose data
        planes = []
        for box, pose in zip(object_boxes, poses):

            # calculate boxes lines
            lines = box.lines()

            # convert Vector3Vector to list
            lines = [lines.at(i) for i in range(lines.size())]

            # calculate projection matrix
            P = quadricslam.QuadricCamera.transformToImage(pose, calibration).transpose()

            # project lines to planes
            planes += [P @ line for line in lines]

        # create A matrix
        A = np.asarray([np.array([p[0]**2,  2*(p[0]*p[1]),  2*(p[0]*p[2]),  2*(p[0]*p[3]),
                                                p[1]**2,  	2*(p[1]*p[2]),  2*(p[1]*p[3]),
                                                                p[2]**2,  	2*(p[2]*p[3]),
                                                                               p[3]**2]) for p in planes])

        # solve SVD for Aq = 0, which should be equal to p'Qp = 0
        _,_,V = np.linalg.svd(A, full_matrices=True)
        q = V.T[:, -1]

        # construct quadric
        dual_quadric = np.array([[q[0], q[1], q[2], q[3]],
                                [q[1], q[4], q[5], q[6]],
                                [q[2], q[5], q[7], q[8]],
                                [q[3], q[6], q[8], q[9]]])

        return dual_quadric

    def is_okay(self, quadric, poses, calibration):
        """
        Checks quadric is valid:
            quadric constrained correctly
            paralax > threshold
            reprojections valid in each frame 
                quadric infront of camera : positive depth 
                camera outside quadric
                conic is an ellipse 
            ensure views provide enough DOF (due to edges / out of frame)
        """
        for pose in poses:

            # quadric must have positive depth
            if quadric.isBehind(pose):
                return False

            # camera pose must be outside quadric 
            if quadric.contains(pose):
                return False

            # conic must be valid and elliptical 
            conic = quadricslam.QuadricCamera.project(quadric, pose, calibration)
            if conic.isDegenerate():
                return False
            if not conic.isEllipse():
                return False
                
        return True



def main(main_args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', dest='config_path', type=str, required=True ,
                        help='path to the camera configuartion file')
    parser.add_argument('--depth', dest='depth', type=int, default=10, 
                        help='the queue depth to store topic messages')
    parser.add_argument('--record', dest='record', type=bool, default=False, 
                        help='boolean to record map visualization')
    parser.add_argument('--views', dest='minimum_views', type=int, default=5, 
                        help='minimum views required to initialize object')
    parser.add_argument('--init', dest='initialization_method', type=str, choices=['SVD', 'other'], default='SVD', 
                        help='method to use for initialization')
    args = parser.parse_args()
    
    # init ros
    rclpy.init(args=main_args)

    # create node
    system = ROSQuadricSLAM(args)

    # spin node
    rclpy.spin(system)

    # shutdown 
    system.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
