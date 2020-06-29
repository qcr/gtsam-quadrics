# import standard libraries
import os
import sys
import numpy as np
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')
import cv2

# import ros libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from detection_msgs.msg import AssociatedAlignedBox2D
from detection_msgs.msg import AssociatedAlignedBox2DArray
from detection_msgs.msg import AlignedBox2D
from detection_msgs.msg import AlignedBox2DArray
from cv_bridge import CvBridge
import message_filters

# import custom python modules
sys.path.append('/home/lachness/git_ws/quadricslam/ros/src/rosquadricslam/rosquadricslam')
sys.path.append('/home/lachness/git_ws/quadricslam/examples/python/example_frontend')
sys.dont_write_bytecode = True
from data_association import DataAssociation
from dataset_interfaces.scenenet_dataset import SceneNetDataset
from base.containers import Trajectory, Quadrics
from visualization.drawing import CV2Drawing

# import gtsam and extension
import gtsam
import quadricslam


"""
The only way to provide re-association is to store key inside object. 
Allowing multiple boxes at the same key requires us to always count how many boxes we get from .at
"""

class AssociatedPose(gtsam.Pose3):
    def __init__(self, pose, pose_key=-1):
        super().__init__(pose)
        self.pose_key = pose_key
        self.used = False

class Poses(object):
    def __init__(self, poses=None):
        if poses is None:
            self._poses = []
        elif type(poses) is list:
            self._poses = poses
        elif type(poses) is Boxes:
            self._poses = poses._poses
        else:
            raise TypeError('incorrect Poses init type')

    def __len__(self):
        return len(self._poses)

    def __getitem__(self, index):
        return self._poses[index]

    def add_pose(self, pose, pose_key=-1):
        self._poses.append(AssociatedPose(pose, pose_key))

    def add_associated_pose(self, apose):
        self._poses.append(apose)

    def add_poses(self, other):
        self._poses += other._poses

    def at(self, pose_key):
        return Poses([pose for pose in self._poses if pose.pose_key==pose_key])

    def at_keys(self, pose_keys):
        return Poses([pose for pose in self._poses if pose.pose_key in pose_keys])

    def unused(self):
        return Poses([pose for pose in self._poses if pose.used==False])


class AssociatedBox(quadricslam.AlignedBox2): 
    def __init__(self, box, pose_key=-1, object_key=-1, time=None):
        super().__init__(box)
        self.time = time
        self.pose_key = pose_key
        self.object_key = object_key
        self.used = False

class Boxes(object):
    """
    Key idea (sorry for the pun):
        Use the meta wrapper as exactly what it is. Storage for keys. 
        Keep it explicit, don't try and hide it away. 
    
    Store objects with 2 keys.
    Can store with either one of the keys unassociated. 
    Can have multiple objects at the same key pair. 
    Provides:
        boxes.atKey1()
        boxes.atKey2()
        boxes.at(k1,k2)
        for (k1,k2),box in boxes.items():
    Allows you to reassociate keys.
    Allows you to mark objects as having been used. 
    """
    def __init__(self, boxes=None):
        if boxes is None:
            self._boxes = []
        elif type(boxes) is list:
            self._boxes = boxes
        elif type(boxes) is Boxes:
            self._boxes = boxes._boxes
        else:
            raise TypeError('incorrect Boxes init type')

    def __len__(self):
        return len(self._boxes)

    def __getitem__(self, index):
        return self._boxes[index]

    def add_box(self, box, pose_key=-1, object_key=-1, time=None):
        self._boxes.append(AssociatedBox(box, pose_key, object_key, time))

    def add_associated_box(self, abox):
        self._boxes.append(abox)

    def add_boxes(self, boxes):
        self._boxes += boxes._boxes

    def at(self, pose_key, object_key):
        return Boxes([box for box in self._boxes if box.pose_key==pose_key])
    
    def at_pose(self, pose_key):
        return Boxes([box for box in self._boxes if box.pose_key==pose_key])

    def at_object(self, object_key):
        """ Slow. 
        Use groupby_objects instead of looping through this function """
        return Boxes([box for box in self._boxes if box.object_key==object_key])

    def unused(self):
        return Boxes([box for box in self._boxes if box.used==False])

    def associated(self):
        return Boxes([box for box in self._boxes if box.object_key!=-1])

    def pose_keys(self):
        return [box.pose_key for box in self._boxes]

    def object_keys(self):
        return [box.object_key for box in self._boxes]


    







class ROSQuadricSLAM(Node):
    def __init__(self):
        # set node name
        super().__init__('ROSQuadricSLAM')

        # settings
        UPDATE_TIME = 1.0  # seconds
        POSE_SIGMA = 0.001
        BOX_SIGMA = 20.0

        # store constants
        self.X = lambda i: int(gtsam.symbol(ord('x'), i))
        self.Q = lambda i: int(gtsam.symbol(ord('q'), i))

        # start subscriptions
        self.pose_subscription = message_filters.Subscriber(self, PoseStamped, 'poses')
        self.detection_subscription = message_filters.Subscriber(self, AlignedBox2DArray, 'detections')
        self.image_subscription = message_filters.Subscriber(self, Image, 'image')
        self.time_synchronizer = message_filters.TimeSynchronizer([self.image_subscription, self.pose_subscription, self.detection_subscription], 10)
        self.time_synchronizer.registerCallback(self.update)

        # store Image->msg converter
        self.bridge = CvBridge()

        # get camera calibration
        # dataset = SceneNetDataset(
        #     dataset_path = '/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/data/train',
        #     protobuf_folder = '/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/data/train_protobufs',
        #     reader_path = '/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/scenenet_pb2.py',
        #     shapenet_path = '/media/lachness/DATA/Datasets/ShapeNet/ShapeNetCore.v2'
        # )
        # self.calibration = dataset.load_calibration()
        self.calibration = gtsam.Cal3_S2(
            311.48, 
            311.26, 
            0.0, 
            309.43, 
            237.72
        )

        # store true data
        # sequence = dataset[0]
        # self.true_trajectory = sequence.true_trajectory
        # self.true_quadrics = sequence.true_quadrics

        # set noise models
        self.prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([POSE_SIGMA]*6, dtype=np.float))
        self.bbox_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([BOX_SIGMA]*4, dtype=np.float))

        # set graph / estimate
        self.graph = gtsam.NonlinearFactorGraph()
        self.estimate = gtsam.Values()

        # set parameters / optimizer
        self.parameters = gtsam.ISAM2Params()
        self.parameters.setRelinearizeSkip(100)
        # self.parameters.setEnableRelinearization(False)
        self.parameters.print_("ISAM2 Parameters")
        self.isam = gtsam.ISAM2(self.parameters)

        # set measurement storage 
        self.images = dict()
        self.poses = Poses()
        self.boxes = Boxes()
        self.initial_quadrics = Quadrics()

        # store DA tracker
        self.data_association = DataAssociation()
        self.associated_times = []

        # create timer for update function
        # self.timer = self.create_timer(UPDATE_TIME, self.update)
        
        # convert from time stamp to pose_keys
        # needed because we recieve poses/images/detections with times
        # but end up storing them in Poses/graph with keys
        self.pose_keys = dict()

        # store current estimates to draw each frame
        self.current_trajectory = Poses()
        self.current_quadrics = Quadrics()
        print('\n~ Awaiting Measurements ~')

        # self.quadric = quadricslam.ConstrainedDualQuadric(gtsam.Rot3(), gtsam.Point3(0.,0.,0.), np.array([0.02,0.02,0.02]))
        # self.current_quadrics.add(self.quadric, 1)
        self.only_track = False


    def msg2boxes(self, msg):
        return [quadricslam.AlignedBox2(msg_box.xmin, msg_box.ymin, msg_box.xmax, msg_box.ymax) for msg_box in msg.boxes]

    def msg2pose(self, msg):
        point = gtsam.Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        rot = gtsam.Rot3.Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        return gtsam.Pose3(rot, point)

    def msg2image(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
    def msg2time(self, msg):
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec)*1e-9




    def draw_detections(self):
        for time, image in self.images.items():
            boxes = [b for b in self.boxes if b.time == time]
            for box in boxes:
                cv2.rectangle(image, (int(box.xmin()),int(box.ymin())), (int(box.xmax()),int(box.ymax())), (255,255,0), 1)
            cv2.imshow('test', image)
            cv2.waitKey(100)

    def time2key(self, float_time):
        try:
            return self.pose_keys[float_time]
        except KeyError as key_error:
            pose_key = len(self.pose_keys)
            self.pose_keys[float_time] = pose_key
            return pose_key

    def update(self, image_msg, pose_msg, detections_msg):
        # self.get_logger().info('Started Update function')

        # print('image time: {}'.format(self.msg2time(image_msg)))
        # print('pose time: {}'.format(self.msg2time(pose_msg)))
        # print('boxes time: {}'.format(self.msg2time(detections_msg)))

        # convert msgs to data
        image = self.msg2image(image_msg)
        camera_pose = self.msg2pose(pose_msg).inverse()
        boxes = self.msg2boxes(detections_msg)
        float_time = self.msg2time(detections_msg)
        pose_key = self.time2key(float_time)

        # draw detections
        # draw current map 
        img = image.copy()
        drawing = CV2Drawing(img)
        for quadric in self.current_quadrics.data():
            try:
                drawing.quadric(camera_pose, quadric, self.calibration, (255,0,255))
            except:
                print('failed with pose: ')
                print(camera_pose)
                exit()
        cv2.imshow('Current view', img)
        cv2.waitKey(1)

        # if self.only_track:
        #     return

        # convert boxes to pose stamped Boxes
        new_boxes = Boxes()
        for box in boxes:
            new_boxes.add_box(box, pose_key, -1, float_time)

        # associate new measurements with existing keys
        self.data_association.track(image, new_boxes)
        # for box in new_boxes:
        #     box.object_key = 1

        # store new boxes and pose for later initialization and factor adding
        self.boxes.add_boxes(new_boxes)
        self.poses.add_pose(camera_pose, pose_key)

        # create local graph and estimate
        local_graph = gtsam.NonlinearFactorGraph()
        local_estimate = gtsam.Values()
        
        # add new pose measurements to graph / estimate
        local_estimate.insert(self.X(pose_key), camera_pose)
        prior_factor = gtsam.PriorFactorPose3(self.X(pose_key), camera_pose, self.prior_noise)
        local_graph.add(prior_factor)

        # check if we can initialize any new objects
        object_keys, counts = np.unique(self.boxes.object_keys(), return_counts=True)
        for object_key, count in zip(object_keys, counts):

            if self.only_track:
                continue
            

            # dont use uninitialized objects
            if object_key == -1:
                raise Exception("key with -1")

            # no need to re-initialize objects
            if object_key in self.initial_quadrics.keys():
                continue

            # initialize object if seen enough
            if count >= 5:

                object_boxes = self.boxes.at_object(object_key)
                object_poses = self.poses.at_keys(object_boxes.pose_keys())
                quadric_matrix = self.quadric_SVD(object_poses, object_boxes, self.calibration)
                quadric = quadricslam.ConstrainedDualQuadric.constrain(quadric_matrix)

                # check quadric is okay
                if self.is_okay(quadric, object_poses, self.calibration):

                    # add quadric to values and history
                    quadric.addToValues(local_estimate, self.Q(object_key))
                    self.initial_quadrics.add(quadric, object_key)
                    self.only_track = True


        # add measurements if unused
        for box in self.boxes.unused():

            # dont use uninitialized objects
            if box.object_key == -1:
                raise Exception("key with -1")
                
            # add measurements if initialized 
            if box.object_key in self.initial_quadrics.keys():
                bbf = quadricslam.BoundingBoxFactor(box, self.calibration, self.X(box.pose_key), self.Q(box.object_key), self.bbox_noise)
                bbf.addToGraph(local_graph)
                box.used = True


        # check problem
        # self.check_problem(self.graph, self.estimate)

        # use local graph / estimate to update isam2
        self.isam.update(local_graph, local_estimate)

        # append local graph / estimate to full graph
        self.graph.push_back(local_graph)
        self.estimate.insert(local_estimate)

        # calculate current estimate
        current_estimate = self.isam.calculateEstimate()

        # draw current map view
        self.current_trajectory = Trajectory.from_values(current_estimate)
        self.current_quadrics = Quadrics.from_values(current_estimate)


            
    def check_problem(self, graph, estimate): 

        print('Checking problem for missing DOF')

        # extract variables from estimate
        quadrics = Quadrics.from_values(estimate)
        # box_factors = [quadricslam.BoundingBoxFactor.getFromGraph(graph, i) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
        box_factor_keys = [int(gtsam.symbolIndex(graph.at(i).keys().at(1))) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
        # pose_factors = [graph.at(i) for i in range(graph.size()) if graph.at(i).keys().size() == 1 and chr(gtsam.symbolChr(graph.at(i).keys().at(0))) == 'x']

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
            break
        
            print('Object Q{}'.format(object_key))

            # extract bbfs targeting this object
            bbfs = [f for f in box_factors if gtsam.symbolIndex(f.keys().at(1)) == object_key]

            # get associated poses
            pose_keys = [gtsam.symbolIndex(f.keys().at(0)) for f in bbfs]
            poses = [trajectory.at(pose_key) for pose_key in pose_keys]

            # calculate error with jacobian 
            for bbf, pose, pose_key in zip(bbfs, poses, pose_keys):
                error = bbf.evaluateError(pose, quadric)
                H1 = bbf.evaluateH1(pose, quadric)
                H2 = bbf.evaluateH2(pose, quadric)
                H1dof = np.sum(H1.any(1))
                H2dof = np.sum(H2.any(1))

                print('  factor q{}->x{} has {},{} DOF'.format(
                    object_key, pose_key,
                    H1dof, H2dof
                ))

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



def main(args=None):
    # init ros
    rclpy.init(args=args)

    # create node
    system = ROSQuadricSLAM()

    # spin node
    rclpy.spin(system)

    # shutdown 
    system.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
