# import standard libraries
import os
import sys
import numpy as np
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')

# import ros libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from detection_msgs.msg import AssociatedAlignedBox2D
from detection_msgs.msg import AssociatedAlignedBox2DArray

# import custom python modules
sys.path.append('/home/lachness/git_ws/quadricslam/ros/src/py_detector/py_detector')
sys.path.append('/home/lachness/git_ws/quadricslam/examples/python/example_frontend')
sys.dont_write_bytecode = True
from dataset_interfaces.scenenet_dataset import SceneNetDataset
from base.containers import Boxes, Trajectory, Quadrics
from visualization.drawing import MPLDrawing

# import gtsam and extension
import gtsam
import quadricslam


class ROSQuadricSLAM(Node):
    def __init__(self):
        # set node name
        super().__init__('ROSQuadricSLAM')

        # store constants
        self.X = lambda i: int(gtsam.symbol(ord('x'), i))
        self.Q = lambda i: int(gtsam.symbol(ord('q'), i))

        # start subscriptions
        self.pose_subscription = self.create_subscription(PoseStamped, 'poses', self.pose_callback, 10)
        self.detection_subscription = self.create_subscription(AssociatedAlignedBox2DArray, 'detections', self.detection_callback, 10)

        # get camera calibration
        dataset = SceneNetDataset(
            dataset_path = '/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/data/train',
            protobuf_folder = '/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/data/train_protobufs',
            reader_path = '/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/scenenet_pb2.py',
            shapenet_path = '/media/lachness/DATA/Datasets/ShapeNet/ShapeNetCore.v2'
        )
        self.calibration = dataset.load_calibration()

        # store true data
        sequence = dataset[0]
        self.true_trajectory = sequence.true_trajectory
        self.true_quadrics = sequence.true_quadrics

        # set noise models
        self.prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.001]*6, dtype=np.float))
        self.bbox_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.5]*4, dtype=np.float))

        # set graph / estimate
        self.graph = gtsam.NonlinearFactorGraph()
        self.estimate = gtsam.Values()
        self.graph_full = gtsam.NonlinearFactorGraph()
        self.estimate_full = gtsam.Values()

        # set parameters / optimizer
        self.parameters = gtsam.ISAM2Params()
        self.parameters.setRelinearizeSkip(100)
        # self.parameters.setEnableRelinearization(False)
        self.parameters.print_("ISAM2 Parameters")
        self.isam = gtsam.ISAM2(self.parameters)

        # set measurement storage 
        self.poses = dict()
        self.boxes = Boxes()
        self.pose_history = dict()
        self.box_history = Boxes()
        self.initial_quadrics = Quadrics()

        # create timer for update function
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.update)
        
        
        print('\n~ Awaiting Measurements ~')


    def update(self):
        self.get_logger().info('updating isam2 with measurements')
        
        # add new pose measurements to graph / estimate
        for pose_key, pose in self.poses.items():
            self.estimate.insert(self.X(pose_key), pose)
            self.estimate_full.insert(self.X(pose_key), pose)
            prior_factor = gtsam.PriorFactorPose3(self.X(pose_key), pose, self.prior_noise)
            self.graph.add(prior_factor)
            self.graph_full.add(prior_factor)

        # add new measurements to history
        self.pose_history.update(self.poses)
        
        # clear pose measurements
        self.poses = dict()

        # check if we can initialize any new objects
        object_keys, counts = np.unique(self.boxes.object_keys(), return_counts=True)
        for object_key, count in zip(object_keys, counts):

            # no need to re-initialize objects
            if object_key in self.initial_quadrics.keys():
                continue

            if count >= 5:
                object_boxes = self.boxes.at_object(object_key)
                # # need to use pose history as old measurements would have had the corrosponding poses cleared
                object_poses = [self.pose_history[pose_key] for pose_key in object_boxes.pose_keys()]
                quadric_matrix = self.quadric_SVD(object_poses, object_boxes, self.calibration)
                quadric = quadricslam.ConstrainedDualQuadric.constrain(quadric_matrix)

                # quadric = self.true_quadrics.at(object_key)

                # check quadric is okay
                if self.is_okay(quadric, object_poses, self.calibration):

                    # add quadric to values and history
                    quadric.addToValues(self.estimate, self.Q(object_key))
                    quadric.addToValues(self.estimate_full, self.Q(object_key))
                    self.initial_quadrics.add(quadric, object_key)

        # add measurements if initialized 
        for (pose_key, object_key), box in self.boxes.items():
            if object_key in self.initial_quadrics.keys():
                bbf = quadricslam.BoundingBoxFactor(box, self.calibration, self.X(pose_key), self.Q(object_key), self.bbox_noise)
                bbf.addToGraph(self.graph)
                bbf.addToGraph(self.graph_full)
                self.boxes.remove(pose_key, object_key)
                self.box_history.add(box, pose_key, object_key)


        # check problem
        self.check_problem(self.graph_full, self.estimate_full)

        # use local graph / estimate to update isam2
        self.isam.update(self.graph, self.estimate)

        # clear graph / estimate for next iteration
        self.graph.resize(0)
        self.estimate.clear()

        # calculate current estimate
        current_estimate = self.isam.calculateEstimate()


        # # lvm optimisation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # params = gtsam.LevenbergMarquardtParams()
        # params.setVerbosityLM("SUMMARY")    # SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA : VALUES, ERROR 
        # params.setMaxIterations(20)
        # params.setlambdaInitial(1e-5)
        # params.setlambdaUpperBound(1e10)
        # params.setlambdaLowerBound(1e-8)
        # params.setRelativeErrorTol(1e-5)
        # params.setAbsoluteErrorTol(1e-5)
  
        # # create optimizer
        # optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph_full, self.estimate_full, params)

        # # run optimizer
        # print('starting optimization')
        # current_estimate = optimizer.optimize()
        # print('finished optimization')
        # # lvm optimisation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        # draw current estimate 
        current_trajectory = Trajectory.from_values(current_estimate)
        current_quadrics = Quadrics.from_values(current_estimate)
        initial_trajectory = Trajectory.from_values(self.estimate_full)
        initial_quadrics = Quadrics.from_values(self.estimate_full)
        if len(current_trajectory) > 3 and len(current_quadrics) > 0:
            mpldrawing = MPLDrawing('result')
            mpldrawing.plot_result([self.true_trajectory, current_trajectory, initial_trajectory], [self.true_quadrics, current_quadrics, initial_quadrics], ['g', 'm', 'r'], ['true_estimate', 'current_estimate', 'initial_estimate'])

            
    def check_problem(self, graph, estimate): 

        print('Checking problem for missing DOF')

        # extract variables from estimate
        trajectory = Trajectory.from_values(estimate)
        quadrics = Quadrics.from_values(estimate)
        # box_factors = [quadricslam.BoundingBoxFactor.getFromGraph(graph, i) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
        box_factor_keys = [int(gtsam.symbolIndex(graph.at(i).keys().at(1))) for i in range(graph.size()) if graph.at(i).keys().size() == 2 and chr(gtsam.symbolChr(graph.at(i).keys().at(1))) == 'q']
        # pose_factors = [graph.at(i) for i in range(graph.size()) if graph.at(i).keys().size() == 1 and chr(gtsam.symbolChr(graph.at(i).keys().at(0))) == 'x']

        pose_factor_keys = [int(gtsam.symbolIndex(graph.at(i).keys().at(0))) for i in range(graph.size()) if graph.at(i).keys().size() == 1 and chr(gtsam.symbolChr(graph.at(i).keys().at(0))) == 'x']
        pose_estimate_keys = trajectory.keys()

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

    def detection_callback(self, msg):
        self.get_logger().info('Receieved detection @ {}'.format(msg.header.frame_id))

        # convert msg to AlignedBox2 and store
        pose_key = int(msg.header.frame_id)
        for msg_box in msg.boxes:
            box = quadricslam.AlignedBox2(msg_box.xmin, msg_box.ymin, msg_box.xmax, msg_box.ymax)
            self.boxes.add(box, pose_key, int(msg_box.key))

    def pose_callback(self, msg):
        self.get_logger().info('Receieved pose @ {}'.format(msg.header.frame_id))

        # convert msg to Pose3 and store
        pose_key = int(msg.header.frame_id)
        point = gtsam.Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        rot = gtsam.Rot3.Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        pose = gtsam.Pose3(rot, point)
        self.poses[pose_key] = pose

    def quadric_SVD(self, poses, object_boxes, calibration):
        """ calculates quadric_matrix using SVD """

        # iterate through box/pose data
        planes = []
        for box, pose in zip(object_boxes.data(), poses):

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
