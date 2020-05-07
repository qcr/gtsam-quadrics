"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved
Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)

See LICENSE for the license information

Dataset interface.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""

import os
import sys
import importlib
import numpy as np

class SceneNetDataset(object):
    """
    Train: 16,000 videos
    Val: 1,000 videos

    NOTE: VAL has bad light positions...
    NOTE: VAL object vertices and boxes look okay..
    """

    def __init__(self, dataset_path, protobuf_folder, reader_path):
        """
        :param - dataset_path - path to val or train data
        :param - protobuf_folder - path to .pb protobufs
        :param - reader_path - path to scenenet_pb2.py file
        """
        self.dataset_path = dataset_path
    
        # load path to each video
        self.sequence_paths = self.load_sequence_paths(dataset_path)

        # load protobuf reader 
        self.sn = self.load_protobuf_reader(reader_path)

        # load shapenet
        self.protobuf_folder = protobuf_folder

        # setup storage for protobuf data
        self.sequences_data = {}

    def __len__(self):
        return len(self.sequence_paths)

    def __getitem__(self, index):
        return self.get_sequence(index)

    def get_sequence(self, index):
        """ load videos path and protofbuf data """
        sequence_path = self.sequence_paths[index]
        sequence_data = self.load_sequence_data(sequence_path)
        sequence = SceneNetSequence(sequence_path, sequence_data, self.sn)
        return sequence

    def load_sequence_paths(self, dataset_path):
        """ from dataset path get all video paths """
        render_folders = [fn for fn in os.listdir(dataset_path)
                                if os.path.isdir(os.path.join(dataset_path, fn))]

        video_paths = [os.path.join(dataset_path, pf, vf) for pf in render_folders
                                    for vf in os.listdir(os.path.join(dataset_path, pf))
                                        if (not '.' in vf)]
        return video_paths

    def load_protobuf_reader(self, protobuf_def_path):
        # extract folder / file information
        protobuf_def_folder = protobuf_def_path.replace(protobuf_def_path.split('/')[-1], '')
        protobuf_def_file = protobuf_def_path.replace(protobuf_def_folder, '')

        # load protobuf definition
        sys.path.append(os.path.realpath(protobuf_def_folder))
        sn = importlib.import_module(protobuf_def_file.replace('.py',''))
        return sn

    def load_protobuf(self, sequence_path):
        """ loads full protobuf required for this video """
        protobuf_path = self.get_protobuf_path(sequence_path)

        trajectories = self.sn.Trajectories()
        with open(protobuf_path,'rb') as f:
            trajectories.ParseFromString(f.read())

        for trajectory in trajectories.trajectories:
            render_path = str(trajectory.render_path)
            sequence_path = os.path.join(self.dataset_path, render_path)
            self.sequences_data[sequence_path] = trajectory

    def get_protobuf_path(self, sequence_path):
        """ converts a video path to relevent protobuf file path """
        render_folder = sequence_path.split('/')[-2]
        protobuf_name = 'scenenet_rgbd_{}_{}.pb'.format(self.dataset_name, render_folder)
        protobuf_path = os.path.join(self.protobuf_folder, protobuf_name)
        return protobuf_path

    def load_sequence_data(self, sequence_path):
        """ pulls sequence data from storage or loads from file """
        # load protobuf if required
        if (sequence_path not in self.sequences_data):
            self.load_protobuf(sequence_path)
        sequence_data = self.sequences_data[sequence_path]
        return sequence_data

    @staticmethod
    def load_calibration():
        pixel_width = 320; pixel_height = 240
        vfov = 45; hfov = 60
        fx = (pixel_width/2.0)/math.tan(math.radians(hfov/2.0))
        fy = (pixel_height/2.0)/math.tan(math.radians(vfov/2.0))
        s = 0.0; cx = pixel_width/2.0; cy = pixel_height/2.0
        camera_intrinsics = gtsam.Cal3_S2(fx, fy, s, cx, cy)
        return camera_intrinsics


class SceneNetSequence(Sequence):
    """ 
    Holds video information
    Loads as required:
    - 
    """
    calibration = SceneNetDataset.load_calibration()
    def __init__(self, sequence_path, sequence_data, sn):
        self.shapenet_path = '/media/feyre/DATA1/Datasets/ShapeNet/ShapeNetCore.v2'
        self.sequence_path = sequence_path
        self.sequence_data = sequence_data
        self.sn = sn




class SceneNetPlayer(InteractivePlayer):
    def __init__(self, dataset, player_name='map_player'):
        super(SceneNetPlayer, self).__init__(player_name)

        self.dataset = dataset

        self.video_index = 0
        self.image_index = 0

        self.sequence = self.dataset[self.video_index]

    # def load_sequence(self, video_index):
    #     self.sequence = self.dataset[self.video_index]
    #     self.images = self.sequence.images
    #     self.calibration = self.sequence.camera_intrinsics
    #     self.trajectory = self.sequence.true_trajectory
    #     self.object_bounds = self.sequence.instance_cuboids
    #     self.object_verticies = self.sequence.instance_vertices
    #     self.true_instances = self.sequence.true_instances


    def handle_input(self, key):
        if (key == 'a'):
            self.image_index = np.clip(self.image_index - 1, 0, len(self.sequence.images)-1)
        if (key == 'd'):
            self.image_index = np.clip(self.image_index + 1, 0, len(self.sequence.images)-1)
        if (key == 'q'):
            self.video_index = np.clip(self.video_index - 1, 0, len(self.dataset)-1)
            self.sequence = self.dataset[self.video_index]
            self.image_index = 0
        if (key == 'e'):
            self.video_index = np.clip(self.video_index + 1, 0, len(self.dataset)-1)
            self.sequence = self.dataset[self.video_index]
            self.image_index = 0

    def update(self):
        """ draw measurements at image/pose """
        image = self.images[self.image_index]
        current_pose = self.trajectory[self.image_index]
        detections = self.sequence.true_detections.at_pose(key)
        image_detections = self.true_instances.at_pose_key(pose_key)

        # for det in image_detections:
        #     drawing.cv2_draw_box_and_text(image, det.box.vector, box_color=(0,255,0), text='{}'.format(det.object_key), text_color=(0,0,0))

        # draw lights
        for light_position in self.sequence.light_positions:
            draw_3D_point(current_pose, self.calibration, light_position, image, radius=3, color=(255,0,0), thickness=-1)

        # draw close trajectory
        # for camera_position in self.trajectory[self.image_index:self.image_index+10]:
        #     draw_3D_point(current_pose, self.calibration, camera_position.translation(), image, radius=3, color=(0,0,255), thickness=-1)

        # for camera_position in self.trajectory:
        #     draw_3D_point(current_pose, self.calibration, camera_position.translation(), image, radius=2, color=(0,0,255), thickness=-1)

        # draw selected objects vertices
        # object = list(self.objects.values())[self.object_index]
        # object_key = list(self.objects.keys())[self.object_index]


        #
        # draw everything
        #

        # # draw full trajectory
        # draw_3D_trajectory(current_pose, self.calibration, self.trajectory, image, radius=2, color=(0,0,255), thickness=-1)
        # ## draw selected object vertices
        object_vertices = list(self.object_verticies.values())[self.object_index]
        draw_3D_points(current_pose, self.calibration, object_vertices, image, radius=2, color=(0,255,255), thickness=-1)
        # ## draw all objects bounds in 3D
        for object_bounds in self.object_bounds.objects.values():
            draw_3D_bounds(current_pose, self.calibration, object_bounds.vector, image)




        # show image
        cv2.imshow(self.player_name, image)
        # cv2.imshow(self.player_name+'_occfree', occim)



class Trajectory(object):
    """ 
    Implicit data association. 
    Keys stored in index of pose.
    Cast to gtsamKey when converted to factor.
    """
    X = lambda i: int(gtsam.symbol(ord('x'), i))
    def __init__(self, poses):
        self._poses = poses

    def __len__(self):
        return len(self._poses)

    def __getitem__(self, index):
        return self._poses[index]

    def data(self):
        """ returns list of poses """
        return self._poses

    def at_keys(self, keys):
        """ returns a new trajectory only with these keys """
        return Trajectory([self._poses[key] for key in keys])

    def as_odometry(self):
        relative_poses = [self._poses[i].between(self._poses[i+1]) for i in range(len(self._poses)-1)]
        return Odometry(relative_poses)

    def add_prior(self, graph, noisemodel):
        prior_factor = gtsam.PriorFactorPose3(Trajectory.X(0), self._poses[0], noisemodel)
        graph.add(prior_factor)
        return graph

    def add_estimate(self, values):
        for index, pose in enumerate(self._poses):
            values.insert(Trajectory.X(index), pose)
        return values

class Odometry(object):
    """ 
    Implicit data association. 
    Keys stored in index of rpose.
    Cast to gtsamKey when converted to factor.
    """
    def __init__(self, rposes):
        self._rposes = rposes

    def __len__(self):
        return len(self._rposes)

    def __getitem__(self, index):
        return self._rposes[index]

    def data(self):
        return self._rposes

    def as_trajectory(self, reference=gtsam.Pose3()):
        global_poses = [reference]
        for rpose in self._rposes:
            global_poses.append( global_poses[-1].compose(rpose) )
        return Trajectory(global_poses)

    def add_factors(self, graph, noisemodel):
        for index, rpose in enumerate(self._rposes):
            odometry_factor = gtsam.BetweenFactorPose3(Trajectory.X(index), Trajectory.X(index+1), rpose, noisemodel)
        return graph


class Quadrics(object):
    """
    Explicit data association.
    Keys stored as dictionary keys
    Mapped to gtsamKey when added to estimate.
    """
    Q = lambda i: int(gtsam.symbol(ord('q'), i))
    def __init__(self):
        self._quadrics = dict()

    def add(self, quadric, key):
        self._quadrics[key] = quadric

    def add_estimate(self, values):
        for key, quadric in self._quadrics.items():
            values.insert(Quadrics.Q(key), quadric)
        return values


from collections import defaultdict

# TODO: allow multiple boxes at same key set
# TODO: allow boxes with undefined object key
# TODO: provide fast lookup for boxes.at(pose) and boxes.at(quadric)
# TODO: provide fast access to list[boxes]
# TODO: provide fast access to boxes.pose_keys(), boxes.object_keys()
class Boxes(object):
    def __init__(self, boxes=None):
        self._boxes = boxes
        if boxes is None:
            self._boxes = dict() # acts as a mock sparse 2D array

    def add(self, box, pose_key, object_key):
        self._boxes[pose_key, object_key] = box

    def __len__(self):
        return len(self._boxes.values())

    def __getitem__(self, index):
        """ returns ((pose_key, object_key), box) | O(1)"""
        return self._boxes.items()[index]

    def data(self):
        """ returns [boxes] | O(1)"""
        return self._boxes.data()

    def pose_keys(self):
        """ return in order with .data() the pose_keys | O(n)"""
        return [keypair[0] for keypair in self._boxes.keys()]

    def object_keys(self):
        """ return in order with .data() the object_keys | O(n)"""
        return [keypair[1] for keypair in self._boxes.keys()]

    def at(self, pose_key, object_key):
        return self._boxes[pose_key, object_key]

    def at_pose(self, pose_key):
        """ returns a Boxes object of boxes at pose_key | O(n)"""
        return Boxes({k:v for k,v in self._boxes.items() if k[0] == pose_key})

    def at_object(self, object_key):
        """ returns a Boxes object of boxes at object_key | O(n)"""
        return Boxes({k:v for k,v in self._boxes.items() if k[1] == pose_key})

    # def groupby_object(self):
    #     pass

    # def groupby_pose(self):
    #     pass

    





class System(object):
    """
    Python front-end to build graph/estimate from dataset. 
    """
    X = lambda i: int(gtsam.symbol(ord('x'), i))
    Q = lambda i: int(gtsam.symbol(ord('q'), i))
    L = lambda i: int(gtsam.symbol(ord('l'), i))
    ZERO = 0.000001 

    @staticmethod
    def build_graph(dataset):
        """
        Dataset contains:
        * noisyOdometry
        * noisyBoxes
        * calibration 
        """

        # create empty graph / estimate
        graph = gtsam.NonlinearFactorGraph()
        initial_estimate = gtsam.Values()

        # declare noise models
        odom_sigma = 0.01; box_sigma = 3
        noise_zero = gtsam.noiseModel_Diagonal.Sigmas(np.array([1e-12]*6, dtype=np.float))
        odometry_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([odom_sigma]*3 + [odom_sigma]*3, dtype=np.float))
        bbox_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([box_sigma]*4, dtype=np.float))
        X = lambda i: int(gtsam.symbol(ord('x'), i))
        Q = lambda i: int(gtsam.symbol(ord('q'), i))

        # initialize trajectory and quadrics
        initial_trajectory = dataset.noisy_odometry.as_trajectory()
        initial_quadrics = System.initialize_quadrics(initial_trajectory, dataset.noisy_boxes, dataset.calibration)

        # add prior pose
        initial_trajectory.add_prior(graph, noise_zero)

        # add odometry measurements
        dataset.noisy_odometry.add_factors(graph, odometry_noise)

        # add bbox measurements
        dataset.noisy_boxes.add_factors(graph, bbox_noise)

        # add initial pose estimates
        initial_trajectory.add_estimates(initial_estimate)
        
        # add initial landmark estimates
        initial_quadrics.add_estimates(initial_estimate)

        return graph, initial_estimate

    @staticmethod
    def initialize_quadrics(trajectory, boxes, calibration):
        # quadrics = 

        # loop through object keys
        for object_key in np.unique(boxes.object_keys()):

            # get all detections at object key
            object_boxes = boxes.at_object(object_key)

            # get the poses associated with each detection 
            pose_keys = object_boxes.pose_keys()
            poses = trajectory.at_keys(pose_keys)

            # ensure quadric seen from > 3 views
            if len(np.unique(pose_keys)) < 3:
                continue

            quadric = System.initialize_quadric(poses, object_boxes, calibration)
            # quadrics.add(quadric, object_key)

    @staticmethod
    def initialize_quadric(poses, object_boxes, calibration):

        # iterate through box/pose data
        planes = []
        for box, pose in zip(poses.data(), object_boxes.data()):

            # convert Vector3Vector to list
            lines = [lines.at(i) for i in range(lines.size())]

            # calculate projection matrix
            P = quadricslam.QuadricCamera.transformToImage();

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

        # constrain quadric
        constrained_dual_quadric = System.constrain_quadric(dual_quadric)

        return constrained_dual_quadric
            
    @staticmethod
    def constrain_quadric(dual_quadric_matrix):
        # calculate point quadric
        point_quadric = np.linalg.inv(dual_quadric_matrix)

        # normalize point quadric
        point_quadric = point_quadric/point_quadric[-1,-1]

        # calculate shape
        lambdaa = np.linalg.eigh(point_quadric[:3,:3])[0]
        lambdaa = [complex(ele) for ele in lambdaa]
        s = np.sqrt( -( np.linalg.det(point_quadric) / np.linalg.det(point_quadric[:3,:3]) ) *  1.0/lambdaa)
        s = np.abs(s)

        # calculate normal dual quadric for translation
        Q = dual_quadric_matrix/dual_quadric_matrix[-1,-1]

        # calculate translation
        t = np.asarray([Q[0,3]/Q[-1,-1], Q[1,3]/Q[-1,-1], Q[2,3]/Q[-1,-1]])

        # calculate rotation
        r1 = np.linalg.eigh(point_quadric[:3,:3])[1]

        # store as quadric
        ellipsoid = gtsam_quadrics.ConstrainedDualQuadric(gtsam.Rot3(r1), gtsam.Point3(t), s)
        ellipsoid_vector = gtsam_quadrics.ConstrainedDualQuadric.LocalCoordinates(ellipsoid)


        # check if rotation valid
        if np.isclose(np.linalg.det(r1), -1.0) or np.any(np.isinf(ellipsoid_vector)) or np.any(np.isnan(ellipsoid_vector)):

            # identify bad rotation
            # print('[SVD] estimated quadric rotation invalid, resolving by inverting axis')

            # flip if rotation is improper
            AxisFlip = np.eye(3); AxisFlip*=-1.0;
            r2 = r1.dot(AxisFlip)
            ellipsoid_2 = gtsam_quadrics.ConstrainedDualQuadric(gtsam.Rot3(r2), gtsam.Point3(t), s)
            ellipsoid_vector_2 = gtsam_quadrics.ConstrainedDualQuadric.LocalCoordinates(ellipsoid_2)

            # check if still broken
            if np.isclose(np.linalg.det(r2), -1.0) or np.any(np.isinf(ellipsoid_vector_2)) or np.any(np.isnan(ellipsoid_vector_2)):
                print('\n\n ~~~~~ STILL BROKEN ~~~~~~ \n\n')

            # save result
            ellipsoid = ellipsoid_2

        return ellipsoid





if __name__ == '__main__':
    trainval = 'train'
    dataset_path = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/data/{}'.format(trainval)
    protobuf_folder = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/data/{}_protobufs'.format(trainval)
    reader_path = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/scenenet_pb2.py'
    dataset = SceneNetDataset(dataset_path, protobuf_folder, reader_path)


