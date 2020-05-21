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
import importlib
import numpy as np
from PIL import Image

# import gtsam and extension
import gtsam
import quadricslam

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

# import custom python modules
sys.dont_write_bytecode = True
from base.containers import Boxes
from base.containers import Trajectory
from base.containers import Odometry
from base.containers import Quadrics
from visualization.interactive_player import InteractivePlayer


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

        # load protobuf path
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
        valtrain = 'val' if 'val' in sequence_path else 'train'
        protobuf_name = 'scenenet_rgbd_{}_{}.pb'.format(valtrain, render_folder)
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





class cached_property(object):
    """A decorator that converts a function into a lazy property.  The
    function wrapped is called the first time to retrieve the result
    and then that calculated result is used the next time you access
    the value::
        class Foo(object):
            @cached_property
            def foo(self):
                # calculate something important here
                return 42
    The class has to have a `__dict__` in order for this property to
    work.
    """
    def __init__(self, func, name=None, doc=None):
        self.__name__ = name or func.__name__
        self.__module__ = func.__module__
        self.__doc__ = doc or func.__doc__
        self.func = func

    def __get__(self, obj, type=None):
        if obj is None:
            return self
        value = self.func(obj)
        obj.__dict__[self.__name__] = value
        return value





import re
def natural_sort(list_data):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(list_data, key=alphanum_key)



WNID_TO_NYU = {
	'04593077':4, '03262932':4, '02933112':6, '03207941':7, '03063968':10, '04398044':7, '04515003':7,
	'00017222':7, '02964075':10, '03246933':10, '03904060':10, '03018349':6, '03786621':4, '04225987':7,
	'04284002':7, '03211117':11, '02920259':1, '03782190':11, '03761084':7, '03710193':7, '03367059':7,
	'02747177':7, '03063599':7, '04599124':7, '20000036':10, '03085219':7, '04255586':7, '03165096':1,
	'03938244':1, '14845743':7, '03609235':7, '03238586':10, '03797390':7, '04152829':11, '04553920':7,
	'04608329':10, '20000016':4, '02883344':7, '04590933':4, '04466871':7, '03168217':4, '03490884':7,
	'04569063':7, '03071021':7, '03221720':12, '03309808':7, '04380533':7, '02839910':7, '03179701':10,
	'02823510':7, '03376595':4, '03891251':4, '03438257':7, '02686379':7, '03488438':7, '04118021':5,
	'03513137':7, '04315948':7, '03092883':10, '15101854':6, '03982430':10, '02920083':1, '02990373':3,
	'03346455':12, '03452594':7, '03612814':7, '06415419':7, '03025755':7, '02777927':12, '04546855':12,
	'20000040':10, '20000041':10, '04533802':7, '04459362':7, '04177755':9, '03206908':7, '20000021':4,
	'03624134':7, '04186051':7, '04152593':11, '03643737':7, '02676566':7, '02789487':6, '03237340':6,
	'04502670':7, '04208936':7, '20000024':4, '04401088':7, '04372370':12, '20000025':4, '03956922':7,
	'04379243':10, '04447028':7, '03147509':7, '03640988':7, '03916031':7, '03906997':7, '04190052':6,
	'02828884':4, '03962852':1, '03665366':7, '02881193':7, '03920867':4, '03773035':12, '03046257':12,
	'04516116':7, '00266645':7, '03665924':7, '03261776':7, '03991062':7, '03908831':7, '03759954':7,
	'04164868':7, '04004475':7, '03642806':7, '04589593':13, '04522168':7, '04446276':7, '08647616':4,
	'02808440':7, '08266235':10, '03467517':7, '04256520':9, '04337974':7, '03990474':7, '03116530':6,
	'03649674':4, '04349401':7, '01091234':7, '15075141':7, '20000028':9, '02960903':7, '04254009':7,
	'20000018':4, '20000020':4, '03676759':11, '20000022':4, '20000023':4, '02946921':7, '03957315':7,
	'20000026':4, '20000027':4, '04381587':10, '04101232':7, '03691459':7, '03273913':7, '02843684':7,
	'04183516':7, '04587648':13, '02815950':3, '03653583':6, '03525454':7, '03405725':6, '03636248':7,
	'03211616':11, '04177820':4, '04099969':4, '03928116':7, '04586225':7, '02738535':4, '20000039':10,
	'20000038':10, '04476259':7, '04009801':11, '03909406':12, '03002711':7, '03085602':11, '03233905':6,
	'20000037':10, '02801938':7, '03899768':7, '04343346':7, '03603722':7, '03593526':7, '02954340':7,
	'02694662':7, '04209613':7, '02951358':7, '03115762':9, '04038727':6, '03005285':7, '04559451':7,
	'03775636':7, '03620967':10, '02773838':7, '20000008':6, '04526964':7, '06508816':7, '20000009':6,
	'03379051':7, '04062428':7, '04074963':7, '04047401':7, '03881893':13, '03959485':7, '03391301':7,
	'03151077':12, '04590263':13, '20000006':1, '03148324':6, '20000004':1, '04453156':7, '02840245':2,
	'04591713':7, '03050864':7, '03727837':5, '06277280':11, '03365592':5, '03876519':8, '03179910':7,
	'06709442':7, '03482252':7, '04223580':7, '02880940':7, '04554684':7, '20000030':9, '03085013':7,
	'03169390':7, '04192858':7, '20000029':9, '04331277':4, '03452741':7, '03485997':7, '20000007':1,
	'02942699':7, '03231368':10, '03337140':7, '03001627':4, '20000011':6, '20000010':6, '20000013':6,
	'04603729':10, '20000015':4, '04548280':12, '06410904':2, '04398951':10, '03693474':9, '04330267':7,
	'03015149':9, '04460038':7, '03128519':7, '04306847':7, '03677231':7, '02871439':6, '04550184':6,
	'14974264':7, '04344873':9, '03636649':7, '20000012':6, '02876657':7, '03325088':7, '04253437':7,
	'02992529':7, '03222722':12, '04373704':4, '02851099':13, '04061681':10, '04529681':7,}








class SceneNetSequence(object):
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


    ###############################################################
    # Loading Images
    ###############################################################
    @cached_property
    def image_paths(self):
        image_paths = [os.path.join(self.sequence_path, 'photo', fn)
                            for fn in os.listdir(os.path.join(self.sequence_path, 'photo'))
                                if fn.endswith('.jpg')]
        image_paths = natural_sort(image_paths)
        return image_paths

    @cached_property
    def instance_paths(self) -> list:
        instance_paths = [image_path.replace('photo','instance').replace('.jpg','.png') for image_path in self.image_paths]
        return instance_paths

    @cached_property
    def depth_paths(self) -> list:
        depth_paths = [image_path.replace('photo','depth').replace('.jpg','.png') for image_path in self.image_paths]
        return depth_paths
        
    @cached_property
    def images(self):
        return SmartImages(self.image_paths)

    ###############################################################
    # Loading Detections
    ###############################################################
    @cached_property
    def true_boxes(self):
        boxes = Boxes()
        for index, instance_path in enumerate(self.instance_paths):

            # key must align with trajectory keys
            pose_key = index

            # load instance image
            instance_image = np.array(Image.open(instance_path))

            # calculate instance boxes
            image_boxes = self.boxes_from_instance(instance_image, pose_key)

            # add to video_boxes
            boxes.add_boxes(image_boxes)
        return boxes

    def boxes_from_instance(self, instance_image, pose_key):
        """
        Calculates discrete bounding box from pixels:
        [0, 0, 0]
        [0, 1, 0] would effectively be (1,1,2,2) 
        [0, 0, 0]
        """
        image_boxes = Boxes()
        for instance_id in np.unique(instance_image):

            # only use box if class is valid
            if (not self.is_valid(instance_id)):
                continue

            # get class distribution
            # nyu13_class = instance_to_nyu[instance_id]
            # distribution = np.eye(13)[nyu13_class]

            # get masks
            instance_mask = (instance_image == instance_id)
            horizontal_mask = np.any(instance_mask, axis=0)
            vertical_mask = np.any(instance_mask, axis=1)

            # calculate discrete bounds
            xmin, xmax = np.where(horizontal_mask)[0][[0,-1]]
            ymin, ymax = np.where(vertical_mask)[0][[0,-1]]

            # convert discrete to continious bounds 
            box = quadricslam.AlignedBox2(xmin,ymin,xmax+1,ymax+1)

            # store box, pose, object keys. 
            image_boxes.add(box, pose_key, int(instance_id))
        return image_boxes

    def is_valid(self, instance_id):
        """ 
        Instances valid if: 
            RANDOM_OBJECTS 
            and WNID has mapping to NYU 
        """
        return instance_id in self.valid_instances

    @cached_property
    def valid_instances(self):
        valid = [i.instance_id for i in self.sequence_data.instances 
                    if i.instance_type == self.sn.Instance.RANDOM_OBJECT 
                                and i.semantic_wordnet_id in WNID_TO_NYU]
        return valid

    ###############################################################
    # Loading Trajectory
    ###############################################################
    @cached_property
    def true_trajectory(self):
        """ posekeys must align with boxes / images """
        poses = []
        for index, view in enumerate(self.sequence_data.views):
            # pose_key = view.frame_num 
            pose_key = index
            pose_matrix = self.calculate_pose_matrix(view)
            pose = gtsam.Pose3(pose_matrix.astype(np.float))
            poses.append(pose)
        return Trajectory(poses)

    def calculate_pose_matrix(self, view):
        true_pose = self.interpolate_poses(view.shutter_open, view.shutter_close, 0.5)
        world_to_camera = self.world_to_camera_with_pose(true_pose)
        camera_to_world = np.linalg.inv(world_to_camera)
        return camera_to_world

    def interpolate_poses(self, start_pose, end_pose, alpha):
        assert alpha >= 0.0
        assert alpha <= 1.0
        camera_pose = alpha * np.array([end_pose.camera.x, end_pose.camera.y, end_pose.camera.z])
        camera_pose += (1.0 - alpha) * np.array([start_pose.camera.x, start_pose.camera.y, start_pose.camera.z])
        lookat_pose = alpha * np.array([end_pose.lookat.x, end_pose.lookat.y, end_pose.lookat.z])
        lookat_pose += (1.0 - alpha) * np.array([start_pose.lookat.x, start_pose.lookat.y, start_pose.lookat.z])
        timestamp = alpha * end_pose.timestamp + (1.0 - alpha) * start_pose.timestamp
        pose = self.sn.Pose()
        pose.camera.x = camera_pose[0]
        pose.camera.y = camera_pose[1]
        pose.camera.z = camera_pose[2]
        pose.lookat.x = lookat_pose[0]
        pose.lookat.y = lookat_pose[1]
        pose.lookat.z = lookat_pose[2]
        pose.timestamp = timestamp
        return pose

    def world_to_camera_with_pose(self, view_pose):
        lookat_pose = np.array([view_pose.lookat.x, view_pose.lookat.y, view_pose.lookat.z])
        camera_pose = np.array([view_pose.camera.x, view_pose.camera.y, view_pose.camera.z])
        up = np.array([0,1,0])
        R = np.diag(np.ones(4))
        R[2,:3] = self.normalize(lookat_pose - camera_pose)
        R[0,:3] = self.normalize(np.cross(R[2,:3],up))
        R[1,:3] = -self.normalize(np.cross(R[0,:3],R[2,:3]))
        T = np.diag(np.ones(4))
        T[:3,3] = -camera_pose
        return R.dot(T)

    def normalize(self, vector):
        return vector / np.linalg.norm(vector)

    ###############################################################
    # Loading Objects
    ###############################################################
    @cached_property
    def true_quadrics(self):
        """ 
        Can estimate each objects quadric,
        either using the vertices bounds and inscribing a quadric,
        or by calculating the quadric of best fit for each vertices set.
        The problem with the latter is that vertices are not distributed evenly.
        """
        quadrics = Quadrics()
        for instance in self.sequence_data.instances:

            # only calculate for valid instances
            if instance.instance_id not in self.valid_instances:
                continue

            # load local vertices and map to global
            vertices = self.load_vertices(instance)

            # calculate 3D bounds of vertices
            bounds = self.vertices_bounds(vertices)

            # convert bounds to quadric
            quadric = self.bounds_to_quadric(bounds)

            # add to quadrics
            quadrics.add(quadric, int(instance.instance_id))
        return quadrics

    def bounds_to_quadric(self, bounds):
        radii = bounds.dimensions() / 2.0
        pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(bounds.centroid()))
        return quadricslam.ConstrainedDualQuadric(pose, radii)

    def vertices_bounds(self, vertices):
        min_x = min([v[0] for v in vertices])
        max_x = max([v[0] for v in vertices])
        min_y = min([v[1] for v in vertices])
        max_y = max([v[1] for v in vertices])
        min_z = min([v[2] for v in vertices])
        max_z = max([v[2] for v in vertices])
        xxyyzz = np.array([min_x, max_x, min_y, max_y, min_z, max_z])
        box = quadricslam.AlignedBox3(xxyyzz)
        return box

    def load_vertices(self, instance):
        # get object pose
        op = instance.object_info.object_pose
        object_pose = np.array([[op.rotation_mat11, op.rotation_mat12, op.rotation_mat13, op.translation_x],
                                [op.rotation_mat21, op.rotation_mat22, op.rotation_mat23, op.translation_y],
                                [op.rotation_mat31, op.rotation_mat32, op.rotation_mat33, op.translation_z],
                                [0, 0, 0, 1]])
        object_height = instance.object_info.height_meters
        
        # get local verticies from shapenet
        object_path = os.path.join(self.shapenet_path, instance.object_info.shapenet_hash, 'models', 'model_normalized.obj')
        local_vertices = self.load_local_vertices(object_path)

        # convert to global vertices
        global_vertices = self.local_to_global_vertices(local_vertices, object_pose[0:3,0:3], object_pose[0:3,3], object_height)
        return global_vertices
            
    def load_local_vertices(self, object_path):
        shapenet_obj_file = open(object_path, "r")
        vertices = []
        for l in shapenet_obj_file:
            if l.startswith('v '):
                s = l[2:].split()
                x = float(s[0])
                y = float(s[1])
                z = float(s[2])
                vertices.append([x, y, z])
        return np.array(vertices, dtype=np.float)

    def local_to_global_vertices(self, local_vertices, object_rotation, object_translation, object_height):
        """As defined in pyscenenetrgbd/generate_scene_obj.py merge_scene_obj()"""
        bb = self.vertices_bounds(local_vertices).vector()
        centroid = np.array([bb[0] + ((bb[1] - bb[0]) / 2.0), bb[2] + ((bb[3] - bb[2]) / 2.0), bb[4] + ((bb[5] - bb[4]) / 2.0)])
        # print('cent', centroid)
        centroid[1] -= 0.6 * (bb[3] - bb[2])
        # print('cent2', centroid)
        global_vertices = []
        for lv in local_vertices:
            # print('lv', lv)
            gv = (lv - centroid) * (object_height / (bb[3] - bb[2]))
            # print('gv', gv)
            gv = object_rotation.dot(gv) + object_translation
            # print('gv2', gv)
            global_vertices.append(gv)
        return np.array(global_vertices)





class SmartImages(object):
    def __init__(self, image_paths):
        self.image_paths = image_paths
        
    def __getitem__(self, index):
        image_path = self.image_paths[index]
        image = cv2.imread(image_path)
        return image

    def __len__(self):
        return len(self.image_paths)



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
        image = self.sequence.images[self.image_index]
        current_pose = self.sequence.true_trajectory[self.image_index]
        image_boxes = self.sequence.true_boxes.at_pose(self.image_index)

        for (pose_key, object_key), box in image_boxes.items():
            self.draw_box(image, box, text='{}'.format(object_key))
            # drawing.cv2_draw_box_and_text(image, det.box.vector, box_color=(0,255,0), text='{}'.format(det.object_key), text_color=(0,0,0))

        # show image
        cv2.imshow(self.player_name, image)

    def draw_box(self, image, box, text):
        box = box.vector()
        color = (255,0,255)
        thickness = 2
        cv2.rectangle(image, (int(box[0]),int(box[1])), (int(box[2]),int(box[3])), color, thickness)
        self.cv2_draw_text(image, text, (box[0]-thickness,box[1]-thickness), (255,255,255), True, (0,0,255), 3)

    def cv2_draw_text(self, image, text, lower_left, color=(255,255,255), thickness=1, background=False, background_color=(0,0,255), background_margin=3):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5

        lower_left = [lower_left[0]+background_margin, lower_left[1]-background_margin]
        text_size = cv2.getTextSize(text, font, font_scale, thickness)
        text_width = text_size[0][0] + background_margin * 2
        text_height = text_size[0][1] + background_margin * 2
        image_width = image.shape[1]
        image_height = image.shape[0]

        # lower_left = [upper_left[0], upper_left[1]+text_width]
        final_position = list(lower_left)
        final_position[1] = int(np.clip(lower_left[1], text_height, image_height))
        final_position[0] = int(np.clip(lower_left[0], 0, image_width-text_width))
        final_position = tuple(final_position)

        if (background):
            upper_left = [final_position[0], final_position[1]-text_height]
            xmin = upper_left[0]-background_margin+1
            ymin = upper_left[1]-background_margin+4
            xmax = upper_left[0]+text_width+background_margin
            ymax = upper_left[1]+text_height+background_margin+1
            cv2.rectangle(image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), background_color, cv2.FILLED)
        cv2.putText(image, text, final_position, font, font_scale, color, thickness, cv2.LINE_AA)













if __name__ == '__main__':
    trainval = 'train'
    dataset_path = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/data/{}'.format(trainval)
    protobuf_folder = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/data/{}_protobufs'.format(trainval)
    reader_path = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/scenenet_pb2.py'
    dataset = SceneNetDataset(dataset_path, protobuf_folder, reader_path)

    player = SceneNetPlayer(dataset)
    player.start()




