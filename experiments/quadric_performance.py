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
import atexit
import yaml
import argparse
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../quadricslam'))

# import custom python modules
sys.dont_write_bytecode = True
from quadricslam_online import QuadricSLAM_Online
from quadricslam_offline import QuadricSLAM_Offline
from base.data_association import DataAssociation
from base.data_association import KeepAssociator
from base.data_association import MergeAssociator
from visualization.drawing import CV2Drawing
from base.containers import Trajectory, Quadrics, Detections, ObjectDetection
from base.initialisation import Initialize

from dataset_interfaces.scenenet_dataset import SceneNetDataset
from dataset_interfaces.tumrgbd_dataset import TUMDataset
from dataset_interfaces.simulated_dataset import SimulatedSequence
from detectors.faster_rcnn import FasterRCNN
from base.evaluation import Evaluation

# import gtsam and extension
import gtsam
import gtsam_quadrics



def test_constrainment():
    config = yaml.safe_load(open('/home/lachness/git_ws/quadricslam/quadricslam/config/online.yaml', 'r'))

    points = []
    points.append(gtsam.Point3(10,0,0))
    points.append(gtsam.Point3(0,-10,0))
    points.append(gtsam.Point3(-10,0,0))
    points.append(gtsam.Point3(0,10,0))
    points.append(gtsam.Point3(10,0,0))

    
    seq3ax = SimulatedSequence(points, [gtsam_quadrics.ConstrainedDualQuadric(gtsam.Pose3(), np.array([0.3,0.6,0.9]))], 5)
    seq2ax = SimulatedSequence(points, [gtsam_quadrics.ConstrainedDualQuadric(gtsam.Pose3(), np.array([0.3,0.6,0.3]))], 5)
    seq1ax = SimulatedSequence(points, [gtsam_quadrics.ConstrainedDualQuadric(gtsam.Pose3(), np.array([0.3,0.3,0.3]))], 5)
    scenes = {
        'unique': seq3ax,
        'cylinder': seq2ax,
        'sphere': seq1ax,
    }

    optimizers = ["ISAM", "ISAM-D", "LVM", "GN"]
    priors = ['None', 'Ri'] #['None', 'Q0', 'R0', 'Qi', 'Ri', 'A']
    seeds = np.arange(10)

    frames = []

    for quadric_name, scene in scenes.items():
        print('~~~~~~~~~~~~~~Scene {}~~~~~~~~~~~~~~~~'.format(quadric_name))
        true_bounds = {k:v.bounds() for k,v in scene.true_quadrics.items()}

        for opt in optimizers:
            print('~~~~~~ OPT {} ~~~~~~~~'.format(opt))
            config['Optimizer'] = opt

            for prior in priors:
                if prior == 'None':
                    config['QuadricSLAM.quad_priors'] = False
                    config['QuadricSLAM.prior_rots'] = False
                    config['QuadricSLAM.angle_factors'] = False
                if prior == 'Q0': 
                    config['QuadricSLAM.zero_prior'] = True
                    config['QuadricSLAM.quad_priors'] = True
                    config['QuadricSLAM.prior_rots'] = False
                    config['QuadricSLAM.angle_factors'] = False
                if prior == 'R0': 
                    config['QuadricSLAM.zero_prior'] = True
                    config['QuadricSLAM.quad_priors'] = False
                    config['QuadricSLAM.prior_rots'] = True
                    config['QuadricSLAM.angle_factors'] = False
                if prior == 'Qi': 
                    config['QuadricSLAM.zero_prior'] = False
                    config['QuadricSLAM.quad_priors'] = True
                    config['QuadricSLAM.prior_rots'] = False
                    config['QuadricSLAM.angle_factors'] = False
                if prior == 'Ri': 
                    config['QuadricSLAM.zero_prior'] = False
                    config['QuadricSLAM.quad_priors'] = False
                    config['QuadricSLAM.prior_rots'] = True
                    config['QuadricSLAM.angle_factors'] = False
                if prior == 'A': 
                    config['QuadricSLAM.zero_prior'] = False
                    config['QuadricSLAM.quad_priors'] = False
                    config['QuadricSLAM.prior_rots'] = False
                    config['QuadricSLAM.angle_factors'] = True

                for seed in seeds:
                    np.random.seed(seed)

                    # make SLAM system
                    SLAM = QuadricSLAM_Online(scene.calibration, config)

                    # record GT partial trajectory
                    partial_true = Trajectory()

                    # inject noise into trajectory
                    true_trajectory = scene.true_trajectory
                    true_odometry = true_trajectory.as_odometry()
                    noisy_odometry = true_odometry.add_noise(mu=0.0, sd=0.01)
                    noisy_trajectory = noisy_odometry.as_trajectory(true_trajectory.values()[0])
                    # noisy_trajectory = scene.true_trajectory

                    # inject noise into measurements
                    true_detections = scene.true_detections
                    noisy_detections = true_detections.add_noise_strict(mu=0.0, sd=2, image_dimensions=(320,240))
                    # noisy_detections = scene.true_detections


                    # iteratively pass new info
                    for pose_key, pose in noisy_trajectory.items():
                        partial_true.add(true_trajectory.at(pose_key), pose_key)
                        image = np.zeros((240,320,3))

                        # get image detections
                        image_detections = Detections()
                        for object_key, detection in noisy_detections.at_pose(pose_key).items():
                            image_detections.add(detection, pose_key, object_key)

                        # SLAM update
                        success = SLAM.update(image, image_detections, pose, scene.true_quadrics)


                        # get current graph / values
                        graph = SLAM.global_graph
                        initial_values = SLAM.global_values
                        current_values = SLAM.current_estimate

                        # check validity
                        is_valid, cond = check_quadric_constrained(graph, current_values)

                        # get estimates
                        estimated_quadrics = SLAM.current_quadrics
                        estimated_trajectory = SLAM.current_trajectory

                        init_iou = Evaluation.evaluate_map(scene.true_quadrics, true_bounds, scene.true_trajectory, scene.true_trajectory, type='weak')[1]
                        iou = Evaluation.evaluate_map(estimated_quadrics, true_bounds, estimated_trajectory, scene.true_trajectory, type='weak')[1]
                        ate = Evaluation.evaluate_trajectory(estimated_trajectory, partial_true, 'weak')[0]

                        if is_valid and success:
                            valid = 'V'
                        if is_valid and not success:
                            valid = 'V-ILS'
                        if not is_valid and success:
                            valid = 'NV'
                        if not is_valid and not success:
                            valid = 'NV-ILS'

                        
                        name = '{}-{}'.format(quadric_name, prior)
                        if prior == 'None':
                            name = quadric_name
                        # if prior == 'Ri':
                        #     name = quadric_name+'-'+'prior'
                        
                        frames.append({
                            'name': name,
                            'seed': seed,
                            'quadric_name': quadric_name,
                            'step': pose_key,
                            'valid': valid,
                            'cond': cond,
                            'aiou': iou,
                            'aiou_improve': (iou-init_iou)/init_iou,
                            'ate': ate,
                            'opt': opt,
                            'prior': prior,
                        })

                        if not success:
                            break

    df = pd.DataFrame(frames)

    # plot lineplots
    # figure for each optimizer
    # subplot for each metric
    # plot lines for opt-fix-sigma
    for opt in optimizers:
        fig, axs = plt.subplots(2,2)
        fig.suptitle('Optimizer {}'.format(opt))
        for i, metric in enumerate(['ate','aiou','cond', 'valid']):
            ax = axs[np.unravel_index(i, (2,2))]

            subdata = df[df.opt==opt]
            if metric != 'valid':
                sns.lineplot(ax=ax, data=subdata, x='step', y=metric, hue='name', marker='o')
            if metric == 'valid':
                sns.swarmplot(ax=ax, data=subdata, x='step', y=metric, hue='name')
            # sns.scatterplot(ax=ax, data=subdata, x='step', y=metric, hue='name', style='valid')

            if metric == 'cond':
                ax.set_yscale('log')
    plt.show()


    # # plot lineplots
    # for scene_n in range(3):
    #     fig, axs = plt.subplots(2,2)
    #     for opt_n, opt in enumerate(["ISAM", "ISAM-D", "LVM", "GN"]):
    #         ax = axs[np.unravel_index(opt_n, (2,2))]
    #         ax.set_title('Scene {} | Opt {}'.format(scene_n,opt))
    #         sns.lineplot(ax=ax, data=df[df.scene_n==scene_n][df.opt==opt], x='step', y='cond', hue='prior')
    #         sns.scatterplot(ax=ax, data=df[df.scene_n==scene_n][df.opt==opt], x='step', y='cond', hue='prior', style='valid')
    #         # ax.set_yscale('log')
    #         # for prior in ['None', 'Q0', 'R0', 'Qi', 'Ri', 'A']:
    plt.show()
            
    
    

            
def check_quadric_constrained(graph, values):
    # # collect nonlinear factors from graph
    # box_factors = []
    # for i in range(graph.size()):
    #     factor = graph.at(i)
    #     if factor.keys().size() == 2 and chr(gtsam.symbolChr(factor.keys().at(1))) == 'q':
    #         box_factors.append(gtsam_quadrics.dynamic_cast_BoundingBoxFactor_NonlinearFactor(factor))

    # quad_keys = [f.keys().at(1) for f in box_factors]
    # for object_key in np.unique(quad_keys):
    #     key = gtsam.symbolIndex(object_key)

    #     quad_factors = [f for f in box_factors if f.keys().at(1) == object_key]
    #     dquad = [f.evaluateH2(values) for f in quad_factors]
    #     dpose = [f.evaluateH1(values) for f in quad_factors]

    #     n_factors = len(quad_factors)
    #     nzero_cols = [np.any(np.abs(j)>1e-10, axis=0) for j in dquad]
    #     nzero_rows = [np.any(np.abs(j)>1e-10, axis=1) for j in dquad]
    #     box_dofs_per_dim = np.array(nzero_rows).sum(0)
    #     quad_dofs_per_dim = np.array(nzero_cols).sum(0)

    #     conds = [np.linalg.cond(j) for j in dquad]
    #     print('Quadric: {} | factors {}'.format(key, n_factors))
    #     # print('   quaddofs', quad_dofs_per_dim)
    #     print(quad_dofs_per_dim)
    #     print('max J cond', np.max(conds))


        # if not np.all(np.logical_and.reduce(nzero_cols)):
        #     print('Some quadric dimensions uncontrained')
        #     print(np.logical_and.reduce(nzero_cols))
        #     print(quad_dofs_per_dim)

        #     import code
        #     code.interact(local=dict(globals(),**locals()))

    # ensure each quadric has at least 9dof from boxes
    # ensure each quadric parameter affects the box

    gfg = graph.linearize(values) 
    jacobian = gfg.jacobian()[0]
    hessian = gfg.hessian()[0]

    is_valid = True
    # check if underdetermined
    if np.linalg.matrix_rank(jacobian) < values.dim() \
        or np.linalg.matrix_rank(hessian) < values.dim():
        # print('  NOT VALID: underdetermined')
        is_valid = False

    # check if indefinite, i.e not positive semidefinite or negative semidefinite
    eigv = np.linalg.eigh(hessian)[0]
    if np.any(eigv<0) and np.any(eigv>0):
        # print('  NOT VALID: indefinite hessian')
        is_valid = False

    if not np.all(eigv>0):
        # print('  NOT VALID: not postive definite')
        is_valid = False

    # check conditioning 
    cond = np.linalg.cond(jacobian)
    return is_valid, cond



def run_tum_offline():
    # load config
    config = yaml.safe_load(open('/home/lachness/git_ws/quadricslam/quadricslam/config/scenenet.yaml', 'r'))

    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM(config['Optimizer.verbosity'])    
    params.setMaxIterations(config['Optimizer.max_iterations'])
    params.setlambdaInitial(config['Optimizer.lambda_initial'])
    params.setlambdaUpperBound(config['Optimizer.lambda_upper_bound'])
    params.setlambdaLowerBound(config['Optimizer.lambda_lower_bound'])
    params.setRelativeErrorTol(config['Optimizer.relative_error_tol'])
    params.setAbsoluteErrorTol(config['Optimizer.absolute_error_tol'])
    

    # load dataset
    dataset = TUMDataset('/media/lachness/DATA/Datasets/TUM/') 

    for scene in dataset:

        SLAM = QuadricSLAM_Offline(scene.calibration, params, config)


        # inject noise into trajectory
        true_trajectory = scene.aligned_trajectory
        true_odometry = true_trajectory.as_odometry()
        noisy_odometry = true_odometry.add_noise(mu=0.0, sd=config['Noise.odom_sd'])
        noisy_trajectory = noisy_odometry.as_trajectory(true_trajectory.values()[0])
        

        SLAM.run(noisy_trajectory, scene.associated_detections, true_trajectory=true_trajectory, visualize=True)







def run_tum():
    # load config
    config = yaml.safe_load(open('/home/lachness/git_ws/quadricslam/quadricslam/config/online.yaml', 'r'))

    # load dataset
    dataset = TUMDataset('/media/lachness/DATA/Datasets/TUM/')

    # load detector
    # predictor = FasterRCNN('COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml', batch_size=5)

    print('starting')

    for scene_n, scene in enumerate(dataset):
        if scene_n < 2:
            continue

        # start SLAM
        SLAM = QuadricSLAM_Online(scene.calibration, config)


        print('testing scene {}'.format(scene.name))
        # print('calibration:')
        # print(scene.calibration)
        
        # iterate through timesteps and send to SLAM
        for pose_key, rgb_path in scene.rgb_images.items():

            # load image
            rgb_image = cv2.imread(rgb_path)

            # load odometry
            camera_pose = scene.trajectory[pose_key]

            # calculate detections
            # detections = predictor([rgb_image])[0]

            # convert image_detections to Detections
            detections = Detections()
            for object_key, d in scene.detections.at_pose(pose_key).items():
                detections.add(d, pose_key, object_key)
            
            # filter partials
            detections = filter_partials(detections, 640., 480., 10)
            

            success = SLAM.update(rgb_image, detections, camera_pose, visualize=True, pose_key=pose_key)

            # print('')
            # cv2.waitKey(0)

            
            if not success:
                print('UPDATE FAILED!')
                break


def filter_partials(detections, width, height, pixels):
    # filter partials
    image_bounds = gtsam_quadrics.AlignedBox2(0,0,width,height)
    filter_pixels = pixels
    filter_bounds = image_bounds.vector() + np.array([1,1,-1,-1])*filter_pixels
    filter_bounds = gtsam_quadrics.AlignedBox2(filter_bounds)
    filtered_detections = Detections()
    for (pose_key, object_key), detection in detections.items():
        if filter_bounds.contains(detection.box):
            filtered_detections.add(detection, pose_key, object_key)
    return filtered_detections




def run_scenenet():
    config = yaml.safe_load(open('/home/lachness/git_ws/quadricslam/quadricslam/config/online.yaml', 'r'))
    dataset = SceneNetDataset(
        dataset_path = "/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/data/train",
        protobuf_folder = "/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/data/train_protobufs",
        reader_path = "/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/scenenet_pb2.py",
        shapenet_path = "/media/lachness/DATA/Datasets/ShapeNet/ShapeNetCore.v2"
    )
    rendered = load_rendered_detections("/home/lachness/backups/ICRA_2021/experiments/data/train_100_occfree.npy")

    data = []

    # dataset pose keys need to match frame_n
    for scene_n, scene in enumerate(dataset):
        # if scene_n < 2:
        #     continue

        images = scene.images
        detections = scene.true_detections
        rendered_detections = rendered[scene_n]
        trajectory = scene.true_trajectory

        # true bounds for eval
        true_bounds = {k:v.bounds() for k,v in scene.true_quadrics.items()}

        # # filter partials
        # image_bounds = gtsam_quadrics.AlignedBox2(0,0,320.,240.)
        # filter_pixels = 10
        # filter_bounds = image_bounds.vector() + np.array([1,1,-1,-1])*filter_pixels
        # filter_bounds = gtsam_quadrics.AlignedBox2(filter_bounds)
        # filtered_detections = Detections()
        # for (pose_key, object_key), detection in detections.items():
        #     if filter_bounds.contains(detection.box):
        #         filtered_detections.add(detection, pose_key, object_key)
        # detections = filtered_detections

        # eliminate measurements that are not common between datasets
        rendered_detections, detections = align_measurements(rendered_detections, detections)

        # flag partials
        detections = remove_partials(rendered_detections, detections)

        # remove small measurements
        detections = remove_small(detections, dim_limit=10)

        optimizers = ['ISAM-D']#, 'LVM', 'GN']
        fixes = ['none', 'prior']
        sigmas = [100, 1000]

        for opt in optimizers:
            for fix in fixes:
                for sigma in sigmas:
                    name = opt+'_'+fix+'_'+str(sigma)
                    print('TESTING: {}'.format(name))
        

                    config['Optimizer'] = opt

                    if fix == 'none':
                        config['QuadricSLAM.prior_rots'] = False
                    if fix == 'prior':
                        config['QuadricSLAM.prior_rots'] = True
                        config['QuadricSLAM.prior_rots_sd'] = sigma
                        config['QuadricSLAM.inf_sd'] = 1e18
                        
                    
                    SLAM = QuadricSLAM_Online(scene.calibration, config)

                    

                    
        



                    # loop over image,pose,detections
                    success = True
                    for pose_key, pose in trajectory.items():


                        
                        
                        image = images[pose_key]

                        # set detections to have the same posekeys
                        image_detections = Detections()
                        for object_key, detection in detections.at_pose(pose_key).items():
                            image_detections.add(detection, pose_key, object_key)

                        # only optimize full-batch if GN/LVM 
                        dont_optimize = False
                        if opt in ['GN', 'LVM'] and pose_key != trajectory.keys()[-1]:
                            dont_optimize = True

                        success = success and SLAM.update(image, image_detections, pose, scene.true_quadrics, dont_optimize=dont_optimize, visualize=True)


                        # giveup once we receive ILS
                        if not success:
                            break

                        # ensure quadrics improved from SVD | only makes sense if not ILS
                        # init_aiou = Evaluation.evaluate_map(Quadrics.from_values(SLAM.global_values), true_bounds, SLAM.current_trajectory, scene.true_trajectory, type='weak')[1]
                        aiou = Evaluation.evaluate_map(SLAM.current_quadrics, true_bounds, SLAM.current_trajectory, scene.true_trajectory, type='weak')[1]

                        # init_error = SLAM.global_graph.error(SLAM.global_values)
                        error = SLAM.global_graph.error(SLAM.current_estimate)

                        # record final data 
                        data.append({
                            'step': pose_key,
                            'scene_n': scene_n,
                            'success': int(success),
                            'name': name,
                            'aiou': aiou,
                            'error': error,
                        })

                    # only run normal for one sigma
                    if fix == 'none':
                        break



                    # magically check errors
                    # errors = [self.global_graph.at(i).error(self.global_values) for i in range(self.global_graph.size())]
                    # indices = np.where(np.isnan(errors))[0]
                    # bbfs = [gtsam_quadrics.dynamic_cast_BoundingBoxFactor_NonlinearFactor(self.global_graph.at(i)) for i in indices]
                    # object_keys = [bbf.objectKey() for bbf in bbfs]
                    # pose_keys = [bbf.poseKey() for bbf in bbfs]
                    # poses = [self.global_values.atPose3(key) for key in pose_keys]
                    # quadrics = [gtsam_quadrics.ConstrainedDualQuadric.getFromValues(self.global_values, key) for key in object_keys]



                    # # manually check errors
                    # manual_errors = []
                    # for i in range(self.global_graph.size()):
                    #     factor = self.global_graph.at(i)
                    #     if factor.keys().size() == 2 and chr(gtsam.symbolChr(factor.keys().at(1))) == 'q':
                    #         bbf = gtsam_quadrics.dynamic_cast_BoundingBoxFactor_NonlinearFactor(factor)
                    #         quadric = gtsam_quadrics.ConstrainedDualQuadric.getFromValues(self.global_values, bbf.objectKey())
                    #         pose = self.global_values.atPose3(bbf.poseKey())
                    #         error = bbf.evaluateError(pose, quadric)
                    #         for e in error:
                    #             manual_errors.append(e)


        df = pd.DataFrame(data)

        # # make table methods vs ILS count 
        # print('Tested {} Scenes'.format(scene_n+1))
        # table = df.pivot_table(index='name', values=['success','improve'], aggfunc=np.sum)
        # print(table)
        # # import code
        # # code.interact(local=dict(globals(),**locals()))


        fig, axs = plt.subplots(2,1)
        sns.lineplot(ax=axs[0], data=df[df.scene_n==scene_n], x='step', y='aiou', hue='name')
        plt.grid(True)
        sns.lineplot(ax=axs[1], data=df[df.scene_n==scene_n], x='step', y='error', hue='name')
        plt.grid(True)
        axs[1].set_yscale('log')
        plt.show()

    
        # # plot lineplots
        # # figure for each optimizer
        # # subplot for each metric
        # # plot lines for opt-fix-sigma
        # for opt in optimizers:
        #     fig, axs = plt.subplots(2,2)
        #     plt.title('Optimizer {}'.format(opt))
        #     for i, metric in enumerate(['ate','aiou','cond']):
        #         ax = axs[np.unravel_index(i, (2,2))]

        #         subdata = df[df.opt==opt]
        #         sns.lineplot(ax=ax, data=subdata, x='step', y=metric, hue='name')
        #         sns.scatterplot(ax=ax, data=subdata, x='step', y=metric, hue='name', style='valid')

        #         if metric == 'cond':
        #             ax.set_yscale('log')
        # plt.show()





def load_rendered_detections(path):
    """
    Converts Nx8 structure into dict[sequence_n] = Detections
    """
    data = np.load(open(path, 'rb'))
    occfree_detections = {}
    for sequence_n in np.unique(data[:,0]):
        occfree_detections[sequence_n] = Detections()
        sequence_data = data[data[:,0]==sequence_n]
        for row in sequence_data:
            pose_key = row[1]
            object_key = row[2]
            box = gtsam_quadrics.AlignedBox2(*row[-4:])
            detection = ObjectDetection(box, 1.0, 1.0)
            detection.truncated = bool(row[3])
            occfree_detections[sequence_n].add(detection, pose_key, object_key)
    return occfree_detections

def align_measurements(a, b):
    """ only take common measurements | Rendered | Noisy """
    a2 = Detections()
    b2 = Detections()
    for (pose_key, object_key), detection in b.items():
        try:
            a2.add(a.at(pose_key, object_key), pose_key, object_key)
            b2.add(detection, pose_key, object_key)
        except:
            pass
    return a2, b2

def remove_partials(rendered_detections, true_detections):
    d = Detections()
    for (pose_key, object_key), true_detection in true_detections.items():
        rendered_detection = rendered_detections.at(pose_key, object_key)
        
        # check if occluded
        is_occluded = np.any(np.abs(true_detection.box.vector() - rendered_detection.box.vector()) > 1)
        
        true_detection.occluded = is_occluded
        true_detection.truncated = rendered_detection.truncated

        if not (true_detection.occluded or true_detection.truncated):
            d.add(true_detection, pose_key, object_key)
    return d

def remove_small(detections, dim_limit=5.0):
    d = Detections()
    for (pose_key, object_key), measurement in detections.items():
        if measurement.box.width() >= dim_limit and measurement.box.height() >= dim_limit:
            d.add(measurement, pose_key, object_key)
    return d



if __name__ == '__main__':
    np.random.seed(121)
    test_constrainment()