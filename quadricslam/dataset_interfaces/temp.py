# import standard libraries
import os
import sys
import numpy as np
import argparse
import yaml
import matplotlib.pyplot as plt
import cv2
import pandas as pd
import seaborn as sns
import code
import json
import time

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../quadricslam'))
sys.dont_write_bytecode = True

# import quadricslam modules
from quadricslam_offline import QuadricSLAM_Offline
from dataset_interfaces.simulated_dataset import SimulatedSequence
from dataset_interfaces.scenenet_dataset import SceneNetDataset
from visualization.drawing import MPLDrawing
from visualization.drawing import CV2Drawing
from base.evaluation import Evaluation
from base.containers import *

# import gtsam and extension
import gtsam
import gtsam_quadrics



name_to_estimator = {
    'Cauchy': gtsam.noiseModel_mEstimator_Cauchy,
    'DCS': gtsam.noiseModel_mEstimator_DCS,
    'Fair': gtsam.noiseModel_mEstimator_Fair,
    'GemanMcClure': gtsam.noiseModel_mEstimator_GemanMcClure,
    'Huber': gtsam.noiseModel_mEstimator_Huber,
    'Tukey': gtsam.noiseModel_mEstimator_Tukey,
    'Welsch': gtsam.noiseModel_mEstimator_Welsch,
}

name_to_parameter = {
    'Cauchy': 0.30,
    'DCS': 3.79,
    'Fair': 0.1,
    'GemanMcClure': 2.64,
    'Huber': 0.1,
    'Tukey': 16.24,
    'Welsch': 5.46,
}


scene_names = [
    'fr1_desk',
    'fr1_desk2',
    'fr2_desk',
    'fr3_office'
]

odometry_names = [
    'fovisvo',
    'orb',
    'orbvo' 
]

# https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
scene_name_to_calibration = {
    'fr1_desk2': gtsam.Cal3_S2(517.3, 516.5, 0.0, 318.6, 255.3),
    'fr1_desk': gtsam.Cal3_S2(517.3, 516.5, 0.0, 318.6, 255.3),
    'fr2_desk': gtsam.Cal3_S2(520.9, 521.0, 0.0, 325.1, 249.7),
    'fr3_office': gtsam.Cal3_S2(535.4, 539.2, 0.0, 320.1, 247.6),
}

methods = [
    'Baseline',
    'Complex',
    'HighUncertainty',
    'Filtering',
    'Cauchy',
    'DCS',
    'Fair',
    'GemanMcClure',
    'Huber',
    'Tukey',
    'Welsch',
]
















def load_optimizer_parms(config):
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM(config['Optimizer.verbosity'])    
    params.setMaxIterations(config['Optimizer.max_iterations'])
    params.setlambdaInitial(config['Optimizer.lambda_initial'])
    params.setlambdaUpperBound(config['Optimizer.lambda_upper_bound'])
    params.setlambdaLowerBound(config['Optimizer.lambda_lower_bound'])
    params.setRelativeErrorTol(config['Optimizer.relative_error_tol'])
    params.setAbsoluteErrorTol(config['Optimizer.absolute_error_tol'])
    params.setErrorTol(config['Optimizer.error_tol'])
    return params

def load_tum_trajectory(scene_name, odometry_name, folder):
    # get proper path
    path = get_odometry_path(scene_name, odometry_name, folder)
    
    # load data
    text_file = open(path, 'r')
    text = text_file.read()
    lines = text.split('\n')

    # ignore comment lines
    lines = [line for line in lines if line and line[0] != '#']

    # convert each line to pose, key, time
    time_to_key = {}
    trajectory = Trajectory()
    for line in lines:
        time = line.split(' ')[0]; 
        numbers = line.split(' ')[1:]
        precision = len(time.split('.')[-1])

        if len(numbers) != 7:
            print('warning, invalid line: [', line, ']')
            continue

        # initialize gtsam pose
        numbers = np.array(numbers).astype('float')
        trans = gtsam.Point3(numbers[0], numbers[1], numbers[2])
        rot = gtsam.Rot3.Quaternion(numbers[6], numbers[3], numbers[4], numbers[5]) # Quaternion(double w, double x, double y, double z)
        gtsam_pose = gtsam.Pose3(rot, trans)

        # construct pose
        pose_key = len(trajectory)
        trajectory.add(gtsam_pose, pose_key)
        time_to_key[time] = pose_key

    return trajectory, time_to_key

def load_tum_detections(scene_name, folder, time_to_key):
    # load proper path
    path = get_detections_path(scene_name, folder)
    
    # load data
    data = json.load(open(path))

    # convert each measurement to detection
    str_times = []
    detections = Detections()
    for frame in data:

        # extract frame
        object_key = int(frame['object_key'])
        objectness = float(frame['objectness'])
        box = gtsam_quadrics.AlignedBox2(*frame['box'])
        distribution = np.array(frame['scores'])
        str_time = frame['image_path'].split('/')[-1].replace('.png','')
        # pose_key = int(frame['image_key'])

        # get pose key if odometry available
        if str_time in time_to_key:
            pose_key = time_to_key[str_time]
        else:
            continue

        # create detection
        detection = ObjectDetection(box, 1.0, distribution)
        detections.add(detection, pose_key, object_key)

    return detections

def get_odometry_path(scene_name, odometry_name, folder):
    fn = [f for f in os.listdir(folder) if f.endswith('.txt') and odometry_name in f and scene_name+'_' in f]
    if len(fn) == 0:
        raise Exception('no files match scene_name: {}, odom_name: {}, in folder: {}'.format(scene_name, odometry_name, folder))
    elif len(fn) > 1:
        raise Exception('multiple files match scene_name: {}, odom_name: {}, in folder: {}'.format(scene_name, odometry_name, folder))
    return folder+fn[0]

def get_detections_path(scene_name, folder):
    fn = [f for f in os.listdir(folder) if f.endswith('.json') and scene_name+'_' in f]
    if len(fn) == 0:
        raise Exception('no files match scene_name: {}, in folder: {}'.format(scene_name, folder))
    elif len(fn) > 1:
        raise Exception('multiple files match scene_name: {}, in folder: {}'.format(scene_name, folder))
    return folder+fn[0]



def associate_times(first_list, second_list, offset=0.0, max_difference=0.02):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim
    to find the closest match for every input tuple.

    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
                            actually: (first_stamp1, second_stamp1), (first_stamp2, second_stamp2)

    """
    first_keys = list(first_list.keys())
    second_keys = list(second_list.keys())
    potential_matches = [(abs(a - (b + offset)), a, b)
                            for a in first_keys
                            for b in second_keys
                            if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))

    matches.sort()
    return matches


def time_align_trajectories(estimated_trajectory, true_trajectory, estimated_ttk, true_ttk):
    # convert ttks to floats
    estimated_ttk = {float(k):v for k,v in estimated_ttk.items()}
    true_ttk = {float(k):v for k,v in true_ttk.items()}

    # get pose matches between estimated and true trajectory
    matches = associate_times(estimated_ttk, true_ttk, offset=0.0, max_difference=0.02)

    # extract matching poses
    est_traj = Trajectory()
    true_traj = Trajectory()
    for match_key, match in enumerate(matches):
        est_time = match[0]; true_time = match[1]
        est_key = estimated_ttk[est_time]
        true_key = true_ttk[true_time]

        # make new trajectories with the same keys
        est_traj.add(estimated_trajectory.at(est_key), match_key)
        true_traj.add(true_trajectory.at(true_key), match_key)

    return est_traj, true_traj



def evaluate_tum(estimated_trajectory, true_trajectory, estimated_ttk, true_ttk, align_type):
    # time align trajectories
    estimated_trajectory, true_trajectory = time_align_trajectories(estimated_trajectory, true_trajectory, estimated_ttk, true_ttk)
    
    # align trajectory reference frame
    transform = Evaluation.calculate_transform(estimated_trajectory, true_trajectory, type=align_type)
    aligned_trajectory = estimated_trajectory.applyTransform(transform)

    # evaluate ATE
    ate = Evaluation.ATE(aligned_trajectory.values(), true_trajectory.values())
    return ate





def filter_measurements(detections, bounds):
    """ If measurement extends beyond Bounds, it is removed. """
    d = Detections()
    for (pose_key, object_key), detection in detections.items():
        if bounds.contains(detection.box):
            d.add(detection, pose_key, object_key)
    return d



def save_current_figure(name, xscale=0.6, yscale=0.6):
    if 'vs_conditions_vs_method' in name:
        xscale = 0.6; yscale=0.3; 
    fig = plt.gcf()
    fig.set_size_inches(16*xscale,9*yscale)
    plt.savefig(name, bbox_inches='tight', pad_inches=0, dpi=200)


def prepare_figure(name):
    fig = plt.figure(name)
    ax = fig.gca(); ax.clear()
    # plt.title(name)
    plt.grid(True)
    return ax





def remove_objects(measurements, bad_objects):
    new = Detections()
    for (pose_key, object_key), detection in measurements.items():
        if object_key not in bad_objects:
            new.add(detection, pose_key, object_key)
    return new



def save_data(frames, path):
    try:
        savefile = path
        append = 0
        while os.path.isfile(savefile):
            path, ext = os.path.splitext(path)
            savefile = path + '({})'.format(append) + ext
            append += 1
        json.dump(frames, open(savefile, 'w'))
    except:
        code.interact(local=locals())


def resave_tables():
    data_path = '/home/lachness/git_ws/quadricslam/experiments/data/tum_3sd_CORRECT_FINAL.json'

    figure_path = data_path.replace('.json', '/').replace('/data/', '/figures/')
    os.makedirs(figure_path, exist_ok=True)

    # load data 
    with open(data_path, 'r') as f:
        data = json.load(f)

    # create dataframe
    df = pd.DataFrame(data)

    # calculate ate_decrease
    df['ate_decrease'] = (df.initial_ate_weak - df.final_ate_weak)

    # def bold_min(s):
    #     is_min = s==s.min()
    #     return ['font-weight: bold' if v else '' for v in is_min]

    # present initial ate as a method
    table = df[df.method_name=='Baseline'].pivot_table(index='scene_name', values='initial_ate_weak').round(3)
    file = open(figure_path + 'initial_ate.txt', 'w')
    file.write(table.to_latex())
    file.close()
    
    # print table
    table = df.pivot_table(index='scene_name', columns='method_name', values='final_ate_weak')
    table = table[methods]
    table = table.reindex(scene_names).round(3)
    file = open(figure_path + 'final_ate.txt', 'w')
    file.write(table.to_latex())
    file.close()
    

if __name__ == '__main__':
    # resave_tables()
    # exit()

    detections_folder = "/media/lachness/DATA/git_ws/associated_tum_detections/associated_detections/"
    trajectories_folder = "/media/lachness/DATA/experiment_ws/_quadricslams/quadric_slam/datasets/TUM/trajectories/"

    # load system configuration
    config_path = '/home/lachness/git_ws/quadricslam/experiments/configs/tum.yaml'
    config = yaml.safe_load(open(config_path, 'r'))

    opt_params = load_optimizer_parms(config)

    # define filter bounds
    filter_pixels = 20
    image_bounds = gtsam_quadrics.AlignedBox2(0,0,640.,480.)
    filter_bounds = image_bounds.vector() + np.array([1,1,-1,-1])*filter_pixels
    filter_bounds = gtsam_quadrics.AlignedBox2(filter_bounds)



    # # check initial ATE's
    # for scene_name in scene_names
    #     initial_trajectory, time_to_key = load_tum_trajectory(scene_name, 'fovis', trajectories_folder)
    #     true_trajectory, true_time_to_key = load_tum_trajectory(scene_name, 'gt', trajectories_folder)
    #     ate = evaluate_tum(initial_trajectory, true_trajectory, time_to_key, true_time_to_key, align_type='horn')

    # exit()

    name = 'tum_3sd_20pxfilter'
    data_path = '/home/lachness/git_ws/quadricslam/experiments/data/{}.json'.format(name)
    figure_path = data_path.replace('.json', '/').replace('/data/', '/figures/')
    os.makedirs(figure_path, exist_ok=True)



    # test default performance over sigma
    if False:
        sigma_range = np.logspace(-1,3,20)
        data = []
        for scene_name in scene_names:

            # load data from names
            odometry_name = 'fovisvo'
            calibration = scene_name_to_calibration[scene_name]
            initial_trajectory, time_to_key = load_tum_trajectory(scene_name, odometry_name, trajectories_folder)
            true_trajectory, true_time_to_key = load_tum_trajectory(scene_name, 'gt', trajectories_folder)
            measurements = load_tum_detections(scene_name, detections_folder, time_to_key)


            for sigma in sigma_range:
                # print('Testing scene:sigma  {}:{:.3f}                        '.format(scene_name,sigma), end='\r')
                
                # implement method 
                robust_estimator = None
                local_config = config.copy()
                local_config['QuadricSLAM.box_sd'] =  sigma
                
                # create system
                system = QuadricSLAM_Offline(calibration, opt_params, local_config)
                estimated_trajectory, estimated_quadrics = system.run(initial_trajectory, measurements, robust_estimator=robust_estimator)

                # evaluate performance
                initial_ate_weak = evaluate_tum(initial_trajectory, true_trajectory, time_to_key, true_time_to_key, align_type='weak')
                final_ate_weak = evaluate_tum(estimated_trajectory, true_trajectory, time_to_key, true_time_to_key, align_type='weak')
                # print('{:.3f} -> {:.3f}'.format(initial_ate_weak, final_ate_weak))

                data.append({
                    'scene_name': scene_name,
                    'method_name': 'Baseline',
                    'sigma': sigma,
                    'initial_ate_weak': initial_ate_weak,
                    'final_ate_weak': final_ate_weak,
                })


            # create dataframe
            df = pd.DataFrame(data)

            # calculate ate_decrease
            df['ate_decrease'] = 100*(df.initial_ate_weak - df.final_ate_weak)/df.initial_ate_weak

            # plot figures
            ax = prepare_figure('final_ate_weak vs sigma vs scenes')
            sns.lineplot(data=df, y='final_ate_weak', x='sigma', hue='scene_name', hue_order=scene_names)
            ax.set_xscale('log')
            ax = prepare_figure('ate_decrease vs sigma vs scenes')
            sns.lineplot(data=df, y='ate_decrease', x='sigma', hue='scene_name', hue_order=scene_names)
            ax.set_xscale('log')

            ax = prepare_figure('final_ate_weak vs sigma')
            sns.lineplot(data=df, y='final_ate_weak', x='sigma')
            ax.set_xscale('log')
            ax = prepare_figure('ate_decrease vs sigma')
            sns.lineplot(data=df, y='ate_decrease', x='sigma')
            ax.set_xscale('log')

            plt.draw()
            plt.pause(0.01)
        # plt.show()



    # test robust estimators over c
    if False:
        c_range = np.logspace(-1,1,20)
        data = []
        for scene_name in scene_names:
            for method_name in methods:
                if method_name not in name_to_estimator:
                    continue
                print('Testing scene:method  {}:{}                        '.format(scene_name, method_name), end='\r')
                for c in c_range: 

                    # load data from names
                    odometry_name = 'fovisvo'
                    calibration = scene_name_to_calibration[scene_name]
                    initial_trajectory, time_to_key = load_tum_trajectory(scene_name, odometry_name, trajectories_folder)
                    true_trajectory, true_time_to_key = load_tum_trajectory(scene_name, 'gt', trajectories_folder)
                    measurements = load_tum_detections(scene_name, detections_folder, time_to_key)
                    
                    # implement method 
                    local_config = config.copy()
                    robust_estimator = name_to_estimator[method_name](c)
                    
                    # create system
                    system = QuadricSLAM_Offline(calibration, opt_params, local_config)
                    estimated_trajectory, estimated_quadrics = system.run(initial_trajectory, measurements, robust_estimator=robust_estimator)

                    # evaluate performance
                    initial_ate_weak = evaluate_tum(initial_trajectory, true_trajectory, time_to_key, true_time_to_key, align_type='weak')
                    final_ate_weak = evaluate_tum(estimated_trajectory, true_trajectory, time_to_key, true_time_to_key, align_type='weak')

                    data.append({
                        'scene_name': scene_name,
                        'method_name': method_name,
                        'c': c,
                        'initial_ate_weak': initial_ate_weak,
                        'final_ate_weak': final_ate_weak,
                    })

        # create dataframe
        df = pd.DataFrame(data)

        # calculate ate_decrease
        df['ate_decrease'] = (df.initial_ate_weak - df.final_ate_weak)

        # plot figures
        for scene_name in scene_names:
            ax = prepare_figure('{}: final_ate_weak vs c vs m-estimator'.format(scene_name))
            sns.lineplot(data=df[df.scene_name==scene_name], y='final_ate_weak', x='c', hue='method_name')
            plt.xlabel('')
            plt.ylabel('ATE (m)')
            ax.set_xscale('log')
            save_current_figure(figure_path + '{}_filter_tune_ate.pdf')

            ax = prepare_figure('{}: ate_decrease vs c vs m-estimator'.format(scene_name))
            sns.lineplot(data=df[df.scene_name==scene_name], y='ate_decrease', x='c', hue='method_name')
            plt.xlabel('')
            plt.ylabel('ATE decrease (m)')
            ax.set_xscale('log')
            save_current_figure(figure_path + '{}_filter_tune_ate_decrease.pdf')

        try:
            save_data(data, data_path.replace('.json', '_tune_filter.json'))
        except:
            print('failed save')
            code.interact(local=locals())
        plt.show()
        code.interact(local=locals())



        
        
        
    
    

    data = []
    for scene_name in scene_names:

        for method_name in methods:
            print('Testing scene:method  {}:{}                        '.format(scene_name, method_name), end='\r')

            # load data from names
            odometry_name = 'fovis'
            calibration = scene_name_to_calibration[scene_name]
            initial_trajectory, time_to_key = load_tum_trajectory(scene_name, odometry_name, trajectories_folder)
            true_trajectory, true_time_to_key = load_tum_trajectory(scene_name, 'gt', trajectories_folder)
            measurements = load_tum_detections(scene_name, detections_folder, time_to_key)
            
            # implement method 
            robust_estimator = None
            local_config = config.copy()
            if method_name == 'Filtering':
                measurements = filter_measurements(measurements, bounds=filter_bounds)
            elif method_name == 'Complex':
                local_config['QuadricSLAM.error_type'] = 'COMPLEX'
            elif method_name == 'HighUncertainty':
                local_config['QuadricSLAM.box_sd'] = 110.0
            elif method_name in name_to_estimator:
                robust_estimator = name_to_estimator[method_name](name_to_parameter[method_name])

            # if scene_name == 'fr1_desk':
            #     measurements = remove_objects(measurements, [0, 3, 6, 22, 20, 24, 23, 19, 28, 26])
            # elif scene_name == 'fr1_desk2':
            #     measurements = remove_objects(measurements, [0, 39, 25, 20, 34, 29, 40, 4])
            # elif scene_name == 'fr2_desk':
            #     measurements = remove_objects(measurements, [1, 6, 10, 25, 30, 40, 11, 42, 38, 31, 28, 27, 20, 37, 26, 39, 8, 41, 43, 36])
            # elif scene_name == 'fr3_office':
            #     measurements = remove_objects(measurements, [0, 51, 49, 46, 54, 59, 245, 60, 50])
            
            # create system
            system = QuadricSLAM_Offline(calibration, opt_params, local_config)
            estimated_trajectory, estimated_quadrics = system.run(initial_trajectory, measurements, robust_estimator=robust_estimator, custom_noise=False)

            # evaluate performance
            initial_ate_weak = evaluate_tum(initial_trajectory, true_trajectory, time_to_key, true_time_to_key, align_type='weak')
            final_ate_weak = evaluate_tum(estimated_trajectory, true_trajectory, time_to_key, true_time_to_key, align_type='weak')
            # print('{:.3f} -> {:.3f}'.format(initial_ate_weak, final_ate_weak))

            data.append({
                'scene_name': scene_name,
                'method_name': method_name,
                'initial_ate_weak': initial_ate_weak,
                'final_ate_weak': final_ate_weak,
            })


    # create dataframe
    df = pd.DataFrame(data)

    # calculate ate_decrease
    df['ate_decrease'] = (df.initial_ate_weak - df.final_ate_weak)

    # present initial ate as a method
    table = df[df.method_name=='Baseline'].pivot_table(index='scene_name', values='initial_ate_weak').round(3)
    file = open(figure_path + 'initial_ate.txt', 'w')
    file.write(table.to_latex())
    file.close()
    
    # print table
    table = df.pivot_table(index='scene_name', columns='method_name', values='final_ate_weak')
    table = table[methods]
    table = table.reindex(scene_names).round(3)
    file = open(figure_path + 'final_ate.txt', 'w')
    file.write(table.to_latex())
    file.close()

    # plot figures
    ax = prepare_figure('custom_noise scene vs method vs final_ate_weak')
    sns.barplot(data=df, y='final_ate_weak', x='scene_name', hue='method_name', hue_order=methods)
    plt.legend(bbox_to_anchor=(1.05,0.9), loc=2, borderaxespad=0.)
    save_current_figure(figure_path + '{}_ate.pdf')
    # ax = prepare_figure('scene vs method vs initial_ate_weak')
    # sns.barplot(data=df, y='initial_ate_weak', x='scene_name', hue='method_name', hue_order=methods)
    # plt.legend(bbox_to_anchor=(1.05,0.9), loc=2, borderaxespad=0.)
    ax = prepare_figure('custom_noise scene vs method vs ate_decrease')
    sns.barplot(data=df, y='ate_decrease', x='scene_name', hue='method_name', hue_order=methods)
    plt.legend(bbox_to_anchor=(1.05,0.9), loc=2, borderaxespad=0.)
    save_current_figure(figure_path + '{}_ate_decrease.pdf')


    # ax = prepare_figure('method vs final_ate_weak')
    # sns.boxplot(data=df, y='final_ate_weak', x='method_name')
    # ax = prepare_figure('method vs ate_decrease')
    # sns.boxplot(data=df, y='ate_decrease', x='method_name')
    
    # plt.xlabel('')
    # plt.ylabel(ylabel)
    # self.save_current_figure(figure_path + '{}_vs_conditions_vs_method.pdf'.format(yname))

    save_data(data, data_path.replace('.json', '_test.json'))

    plt.show()
    code.interact(local=locals())