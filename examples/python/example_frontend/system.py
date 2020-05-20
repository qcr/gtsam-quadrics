"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: System (front-end) interface 
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# import gtsam and extension
import gtsam
import quadricslam

# import custom python modules
sys.dont_write_bytecode = True
from dataset_interfaces.simulated_dataset import ManualSequence
from dataset_interfaces.scenenet_dataset import SceneNetDataset
from visualization.drawing import Drawing
from base.evaluation import Evaluation
from base.containers import *


class System(object):
    """
    Python front-end to build graph/estimate from dataset. 
    """
    X = lambda i: int(gtsam.symbol(ord('x'), i))
    Q = lambda i: int(gtsam.symbol(ord('q'), i))
    L = lambda i: int(gtsam.symbol(ord('l'), i))
    ZERO = 1e-8

    @staticmethod
    def run(sequence):

        # build graph / estimate
        graph, initial_estimate = System.build_graph(sequence)

        # draw factor graph 
        # Drawing.draw_problem(graph, initial_estimate)

        # check graph + estimate
        System.check_problem(graph, initial_estimate)

        # optimize using c++ back-end
        estimate = System.optimize(graph, initial_estimate, sequence.calibration)

        # draw estimation
        # Drawing.draw_problem(graph, estimate)

        # # extract quadrics / trajectory 
        # estimated_trajectory = Trajectory.from_values(estimate)
        # estimated_quadrics = Quadrics.from_values(estimate)

        # # evaluate results
        # initial_ATE = Evaluation.evaluate_trajectory(Trajectory.from_values(initial_estimate), sequence.true_trajectory, horn=True)
        # estimate_ATE = Evaluation.evaluate_trajectory(estimated_trajectory, sequence.true_trajectory, horn=True)
        # print('Horn, initial_ATE: {}'.format(initial_ATE))
        # print('Horn, estimate_ATE: {}'.format(estimate_ATE))
        # initial_ATE = Evaluation.evaluate_trajectory(Trajectory.from_values(initial_estimate), sequence.true_trajectory, horn=False)
        # estimate_ATE = Evaluation.evaluate_trajectory(estimated_trajectory, sequence.true_trajectory, horn=False)
        # print('initial_ATE: {}'.format(initial_ATE))
        # print('estimate_ATE: {}'.format(estimate_ATE))

        # # plot results
        # trajectories = [Trajectory.from_values(initial_estimate), estimated_trajectory, sequence.true_trajectory]
        # quadrics = [Quadrics.from_values(initial_estimate), estimated_quadrics, sequence.true_quadrics]
        # Drawing.draw_results(trajectories, quadrics, ['r','m','g'])

        


    @staticmethod
    def check_problem(graph, estimate):
        # check trajectory
        # must be prior pose 
        # must be odometry between each pose variable
        # check quadrics
        # each quadric must be viewed from 3 poses

        factor_keys = []
        for i in range(graph.size()):
            factor = graph.at(i)
            int_keys = [factor.keys().at(j) for j in range(factor.keys().size())]
            factor_keys += ['{}{}'.format(chr(gtsam.symbolChr(key)), gtsam.symbolIndex(key)) for key in int_keys]

        estimate_int_keys = [estimate.keys().at(i) for i in range(estimate.keys().size())]
        estimate_keys = ['{}{}'.format(chr(gtsam.symbolChr(key)), gtsam.symbolIndex(key)) for key in estimate_int_keys]

        # check each factor has a variable
        # and each quadric is viewed 3 times
        for factor_key in factor_keys:
            
            if factor_key not in estimate_keys:
                print(factor_key, 'doesnt exist in estimate!')

        for estimate_key in estimate_keys:

            if 'q' in estimate_key:

                mkeys = [fkey for fkey in factor_keys if fkey==estimate_key]
                if len(mkeys) < 3:
                    print(estimate_key, 'doesnt have 3 bbfs')
            
    
    @staticmethod
    def optimize(graph, initial_estimate, calibration):

        # create optimizer parameters
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM("SUMMARY")    # SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA : VALUES, ERROR 
        params.setMaxIterations(100)
        params.setlambdaInitial(1e-5)       # defaults to 1e5
        params.setlambdaUpperBound(1e10)     # defaults to 1e5
        params.setlambdaLowerBound(1e-8)    # defaults to 0.0
        params.setRelativeErrorTol(1e-10)   # stop iterating when change in error between steps is less than this
        params.setAbsoluteErrorTol(1e-8)    # stop when cost-costchange < tol
  
        # create optimizer
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)

        # plot initial estimate
        # Drawing.plot_problem(graph, initial_estimate, calibration)

        # iterate manually 
        # for i in range(100):
        #     optimizer.iterate()

        #     estimate = optimizer.values()

        #     # draw current iteration
        #     Drawing.plot_problem(graph, estimate, calibration)

        # run optimizer
        print('starting optimization')
        estimate = optimizer.optimize()
        print('optimization finished')

        return estimate


    @staticmethod
    def build_graph(sequence):
        """
        Sequence contains:
        * calibration
        * image_dimensions
        * true_trajectory 
        * true_quadrics 
        * true_boxes 
        """

        # create empty graph / estimate
        graph = gtsam.NonlinearFactorGraph()
        initial_estimate = gtsam.Values()

        # declare noise models
        ODOM_SIGMA = 0.01; BOX_SIGMA = 0.5
        ODOM_NOISE = 0.01; BOX_NOISE = 0.0
        noise_zero = gtsam.noiseModel_Diagonal.Sigmas(np.array([System.ZERO]*6, dtype=np.float))
        odometry_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([ODOM_SIGMA]*3 + [ODOM_SIGMA]*3, dtype=np.float))
        bbox_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([BOX_SIGMA]*4, dtype=np.float))
        X = lambda i: int(gtsam.symbol(ord('x'), i))
        Q = lambda i: int(gtsam.symbol(ord('q'), i))

        # get noisy odometry / boxes 
        true_odometry = sequence.true_trajectory.as_odometry()
        noisy_odometry = true_odometry.add_noise(mu=0.0, sd=ODOM_NOISE)
        noisy_boxes = sequence.true_boxes.add_noise(mu=0.0, sd=BOX_NOISE)

        # initialize trajectory
        # TODO: ensure aligned in same reference frame
        initial_trajectory = noisy_odometry.as_trajectory(sequence.true_trajectory.data()[0])
        # initial_trajectory = noisy_odometry.as_trajectory()

        # initialize quadrics
        # NOTE: careful initializing with true quadrics and noise traj as it may not make sense
        # initial_quadrics = sequence.true_quadrics
        initial_quadrics = System.initialize_quadrics(initial_trajectory, noisy_boxes, sequence.calibration)

        # add prior pose
        initial_trajectory.add_prior(graph, noise_zero)

        # add odometry measurements
        noisy_odometry.add_factors(graph, odometry_noise)

        # add valid box measurements
        valid_objects = []
        initialized_quadrics = initial_quadrics.keys()
        for object_key in np.unique(noisy_boxes.object_keys()):

            # add if quadric initialized
            if object_key in initialized_quadrics:
                
                # get all views of quadric
                object_boxes = noisy_boxes.at_object(object_key)

                # add if enough views
                if len(object_boxes) > 3:

                    # add measurements
                    valid_objects.append(object_key)
                    for (pose_key, t), box in object_boxes.items():
                        bbf = quadricslam.BoundingBoxFactor(box, sequence.calibration, sequence.image_dimensions, System.X(pose_key), System.Q(object_key), bbox_noise)
                        bbf.addToGraph(graph)

        # add initial pose estimates
        initial_trajectory.add_estimates(initial_estimate)
        
        # add initial landmark estimates
        for object_key, quadric in initial_quadrics.items():

            # add if seen > 3 times
            if (object_key in valid_objects):
                quadric.addToValues(initial_estimate, System.Q(object_key))

        return graph, initial_estimate

    @staticmethod
    def initialize_quadrics(trajectory, boxes, calibration):
        quadrics = Quadrics()

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

            # initialize and constrain quadric
            quadric_matrix = System.initialize_quadric(poses, object_boxes, calibration)
            # quadric = System.constrain_quadric(quadric_matrix)
            # quadric = quadricslam.ConstrainedDualQuadric.constrain(quadric_matrix)
            quadric = quadricslam.ConstrainedDualQuadric(quadric_matrix)

            # check quadric is okay
            if (System.is_okay(quadric, poses, calibration)):
                quadrics.add(quadric, object_key)

        return quadrics

    @staticmethod
    def initialize_quadric(poses, object_boxes, calibration):
        """ calculates quadric_matrix using SVD """

        # iterate through box/pose data
        planes = []
        for box, pose in zip(object_boxes.data(), poses.data()):

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
            
    @staticmethod
    def constrain_quadric(dual_quadric_matrix):
        """ constrains quadric using method in paper """

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
        ellipsoid = quadricslam.ConstrainedDualQuadric(gtsam.Rot3(r1), gtsam.Point3(t), s)
        ellipsoid_vector = quadricslam.ConstrainedDualQuadric.LocalCoordinates(ellipsoid)


        # check if rotation valid
        if np.isclose(np.linalg.det(r1), -1.0) or np.any(np.isinf(ellipsoid_vector)) or np.any(np.isnan(ellipsoid_vector)):

            # identify bad rotation
            # print('[SVD] estimated quadric rotation invalid, resolving by inverting axis')

            # flip if rotation is improper
            AxisFlip = np.eye(3); AxisFlip*=-1.0;
            r2 = r1.dot(AxisFlip)
            ellipsoid_2 = quadricslam.ConstrainedDualQuadric(gtsam.Rot3(r2), gtsam.Point3(t), s)
            ellipsoid_vector_2 = quadricslam.ConstrainedDualQuadric.LocalCoordinates(ellipsoid_2)

            # check if still broken
            if np.isclose(np.linalg.det(r2), -1.0) or np.any(np.isinf(ellipsoid_vector_2)) or np.any(np.isnan(ellipsoid_vector_2)):
                print('\n\n ~~~~~ STILL BROKEN ~~~~~~ \n\n')

            # save result
            ellipsoid = ellipsoid_2

        return ellipsoid

    @staticmethod
    def is_okay(quadric, poses, calibration):
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
        return True




def test_conic_bounds():

    # # c = quadricslam.DualConic(gtsam.Pose2(50.,50.,0.0), np.array([15.,15.]))
    # # bounds = c.smartBounds().vector()
    # # print(bounds)

    # calibration = gtsam.Cal3_S2(525.0, 525.0, 0.0, 160.0, 120.0)
    # dC = np.array([
    #     [-1.02605e+06, -1.45094e+06,     -10709.8],
    #     [-1.45094e+06,  1.56826e+06,     -11339.9],
    #     [-10709.8,     -11339.9,     -82.2568]])
    # C = np.linalg.inv(dC)
    # C = C/C[-1,-1]
    # print("C1: ", C)
    # # C = np.array([[6.93513e-05, -5.64969e-07,   -0.0089516],
    # #                 [-5.64969e-07,  8.15752e-06,  -0.00105104],
    # #                 [-0.0089516,  -0.00105104,            1]])
    # # print("C2: ", C)
    # c = quadricslam.DualConic(dC)
    # print('bounds: ', c.bounds().vector())

    # # a = C[0,0]; b = C[1,0]*2.0; c = C[1,1]
    # # d = C[2,0]*2.0; e = C[2,1]*2.0; f = C[2,2]
    # # conic_det = C[0,0]*((C[1,1]*C[2,2]) - 
	# # 			 (C[2,1]*C[1,2])) -C[0,1]*(C[1,0]*C[2,2] - 
	# # 			  C[2,0]*C[1,2]) + C[0,2]*(C[1,0]*C[2,1] - C[2,0]*C[1,1])
    # # n = -1*(int(conic_det > 0.0) - int(conic_det < 0.0))
    # # ecc = np.sqrt((2.0*np.sqrt((a-c)**2+b**2))/(n*(a+c)+np.sqrt((a-c)**2+b**2)))
    # # print(conic_det, n, ecc)


    # def get_points_at_x(C, x):
    #     poly = [C[1,1],
    #             2*C[0,1]*x + 2*C[1,2],
    #             C[0,0]*x*x + 2*C[0,2]*x + C[2,2]]
    #     disc = poly[1]**2 - 4.0*poly[0]*poly[2]
    #     print(disc)
    #     roots = np.roots(poly)
    #     return roots

    # def get_points_at_y(C, y):
    #     poly = [C[0,0],
    #             2*C[0,1]*y + 2*C[0,2],
    #             C[1,1]*y*y + 2*C[1,2]*y + C[2,2]]
    #     disc = poly[1]**2 - 4.0*poly[0]*poly[2]
    #     print(disc)
    #     roots = np.roots(poly)
    #     return roots
    
    # bounds = c.bounds().vector()
    # # p1ys = 
    # print(get_points_at_x(C, bounds[0]))
    # # p2xs = 
    # print(get_points_at_y(C, bounds[1]))
    # # p3ys = 
    # print(get_points_at_x(C, bounds[2]))
    # # p4xs = 
    # print(get_points_at_y(C, bounds[3]))

    # print(c.smartBounds(calibration).vector())

    # dC = np.array([[-4.1379e+06, 2.13677e+07,    -12504.5],
    #                 [2.13677e+07, -1.0932e+08,     63999.2],
    #                 [-12504.5,     63999.2,     -37.455]])
    # conic = quadricslam.DualConic(dC)

    qv = np.array([-5.34054,    3.89165,   -8.37635,  -0.306262,   0.203986,  -0.010365,   0.544028,  0.0982984, -0.0842678])
    quadric = quadricslam.ConstrainedDualQuadric(qv)
    # quadric.fullyVisible()
    # quadric.partiallyVisible()
    # quadric.notVisible()
    # Drawing.mpl_draw_conic(dC)
    plt.show()


if __name__ == '__main__':
    np.random.seed(121)

    # trainval = 'train'
    # dataset_path = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/data/{}'.format(trainval)
    # protobuf_folder = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/data/{}_protobufs'.format(trainval)
    # reader_path = '/media/feyre/DATA1/Datasets/SceneNetRGBD/pySceneNetRGBD/scenenet_pb2.py'
    # dataset = SceneNetDataset(dataset_path, protobuf_folder, reader_path)
    # System.run(dataset[0])

    # test_conic_bounds()

    sequence = ManualSequence.sequence1()
    System.run(sequence)

    # for i in range(10):
    #     System.run(sequence)



    # test graph
    # of = gtsam.BetweenFactorPose3(gtsam.symbol(ord('x'),3), gtsam.symbol(ord('x'),4), gtsam.Pose3(), gtsam.noiseModel_Diagonal.Sigmas(np.array([1.,2,3,4,5,6])))
    # bbf = quadricslam.BoundingBoxFactor(quadricslam.AlignedBox2(0.,1.,2.,3.), gtsam.Cal3_S2(), np.array([320.,240.]), gtsam.symbol(ord('x'),3), gtsam.symbol(ord('q'),1), gtsam.noiseModel_Diagonal.Sigmas(np.array([1.]*4)))
    # graph = gtsam.NonlinearFactorGraph()

    # graph.add(of)
    # # graph.add(bbf)
    # bbf.addToGraph(graph)
    # graph.add(of)

    # print(graph.at(0))
    # print(graph.at(1))
    # print(graph.at(2))

    # import code
    # code.interact(local=locals())

    # print(quadricslam.BoundingBoxFactor.getFromGraph(graph, 0))
    # print(quadricslam.BoundingBoxFactor.getFromGraph(graph, 1))
    # print(quadricslam.BoundingBoxFactor.getFromGraph(graph, 2))

    # for i in range(3):
    #     print(graph.exists(i))
    #     print(quadricslam.BoundingBoxFactor.getFromGraph(graph, 1))
    #     print(graph.at(i))
    


