"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Evaluation interface 
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
import numpy as np

# import gtsam and extension
import gtsam

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

# import custom python modules
sys.dont_write_bytecode = True
from visualization.drawing import Drawing


class Evaluation(object):
    @staticmethod
    def evaluate_trajectory(estimated_trajectory, true_trajectory, horn=True):
        # align trajectories
        if horn:
            aligned_trajectory = Evaluation.horn_align(estimated_trajectory, true_trajectory)
        else:
            aligned_trajectory = Evaluation.weak_align(estimated_trajectory, true_trajectory)
        
        # Drawing.draw_results([estimated_trajectory, aligned_trajectory, true_trajectory], [], ['m', 'c', 'g'])

        # evaluate metrics
        rmse_ATE = Evaluation.ATE(aligned_trajectory, true_trajectory)
        rmse_RPE = Evaluation.RPE(aligned_trajectory, true_trajectory)
        # print('         rmse_ATE:  {:.4f}m'.format(rmse_ATE))
        # print('         rmse_RPE:  {:.4f}m'.format(rmse_RPE))
        # for i in range(len(true_trajectory)):
        #     r = Evaluation.window(aligned_trajectory, true_trajectory, i)
        #     print('         rmse_WIN{}: {:.4f}m'.format(i, r))
        return [rmse_ATE, rmse_RPE]



    @staticmethod
    def ATE(trajectory1, trajectory2):
        """ point error = || x ominus x* || """ 
        assert len(trajectory1) == len(trajectory2)
        trans_errors = []
        for p1, p2 in zip(trajectory1.data(), trajectory2.data()):
            point_error = np.linalg.norm(p1.between(p2).translation().vector())
            trans_errors.append(point_error)
        trans_errors = np.array(trans_errors)
        rmse = np.sqrt(np.average(trans_errors**2))
        return rmse
    
    @staticmethod
    def RPE(trajectory1, trajectory2):
        """ 
        Alignes each pose p and calculates the ate between the next poses
        point error = || rp ominus rp* || 
        """ 
        assert len(trajectory1) == len(trajectory2)
        trans_errors = []
        for i,j in zip(range(len(trajectory1)-1), range(1,len(trajectory1))):
            ap1 = trajectory1[i].between(trajectory1[j])
            ap2 = trajectory2[i].between(trajectory2[j])
            pose_diff = ap1.between(ap2)
            trans_errors.append(np.linalg.norm(pose_diff.translation().vector()))
        trans_errors = np.array(trans_errors)
        rmse = np.sqrt(np.average(trans_errors**2))
        return rmse

    @staticmethod
    def window(trajectory1, trajectory2, window):
        """
        Aligns a segment of the trajectory of length window
        Then calculates the ATE between the segments
        """
        # assert len(trajectory1) == len(trajectory2)
        # trans_errors = []
        # for i in range(len(trajectory1)-window):
        #     ap1 = trajectory1[i].between(trajectory1[i+window])
        #     ap2 = trajectory2[i].between(trajectory2[i+window])
        #     pose_diff = ap1.between(ap2)
        #     trans_errors.append(np.linalg.norm(pose_diff.translation().vector()))
        # trans_errors = np.array(trans_errors)
        # rmse = np.sqrt(np.average(trans_errors**2))
        # return rmse
        pass
            

    @staticmethod
    def horn_align(trajectory1, trajectory2):
        """ Aligns trajectory1 with trajectory2 using HORN """
        xyz1 = np.matrix([p.translation().vector() for p in trajectory1.data()]).transpose()
        xyz2 = np.matrix([p.translation().vector() for p in trajectory2.data()]).transpose()
        R,T,trans_error = Evaluation._horn_align(xyz1, xyz2)
        transform = gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(np.array(T)[:,0])) # T = [[x],[y],[z]] [:,0] = tranpose()[0]
        warped_trajectory = trajectory1.applyTransform(transform)
        # print('trans_error: ', np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)))
        return warped_trajectory
        
    @staticmethod
    def weak_align(trajectory1, trajectory2):
        """ Aligns trajectory1 with trajectory 2 """
        transform = trajectory1[0].between(trajectory2[0])
        warped_trajectory = trajectory1.applyTransform(transform)
        return warped_trajectory

    @staticmethod
    def _horn_align(model, data):
        """Align two trajectories using the method of Horn (closed-form).

        Input:
        model -- first trajectory (3xn)
        data -- second trajectory (3xn)

        Output:
        rot -- rotation matrix (3x3)
        trans -- translation vector (3x1)
        trans_error -- translational error per point (1xn)

        From: https://vision.in.tum.de/data/datasets/rgbd-dataset/tools

        """
        model_zerocentered = model - model.mean(1)
        data_zerocentered = data - data.mean(1)

        W = np.zeros( (3,3) )
        for column in range(model.shape[1]):
            W += np.outer(model_zerocentered[:,column],data_zerocentered[:,column])
        U,d,Vh = np.linalg.linalg.svd(W.transpose())
        S = np.matrix(np.identity( 3 ))
        if(np.linalg.det(U) * np.linalg.det(Vh)<0):
            S[2,2] = -1
        rot = U*S*Vh
        trans = data.mean(1) - rot * model.mean(1)

        model_aligned = rot * model + trans
        alignment_error = model_aligned - data

        trans_error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),0)).A[0]

        return rot,trans,trans_error


