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
import code

# import gtsam and extension
import gtsam
import gtsam_quadrics

class Evaluation(object):
    @staticmethod
    def evaluate_map(estimated_quadrics, true_bounds, estimated_trajectory, true_trajectory, type):
        """
        Aligns quadrics->bounds using trajectory. 
        Calculates:
            1. RMSE centroid error
            2. IOU
            3. Aligned IOU
            4. width,height,length error 
        """
        # calculate transform to align trajectories
        transform = Evaluation.calculate_transform(estimated_trajectory, true_trajectory, type=type)

        # align quadrics with transform
        aligned_quadrics = estimated_quadrics.apply_transform(transform)

        # convert quadrics -> alignedbounds3
        estimated_bounds = {k:v.bounds() for k,v in aligned_quadrics.items()}

        trans_error = Evaluation.centroid_error(estimated_bounds, true_bounds)
        global_iou = Evaluation.global_overlap(estimated_bounds, true_bounds)
        aligned_iou = Evaluation.aligned_overlap(estimated_bounds, true_bounds)
        shape_error = Evaluation.shape_error(estimated_bounds, true_bounds)
        # code.interact(local=locals())

        return [trans_error, global_iou, aligned_iou, shape_error]
        
    @staticmethod
    def centroid_error(estimated_bounds, true_bounds):
        centroid_errors = []
        for object_key in estimated_bounds.keys():
            est = estimated_bounds[object_key]
            gt = true_bounds[object_key]
            centroid_errors.append( np.linalg.norm(est.centroid()-gt.centroid()) )
        centroid_errors = np.array(centroid_errors)
        return np.mean(centroid_errors)

    @staticmethod
    def global_overlap(estimated_bounds, true_bounds): 
        ious = []
        for object_key in estimated_bounds.keys():
            est = estimated_bounds[object_key]
            gt = true_bounds[object_key]
            ious.append( gt.iou(est) )
        return np.mean(ious)

    @staticmethod
    def aligned_overlap(estimated_bounds, true_bounds):
        ious = []
        for object_key in estimated_bounds.keys():
            est = estimated_bounds[object_key]
            est_centered = gtsam_quadrics.AlignedBox3( est.vector()-np.repeat(est.centroid(),2) )
            gt = true_bounds[object_key]
            gt_centered = gtsam_quadrics.AlignedBox3( gt.vector()-np.repeat(gt.centroid(),2) )
            ious.append( gt_centered.iou(est_centered) )
        return np.mean(ious)

    @staticmethod
    def shape_error(estimated_bounds, true_bounds):
        errors = []
        for object_key in estimated_bounds.keys():
            est = estimated_bounds[object_key]
            gt = true_bounds[object_key]

            errors.append( np.linalg.norm(est.dimensions() - gt.dimensions()) )
        return np.mean(errors)

    @staticmethod
    def evaluate_trajectory(estimated_trajectory, true_trajectory, type):
        # align trajectories
        transform = Evaluation.calculate_transform(estimated_trajectory, true_trajectory, type=type)
        aligned_trajectory = estimated_trajectory.applyTransform(transform)
        
        # evaluate metrics
        rmse_ATE = Evaluation.ATE(aligned_trajectory.values(), true_trajectory.values())
        rmse_RPE = Evaluation.RPE(aligned_trajectory.values(), true_trajectory.values())
        return [rmse_ATE, rmse_RPE]

    @staticmethod
    def ATE(trajectory1, trajectory2):
        """ point error = || x ominus x* || """ 
        assert len(trajectory1) == len(trajectory2)
        trans_errors = []
        for p1, p2 in zip(trajectory1, trajectory2):
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
    def calculate_transform(trajectory1, trajectory2, type):
        if type == 'horn':
            xyz1 = np.matrix([p.translation().vector() for p in trajectory1.values()]).transpose()
            xyz2 = np.matrix([p.translation().vector() for p in trajectory2.values()]).transpose()
            R,T,trans_error = Evaluation._horn_align(xyz1, xyz2)
            transform = gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(np.array(T)[:,0]))
        elif type == 'weak':
            transform = trajectory1[0].between(trajectory2[0])
        return transform

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


