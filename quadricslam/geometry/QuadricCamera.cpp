/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file QuadricCamera.cpp
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief a class responsible for projecting quadrics
 */

#include <quadricslam/geometry/QuadricCamera.h>
#include <quadricslam/base/NotImplementedException.h>
#include <quadricslam/base/Jacobians.h>

namespace gtsam {

/* ************************************************************************* */
QuadricCamera QuadricCamera::Create(const Pose3& pose, const boost::shared_ptr<Cal3_S2>& K, OptionalJacobian<6,6> dCamera_dPose, OptionalJacobian<6,5> dCamera_dCalibration) {
  if (dCamera_dPose) {
    *dCamera_dPose = Matrix66::Identity();
  } if (dCamera_dCalibration) {
    *dCamera_dCalibration = Matrix65::Zero();
  }
  return QuadricCamera(pose, K);
}


/* ************************************************************************* */
// Note: will compute inverse camPose with jacobian regardless of OptionalJacobian
Matrix34 QuadricCamera::transformToImage(OptionalJacobian<12,6> dP_dCamera) const {
  
  Matrix3 image_T_camera = calibration().K();

  // can also use pose.rotation() and pose.translation() which comes with 3,6 jacobians
  // ^ how can we use this to get full 14,6 jacobian for matricision?
  // Matrix6 dPoseInv_dPose = Matrix6::Zero();
  // Matrix4 camera_T_world = pose().inverse(&dPoseInv_dPose).matrix();
  Matrix4 camera_T_world = pose().matrix().inverse();
  
  Matrix34 image_T_world = image_T_camera * (camera_T_world).block(0,0,3,4);
  
  // if (dP_dCamera) {
  //   *dP_dCamera = Matrix33::Identity() 
  // }
  return image_T_world;
}

/* ************************************************************************* */
DualConic QuadricCamera::project(const ConstrainedDualQuadric& quadric, OptionalJacobian<5,6> dC_dCamera, OptionalJacobian<5,9> dC_dQ) const {
  Matrix34 image_T_world = transformToImage(); 
  Matrix4 dQ = quadric.matrix();
  Matrix4 dQn = dQ/dQ(3,3);
  Matrix3 dC = image_T_world * dQn * image_T_world.transpose();
  return DualConic(dC);
}

// NOTE: requires updating jacobians if we normalize q/c
// this wont happen if we split it into sub functions and just combine jacobians
DualConic QuadricCamera::project(const ConstrainedDualQuadric& quadric, const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration, 
  OptionalJacobian<5,9> dc_dq, OptionalJacobian<5,6> dc_dx, OptionalJacobian<5,5> dc_dk) {
  
  using namespace internal;
  auto I44 = Matrix::Identity(4,4);
  auto I34 = Matrix::Identity(3,4);
  auto I33 = Matrix::Identity(3,3);

  Vector9 q;
  Vector6 x = Pose3::ChartAtOrigin::Local(pose, boost::none);
  Matrix3 K = calibration->K();
  Matrix4 X = pose.matrix();
  Matrix4 Xi = X.inverse();
  Matrix34 P = K * I34 * Xi;
  Matrix4 Q = quadric.matrix();
  Matrix3 C = P * Q * P.transpose();


  if (dc_dq) {
    Eigen::Matrix<double, 5,9> dc_dC;
    Eigen::Matrix<double, 9,16> dC_dQ = kron(P, P);

    // calculate math for Q(q), dQ_dq
    Eigen::Matrix<double, 16,9> dZ_dq; // from Pose3::Local(vec, H);
    Eigen::Matrix<double, 16,9> dQc_dq = Matrix::Zero(16,9); 
    dQc_dq(0,6) = 2.0 * q(6);
    dQc_dq(5,7) = 2.0 * q(7);
    dQc_dq(10,8) = 2.0 * q(8);
    
    Eigen::Matrix<double, 16,9> dQ_dq ;
  } if (dc_dx) {
    Eigen::Matrix<double, 5,9> dc_dC;
    Eigen::Matrix<double, 9,16> dC_dP = kron(I33, P*Q) * TVEC(3,4) + kron(P*Q.transpose(), I33);
    Eigen::Matrix<double, 16,16> dP_dXi = kron(I44, K*I34);
    Eigen::Matrix<double, 16,16> dXi_dX = -kron(Xi.transpose(), Xi);
    Eigen::Matrix<double, 16,6> dX_dx;
  }


}
    
} // namespace gtsam
