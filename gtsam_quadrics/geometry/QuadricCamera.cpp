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

#include <gtsam_quadrics/geometry/QuadricCamera.h>
#include <gtsam_quadrics/base/Utilities.h>

#include <gtsam/base/numericalDerivative.h>

namespace gtsam {

/* ************************************************************************* */
Matrix34 QuadricCamera::transformToImage(const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration) {
  Matrix3 image_T_camera = calibration->K();
  Matrix4 camera_T_world = pose.inverse().matrix();
  Matrix34 image_T_world = image_T_camera * (camera_T_world).block(0,0,3,4);
  // Matrix34 image_T_world = image_T_camera * internal::I34 * camera_T_world;
  return image_T_world;
}

/* ************************************************************************* */
// NOTE: requires updating jacobians if we normalize q/c
// this wont happen if we split it into sub functions and just combine jacobians
DualConic QuadricCamera::project(const ConstrainedDualQuadric& quadric, const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration, 
  OptionalJacobian<9,9> dC_dq, OptionalJacobian<9,6> dC_dx) {
  
  // first retract quadric and pose to compute dX:/dx and dQ:/dq
  Matrix3 K = calibration->K();
  Matrix4 Xi = pose.inverse().matrix();
  static Matrix34 I34 = Matrix::Identity(3,4);
  Matrix34 P = K * I34 * Xi;
  Matrix4 Q = quadric.matrix();
  Matrix3 C = P * Q * P.transpose();
  DualConic dualConic(C);

  if (dC_dq) {
    Eigen::Matrix<double, 9,16> dC_dQ = utils::kron(P, P);
    Eigen::Matrix<double, 16,9> dQ_dq; quadric.matrix(dQ_dq); // NOTE: this recalculates quadric.matrix
    *dC_dq = dC_dQ * dQ_dq;
  } 
    
  if (dC_dx) {
    using utils::kron;
    static Matrix33 I33 = Matrix::Identity(3,3);
    static Matrix44 I44 = Matrix::Identity(4,4);
    Eigen::Matrix<double, 9,12> dC_dP = kron(I33, P*Q) * utils::TVEC(3,4) + kron(P*Q.transpose(), I33);
    Eigen::Matrix<double, 12,16> dP_dXi = kron(I44, K*I34);
    Eigen::Matrix<double, 16,16> dXi_dX = -kron(Xi.transpose(), Xi);
    Eigen::Matrix<double, 16,6> dX_dx; utils::matrix(pose, dX_dx);
    *dC_dx = dC_dP * dP_dXi * dXi_dX * dX_dx;
  }

  return dualConic;
}

/* ************************************************************************* */
std::vector<Vector4> QuadricCamera::project(const AlignedBox2& box, const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration) {
  std::vector<Vector4> planes;
  for (auto line : box.lines()) {
    Vector4 plane = QuadricCamera::transformToImage(pose, calibration).transpose() * line;
    planes.push_back(plane);
  }
  return planes;
}


} // namespace gtsam
