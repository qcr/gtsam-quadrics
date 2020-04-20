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
  Matrix6 dPoseInv_dPose = Matrix6::Zero();
  Matrix4 camera_T_world = pose().inverse(&dPoseInv_dPose).matrix();
  
  Matrix34 image_T_world = image_T_camera * (camera_T_world).block(0,0,3,4);
  
  if (dP_dCamera) {
    *dP_dCamera = Matrix33::Identity() * 
  }
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
    
} // namespace gtsam
