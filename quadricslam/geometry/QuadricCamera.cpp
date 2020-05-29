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
QuadricCamera QuadricCamera::Create(const Pose3& pose, const boost::shared_ptr<Cal3_S2>& K, OptionalJacobian<6,6> H1, OptionalJacobian<6,5> H2) {
  return QuadricCamera(pose, K);
}


/* ************************************************************************* */
Matrix34 QuadricCamera::transformToImage() const {
  Matrix3 image_T_camera = calibration().K();
  Matrix4 camera_T_world = pose().matrix().inverse();
  Matrix34 image_T_world = image_T_camera * (camera_T_world).block(0,0,3,4);
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
