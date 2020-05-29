/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ConstrainedDualQuadric.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a constrained dual quadric 
 */

#include <quadricslam/geometry/ConstrainedDualQuadric.h>
#include <quadricslam/base/NotImplementedException.h>

#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
ConstrainedDualQuadric::ConstrainedDualQuadric() {
  pose_ = Pose3();
  radii_ = Vector3(1,1,1);
}

/* ************************************************************************* */
ConstrainedDualQuadric::ConstrainedDualQuadric(const Matrix44& dQ) {
  // TODO: implement
  throw NotImplementedException();
}

/* ************************************************************************* */
// TODO: check dims > 0
ConstrainedDualQuadric::ConstrainedDualQuadric(const Pose3& pose, const Vector3& radii) {
  pose_ = pose;
  radii_ = radii;
}

/* ************************************************************************* */
ConstrainedDualQuadric::ConstrainedDualQuadric(const Rot3& R, const Point3& t, const Vector3& r) {
  pose_ = Pose3(R, t);
  radii_ = r;
}

/* ************************************************************************* */
Matrix44 ConstrainedDualQuadric::matrix() const {
  return pose_.matrix() * (Vector4() << (radii_).array().pow(2), -1.0).finished().asDiagonal() * pose_.matrix().transpose();
}

/* ************************************************************************* */
// TODO: vectorize, use AlignedBox3
Vector6 ConstrainedDualQuadric::bounds() const {
  Matrix44 dE = this->matrix();
  double x_min = (dE(0,3) + std::sqrt(dE(0,3) * dE(0,3) - (dE(0,0) * dE(3,3)))) / dE(3,3);
  double y_min = (dE(1,3) + std::sqrt(dE(1,3) * dE(1,3) - (dE(1,1) * dE(3,3)))) / dE(3,3);
  double z_min = (dE(2,3) + std::sqrt(dE(2,3) * dE(2,3) - (dE(2,2) * dE(3,3)))) / dE(3,3);
  double x_max = (dE(0,3) - std::sqrt(dE(0,3) * dE(0,3) - (dE(0,0) * dE(3,3)))) / dE(3,3);
  double y_max = (dE(1,3) - std::sqrt(dE(1,3) * dE(1,3) - (dE(1,1) * dE(3,3)))) / dE(3,3);
  double z_max = (dE(2,3) - std::sqrt(dE(2,3) * dE(2,3) - (dE(2,2) * dE(3,3)))) / dE(3,3);
  return (Vector6() << x_min, y_min, z_min, x_max, y_max, z_max).finished();
}

/* ************************************************************************* */
ConstrainedDualQuadric ConstrainedDualQuadric::retract(const Vector9& v) const {
  Pose3 pose = pose_.retract(v.head<6>());
  Vector3 radii = radii_ + v.tail<3>();
  return ConstrainedDualQuadric(pose, radii);
}

/* ************************************************************************* */
Vector9 ConstrainedDualQuadric::localCoordinates(const ConstrainedDualQuadric& other) const {
  Vector9 v = Vector9::Zero();
  v.head<6>() = pose_.localCoordinates(other.pose_);
  v.tail<3>() = other.radii_ - radii_;
  return v;
}

/* ************************************************************************* */
void ConstrainedDualQuadric::print(const std::string& s) const {
  cout << s << " : \n" << this->matrix() << endl;
}

/* ************************************************************************* */
bool ConstrainedDualQuadric::equals(const ConstrainedDualQuadric& other, double tol) const {
  return this->matrix().isApprox(other.matrix(), tol);
}

} // namespace gtsam