/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DualEllipsoid.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a constrained dual quadric 
 */

#include <quadricslam/geometry/DualEllipsoid.h>
#include <quadricslam/base/NotImplementedException.h>

#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
DualEllipsoid::DualEllipsoid() {
  pose_ = Pose3();
  radii_ = Vector3(0.25, 0.25, 0.25);
}

/* ************************************************************************* */
DualEllipsoid::DualEllipsoid(const Matrix44& dQ) {
  // TODO: implement
  throw NotImplementedException();
}

/* ************************************************************************* */
// TODO: check dims > 0
DualEllipsoid::DualEllipsoid(const Pose3& pose, const Vector3& radii) {
  pose_ = pose;
  radii_ = radii;
}

/* ************************************************************************* */
DualEllipsoid::DualEllipsoid(const Rot3& R, const Point3& t, const Vector3& s) {
  pose_ = Pose3(R, t);
  radii_ = s;
}

/* ************************************************************************* */
Matrix44 DualEllipsoid::matrix() const {
  return pose_.matrix() * (Vector4() << radii_, -1.0).finished().asDiagonal() * pose_.matrix().transpose();
}

/* ************************************************************************* */
// TODO: vectorize, use AlignedBox3
Vector6 DualEllipsoid::bounds3() const {
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
DualEllipsoid DualEllipsoid::retract(const Vector9& v) const {
  Pose3 pose = pose_.retract(v.head<6>());
  Vector3 radii = radii_ + v.tail<3>();
  return DualEllipsoid(pose, radii);
}

/* ************************************************************************* */
Vector9 DualEllipsoid::localCoordinates(const DualEllipsoid& other) const {
  Vector9 v = Vector9::Zero();
  v.head<6>() = pose_.localCoordinates(other.pose_);
  v.tail<3>() = other.radii_ - radii_;
  return v;
}

/* ************************************************************************* */
void DualEllipsoid::print(const std::string& s) const {
  cout << s << " : \n" << this->matrix() << endl;
}

/* ************************************************************************* */
bool DualEllipsoid::equals(const DualEllipsoid& other, double tol) const {
  return this->matrix().isApprox(other.matrix(), tol);
}

} // namespace gtsam