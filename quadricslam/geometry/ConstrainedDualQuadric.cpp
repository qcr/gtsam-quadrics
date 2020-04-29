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
#include <quadricslam/base/Jacobians.h>

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
Matrix44 ConstrainedDualQuadric::matrix(OptionalJacobian<16,9> dQ_dq) const {
  Matrix44 Z = pose_.matrix();
  Matrix44 Qc = (Vector4() << (radii_).array().pow(2), -1.0).finished().asDiagonal();
  Matrix44 Q = Z * Qc * Z.transpose();

  if (dQ_dq) {
    using namespace internal;

    // NOTE: this will recalculate pose.matrix
    // NOTE: pose.matrix derivative will also cost an extra Local()
    Eigen::Matrix<double, 16,6> dZ_dx;
    internal::matrix(pose_, dZ_dx);
    Eigen::Matrix<double, 16,9> dZ_dq = Matrix::Zero(16,9);
    dZ_dq.block(0,0,16,6) = dZ_dx;


    Eigen::Matrix<double, 16,9> dQc_dq = Matrix::Zero(16,9);
    dQc_dq(0,6) = 2.0 * radii_(0);
    dQc_dq(5,7) = 2.0 * radii_(1);
    dQc_dq(10,8) = 2.0 * radii_(2);
    
    *dQ_dq = kron(I44, Z*Qc) * T44 * dZ_dq  +  kron(Z, I44) * (kron(I44, Z)*dQc_dq + kron(Qc, I44)*dZ_dq);

    // cout << "DEBUG dZ_dq: \n" << dZ_dq << endl << endl;
    // cout << "DEBUG dQc_dq: \n" << dQc_dq << endl << endl;
  }
  return Q;
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
ConstrainedDualQuadric ConstrainedDualQuadric::addNoise(double sd) {
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, sd);
  vector<double> poseNoise(6); 
  vector<double> radiiNoise(3); 
  std::generate(poseNoise.begin(), poseNoise.end(), [&]{return distribution(generator);});
  std::generate(radiiNoise.begin(), radiiNoise.end(), [&]{return distribution(generator);});

  Pose3 poseDelta = Pose3::Retract(Vector6(poseNoise.data()));
  Vector3 radiiDelta = Vector3(radiiNoise.data());

  Pose3 noisyPose = pose_.compose(poseDelta);
  Vector3 noisyRadii = radii_ + radiiDelta;

  return ConstrainedDualQuadric(noisyPose, noisyRadii);    
}

/* ************************************************************************* */
ConstrainedDualQuadric ConstrainedDualQuadric::Retract(const Vector9& v) {
  Pose3 pose = Pose3::Retract(v.head<6>());
  Vector3 radii = v.tail<3>();
  return ConstrainedDualQuadric(pose, radii);
}

/* ************************************************************************* */
Vector9 ConstrainedDualQuadric::LocalCoordinates(const ConstrainedDualQuadric& q) {
  Vector9 v = Vector9::Zero();
  v.head<6>() = Pose3::LocalCoordinates(q.pose_);
  v.tail<3>() = q.radii_;
  return v;
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
  cout << s << " : " << endl;
  cout << "QuadricPose\n" << pose_.matrix() << endl;
  cout << "QuadricRadii: " << radii_.transpose() << endl;
  cout << "QuadricMatrix\n" << this->matrix() << endl;
}

/* ************************************************************************* */
bool ConstrainedDualQuadric::equals(const ConstrainedDualQuadric& other, double tol) const {
  return this->matrix().isApprox(other.matrix(), tol);
}

} // namespace gtsam