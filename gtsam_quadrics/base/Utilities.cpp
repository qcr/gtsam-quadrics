/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Utilities.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a namespace providing a number of useful functions
 */

#include <gtsam_quadrics/base/Utilities.h>

#include <iostream>

using namespace std;

namespace gtsam_quadrics {
namespace utils {

/* ************************************************************************* */
gtsam::Vector2 solvePolynomial(const double& a, const double& b,
                               const double& c) {
  // calculate polynomial discrimenant
  double disc = b * b - 4.0 * a * c;

  if (disc < 1e-10) {
    disc = 0.0;
  }

  // throw exception if imaginary results
  if (disc < 0.0) {
    stringstream ss;
    ss << "poly solving failed, disc: " << disc << endl;
    throw std::runtime_error(ss.str());
  }

  // calculate and return roots
  double root1 = (-b + std::sqrt(disc)) / (2.0 * a);
  double root2 = (-b - std::sqrt(disc)) / (2.0 * a);
  return gtsam::Vector2(root1, root2);
}

/* ************************************************************************* */
gtsam::Vector2 getConicPointsAtX(
    const Eigen::Matrix<long double, 3, 3>& pointConic, const double& x) {
  const Eigen::Matrix<long double, 3, 3>& C = pointConic;
  return solvePolynomial(C(1, 1), 2 * C(0, 1) * x + 2 * C(1, 2),
                         C(0, 0) * x * x + 2 * C(0, 2) * x + C(2, 2));
}

/* ************************************************************************* */
gtsam::Vector2 getConicPointsAtY(
    const Eigen::Matrix<long double, 3, 3>& pointConic, const double& y) {
  const Eigen::Matrix<long double, 3, 3>& C = pointConic;
  return solvePolynomial(C(0, 0), 2 * C(0, 1) * y + 2 * C(0, 2),
                         C(1, 1) * y * y + 2 * C(1, 2) * y + C(2, 2));
}

/* ************************************************************************* */
gtsam::Pose3 interpolate(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
                         const double& percent) {
  return gtsam::interpolate<gtsam::Pose3>(p1, p2, percent);
}

/* ************************************************************************* */
gtsam::Matrix44 matrix(const gtsam::Pose3& pose,
                       gtsam::OptionalJacobian<16, 6> H) {
  gtsam::Matrix44 poseMatrix = pose.matrix();

  if (H) {
    H->setZero();
    (*H)(4, 0) = poseMatrix(0, 2);
    (*H)(5, 0) = poseMatrix(1, 2);
    (*H)(6, 0) = poseMatrix(2, 2);
    (*H)(8, 0) = -poseMatrix(0, 1);
    (*H)(9, 0) = -poseMatrix(1, 1);
    (*H)(10, 0) = -poseMatrix(2, 1);

    (*H)(0, 1) = -poseMatrix(0, 2);
    (*H)(1, 1) = -poseMatrix(1, 2);
    (*H)(2, 1) = -poseMatrix(2, 2);
    (*H)(8, 1) = poseMatrix(0, 0);
    (*H)(9, 1) = poseMatrix(1, 0);
    (*H)(10, 1) = poseMatrix(2, 0);

    (*H)(0, 2) = poseMatrix(0, 1);
    (*H)(1, 2) = poseMatrix(1, 1);
    (*H)(2, 2) = poseMatrix(2, 1);
    (*H)(4, 2) = -poseMatrix(0, 0);
    (*H)(5, 2) = -poseMatrix(1, 0);
    (*H)(6, 2) = -poseMatrix(2, 0);

    (*H)(12, 3) = poseMatrix(0, 0);
    (*H)(13, 3) = poseMatrix(1, 0);
    (*H)(14, 3) = poseMatrix(2, 0);

    (*H)(12, 4) = poseMatrix(0, 1);
    (*H)(13, 4) = poseMatrix(1, 1);
    (*H)(14, 4) = poseMatrix(2, 1);

    (*H)(12, 5) = poseMatrix(0, 2);
    (*H)(13, 5) = poseMatrix(1, 2);
    (*H)(14, 5) = poseMatrix(2, 2);
  }
  return poseMatrix;
}

/* ************************************************************************* */
gtsam::Matrix kron(const gtsam::Matrix m1, const gtsam::Matrix m2) {
  gtsam::Matrix m3(m1.rows() * m2.rows(), m1.cols() * m2.cols());

  for (int j = 0; j < m1.cols(); j++) {
    for (int i = 0; i < m1.rows(); i++) {
      m3.block(i * m2.rows(), j * m2.cols(), m2.rows(), m2.cols()) =
          m1(i, j) * m2;
    }
  }
  return m3;
}

/* ************************************************************************* */
gtsam::Matrix TVEC(const int m, const int n) {
  gtsam::Matrix T(m * n, m * n);
  for (int j = 0; j < m * n; j++) {
    for (int i = 0; i < m * n; i++) {
      if ((j + 1) == (1 + (m * ((i + 1) - 1)) -
                      ((m * n - 1) * floor(((i + 1) - 1) / n)))) {
        T(i, j) = 1;
      } else {
        T(i, j) = 0;
      }
    }
  }
  return T;
}

}  // namespace utils
}  // namespace gtsam_quadrics
