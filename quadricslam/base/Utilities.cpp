/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
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

#include <quadricslam/base/Utilities.h>

#include <iostream>

using namespace std;

namespace gtsam {
namespace utils {

/* ************************************************************************* */
Pose3 interpolate(const Pose3& p1, const Pose3& p2, const double& percent) {
  return gtsam::interpolate<Pose3>(p1, p2, percent);
}

/* ************************************************************************* */
Matrix44 matrix(const Pose3& pose, OptionalJacobian<16,6> H) {
  Matrix44 poseMatrix = pose.matrix();

  if (H) {

    H->setZero();
    (*H)(4,0) = poseMatrix(0,2);
    (*H)(5,0) = poseMatrix(1,2);
    (*H)(6,0) = poseMatrix(2,2);
    (*H)(8,0) = -poseMatrix(0,1);
    (*H)(9,0) = -poseMatrix(1,1);
    (*H)(10,0) = -poseMatrix(2,1);

    (*H)(0,1) = -poseMatrix(0,2);
    (*H)(1,1) = -poseMatrix(1,2);
    (*H)(2,1) = -poseMatrix(2,2);
    (*H)(8,1) = poseMatrix(0,0);
    (*H)(9,1) = poseMatrix(1,0);
    (*H)(10,1) = poseMatrix(2,0);

    (*H)(0,2) = poseMatrix(0,1);
    (*H)(1,2) = poseMatrix(1,1);
    (*H)(2,2) = poseMatrix(2,1);
    (*H)(4,2) = -poseMatrix(0,0);
    (*H)(5,2) = -poseMatrix(1,0);
    (*H)(6,2) = -poseMatrix(2,0);

    (*H)(12,3) = poseMatrix(0,0);
    (*H)(13,3) = poseMatrix(1,0);
    (*H)(14,3) = poseMatrix(2,0);

    (*H)(12,4) = poseMatrix(0,1);
    (*H)(13,4) = poseMatrix(1,1);
    (*H)(14,4) = poseMatrix(2,1);

    (*H)(12,5) = poseMatrix(0,2);
    (*H)(13,5) = poseMatrix(1,2);
    (*H)(14,5) = poseMatrix(2,2);
  }
  return poseMatrix;
}

/* ************************************************************************* */
Matrix kron(const Matrix m1, const Matrix m2) {
  Matrix m3(m1.rows()*m2.rows(), m1.cols()*m2.cols());

  for (int j = 0; j < m1.cols(); j++) {
    for (int i = 0; i < m1.rows(); i++) {
      m3.block(i*m2.rows(), j*m2.cols(), m2.rows(), m2.cols()) =  m1(i,j)*m2;
    }
  }
  return m3;
}


/* ************************************************************************* */
Matrix TVEC(const int m, const int n) {
  Matrix T(m*n, m*n);
  for (int j = 0; j < m*n; j++) {
    for (int i = 0; i < m*n; i++) {
      if ( (j+1) == ( 1 + (m*((i+1)-1)) - ((m*n-1)*floor(((i+1)-1)/n)))) {
        T(i,j) = 1;
      } else {
        T(i,j) = 0;
      }
    }
  }
  return T;
}

} // namespace utils
} // namespace gtsam
