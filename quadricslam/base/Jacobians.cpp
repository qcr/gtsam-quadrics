/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Jacobians.cpp
 * @date Apr 22, 2020
 * @author Lachlan Nicholson
 * @brief some helper functions for calculating jacobians 
 */

#include <quadricslam/base/Jacobians.h>

#include <gtsam/base/numericalDerivative.h>

using namespace std;
namespace gtsam {
namespace internal {

Eigen::VectorXd flatten(Matrix X) {
  return Eigen::Map<Eigen::VectorXd>(X.data(), X.size());
}

Matrix44 matrix(const Pose3& pose, OptionalJacobian<16,6> H) {
  Matrix44 poseMatrix = pose.matrix();

  if (H) {

    boost::function<Matrix44(const Pose3&)> matrix_(boost::bind(&Pose3::matrix, _1));
    *H = numericalDerivative11(matrix_, pose, 1e-6);


    // // compute pose vector for dX:_dx 
    // Vector6 v = Pose3::LocalCoordinates(pose);
    // double x = v(0), y = v(1), z = v(2);
    // double f = x*x+y*y+z*z+4.0;
    // double d = pow(f,2.0);

    // cout << "pose.matrix()\n" << pose.matrix() << endl << endl;
    // cout << "local(pose)\n" << v.transpose() << endl << endl;

    // (*H).block(0,0,16,6) = Matrix::Zero(16,6);
    // (*H)(0,0) = (x*2.0)/f-x*(x*x-y*y-z*z+4.0)*1.0/d*2.0;
    // (*H)(0,1) = (y*-2.0)/f-y*(x*x-y*y-z*z+4.0)*1.0/d*2.0;
    // (*H)(0,2) = (z*-2.0)/f-z*(x*x-y*y-z*z+4.0)*1.0/d*2.0;
    // (*H)(1,0) = (y*2.0)/f-x*(z*2.0+x*y)*1.0/d*4.0;
    // (*H)(1,1) = (x*2.0)/f-y*(z*2.0+x*y)*1.0/d*4.0;
    // (*H)(1,2) = 4.0/f-z*(z*2.0+x*y)*1.0/d*4.0;
    // (*H)(2,0) = (z*2.0)/f+x*(y*2.0-x*z)*1.0/d*4.0;
    // (*H)(2,1) = -4.0/f+y*(y*2.0-x*z)*1.0/d*4.0;
    // (*H)(2,2) = (x*2.0)/f+z*(y*2.0-x*z)*1.0/d*4.0;
    // (*H)(4,0) = (y*2.0)/f+x*(z*2.0-x*y)*1.0/d*4.0;
    // (*H)(4,1) = (x*2.0)/f+y*(z*2.0-x*y)*1.0/d*4.0;
    // (*H)(4,2) = -4.0/f+z*(z*2.0-x*y)*1.0/d*4.0;
    // (*H)(5,0) = (x*-2.0)/f+x*(x*x-y*y+z*z-4.0)*1.0/d*2.0;
    // (*H)(5,1) = (y*2.0)/f+y*(x*x-y*y+z*z-4.0)*1.0/d*2.0;
    // (*H)(5,2) = (z*-2.0)/f+z*(x*x-y*y+z*z-4.0)*1.0/d*2.0;
    // (*H)(6,0) = 4.0/f-x*(x*2.0+y*z)*1.0/d*4.0;
    // (*H)(6,1) = (z*2.0)/f-y*(x*2.0+y*z)*1.0/d*4.0;
    // (*H)(6,2) = (y*2.0)/f-z*(x*2.0+y*z)*1.0/d*4.0;
    // (*H)(8,0) = (z*2.0)/f-x*(y*2.0+x*z)*1.0/d*4.0;
    // (*H)(8,1) = 4.0/f-y*(y*2.0+x*z)*1.0/d*4.0;
    // (*H)(8,2) = (x*2.0)/f-z*(y*2.0+x*z)*1.0/d*4.0;
    // (*H)(9,0) = -4.0/f+x*(x*2.0-y*z)*1.0/d*4.0;
    // (*H)(9,1) = (z*2.0)/f+y*(x*2.0-y*z)*1.0/d*4.0;
    // (*H)(9,2) = (y*2.0)/f+z*(x*2.0-y*z)*1.0/d*4.0;
    // (*H)(10,0) = (x*-2.0)/f+x*(x*x+y*y-z*z-4.0)*1.0/d*2.0;
    // (*H)(10,1) = (y*-2.0)/f+y*(x*x+y*y-z*z-4.0)*1.0/d*2.0;
    // (*H)(10,2) = (z*2.0)/f+z*(x*x+y*y-z*z-4.0)*1.0/d*2.0;
    // (*H)(12,3) = 1.0;
    // (*H)(13,4) = 1.0;
    // (*H)(14,5) = 1.0;
  }
  return poseMatrix;
}

Matrix kron(const Matrix m1, const Matrix m2) {
  Matrix m3(m1.rows()*m2.rows(), m1.cols()*m2.cols());

  for (int j = 0; j < m1.cols(); j++) {
    for (int i = 0; i < m1.rows(); i++) {
      m3.block(i*m2.rows(), j*m2.cols(), m2.rows(), m2.cols()) =  m1(i,j)*m2;
    }
  }
  return m3;
}


// http://www.ee.ic.ac.uk/hp/staff/dmb/matrix/special.html#VecTranspose
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


} // namespace internal
} // namespace gtsam