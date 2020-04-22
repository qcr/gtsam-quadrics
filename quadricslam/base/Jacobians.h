/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Jacobians.h
 * @date Apr 22, 2020
 * @author Lachlan Nicholson
 * @brief some helper functions for calculating jacobians 
 */

#include <gtsam/base/Matrix.h>


namespace gtsam {
namespace internal {

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
static Matrix TVEC(const int m, const int n) {
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