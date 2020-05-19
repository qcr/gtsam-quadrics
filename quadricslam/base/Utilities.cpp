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
/// NOTE: for single value results disc usually, due to matrix inversion and 
/// other inaccuracies, +- 1e-20. For dual value real/imag results
/// disc is around +- 1e-5. Therefore, if disc 1e-15 or less we round to zero. 
/// TODO: see if we can just improve dual_conic.inverse() and bounds() accuracy
Vector2 solvePolynomial(const double& a, const double& b, const double& c) {
  // calculate polynomial discrimenant 
  double disc = b*b - 4.0*a*c;
  
  // round disc to account for inaccuracies
  if (fabs(disc) < 1e-10) {disc = 0.0;}

  // throw exception if imaginary results
  if (disc < 0.0) {
    throw std::runtime_error("complex values");
  }

  // calculate and return roots
  double root1 = (-b + std::sqrt(disc)) / (2.0*a);
  double root2 = (-b - std::sqrt(disc)) / (2.0*a);
  return Vector2(root1, root2);
}

/* ************************************************************************* */
Vector2 getConicPointsAtX(const Matrix3& pointConic, const double& x) {
  const Matrix3& C = pointConic;
  return gtsam::utils::solvePolynomial(
    C(1,1),
    2*C(0,1)*x + 2*C(1,2),
    C(0,0)*x*x + 2*C(0,2)*x + C(2,2)
  );
}

/* ************************************************************************* */
Vector2 getConicPointsAtY(const Matrix3& pointConic, const double& y) {
  const Matrix3& C = pointConic;
  return gtsam::utils::solvePolynomial(
    C(0,0),
    2*C(0,1)*y + 2*C(0,2),
    C(1,1)*y*y + 2*C(1,2)*y + C(2,2)
  );
}

/* ************************************************************************* */
Pose3 interpolate(const Pose3& p1, const Pose3& p2, const double& percent) {
  return gtsam::interpolate<Pose3>(p1, p2, percent);
}

} // namespace utils
} // namespace gtsam
