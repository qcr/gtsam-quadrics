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

#define ISCLOSE(a,b,e) (fabs(a - b) <= e)

namespace gtsam {
namespace utils {


  
/* ************************************************************************* */
Vector2 solvePolynomial(const double& a, const double& b, const double& c) {
  double disc = b*b - 4.0*a*c;
  // if disc ~ 0, return only the real components
  if (ISCLOSE(disc, 0, 1e-9)) {disc = 0.0;}
  if (disc < 0.0) { 
    std::cout << "broken disc: " << disc << std::endl;
    throw std::runtime_error("complex values");
  }
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


}
}