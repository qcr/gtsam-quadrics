/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DualConic.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a dual conic
 */

#include <quadricslam/geometry/DualConic.h>
#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
DualConic::DualConic() {
  dC_ = (Matrix33() << 1.0, 0.0, 0.0, 
                        0.0, 1.0, 0.0,
                        0.0, 0.0, -1.0).finished();
}

/* ************************************************************************* */
DualConic::DualConic(const Matrix33& dC) {
  dC_ = dC;
}

/* ************************************************************************* */
Matrix33 DualConic::matrix() const {
  return dC_;
}

/* ************************************************************************* */
// TODO: assert conic is closed (eccentricity)
// assert bounds are real-valued
AlignedBox2 DualConic::bounds() const {
  double xmin = (dC_(0,2) + std::sqrt(dC_(0,2)*dC_(0,2)-dC_(2,2)*dC_(0,0))) / dC_(2,2);
  double xmax = (dC_(0,2) - std::sqrt(dC_(0,2)*dC_(0,2)-dC_(2,2)*dC_(0,0))) / dC_(2,2);
  double ymin = (dC_(1,2) + std::sqrt(dC_(1,2)*dC_(1,2)-dC_(2,2)*dC_(1,1))) / dC_(2,2);
  double ymax = (dC_(1,2) - std::sqrt(dC_(1,2)*dC_(1,2)-dC_(2,2)*dC_(1,1))) / dC_(2,2);
  return AlignedBox2(xmin, ymin, xmax, ymax);
}

/* ************************************************************************* */
void DualConic::print(const std::string& s) const {
  cout << s << " : \n" << dC_ << endl;
}

/* ************************************************************************* */
bool DualConic::equals(const DualConic& other, double tol) const {
  return dC_.isApprox(other.dC_, tol);
}
    
} // namespace gtsam
