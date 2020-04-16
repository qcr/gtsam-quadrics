/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file AlignedBox2.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief an axis aligned 2D bounding box
 */

#include <quadricslam/geometry/AlignedBox2.h>
#include <quadricslam/base/NotImplementedException.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
AlignedBox2::AlignedBox2(const double& xmin, const double& ymin, const double& xmax, const double& ymax) {
  tlbr_ = (Vector4() << xmin, ymin, xmax, ymax).finished();
}

/* ************************************************************************* */
// TODO: ensure correct dim order, box has width
AlignedBox2::AlignedBox2(Vector4 tlbr) {
  tlbr_ = tlbr;
}

/* ************************************************************************* */
Vector4 AlignedBox2::vector() const {
  return tlbr_;
}

/* ************************************************************************* */
void AlignedBox2::print(const std::string& s) const {
  cout << s << " : " << this->vector().transpose() << endl;  
}

/* ************************************************************************* */
bool AlignedBox2::equals(const AlignedBox2& other, double tol) const {
  return tlbr_.isApprox(other.tlbr_, tol);
}

} // namespace gtsam
