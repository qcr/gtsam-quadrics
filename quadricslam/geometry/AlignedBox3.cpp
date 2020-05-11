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

#include <quadricslam/geometry/AlignedBox3.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
AlignedBox3::AlignedBox3(const double& xmin, const double& xmax, const double& ymin, const double& ymax, const double& zmin, const double& zmax) {
  xxyyzz_ = (Vector6() << xmin, xmax, ymin, ymax, zmin, zmax).finished();
}

/* ************************************************************************* */
AlignedBox3::AlignedBox3(const Vector6& xxyyzz) {
  xxyyzz_ = Vector6(xxyyzz);
}

/* ************************************************************************* */
Vector6 AlignedBox3::vector() const {
  return xxyyzz_;
}

/* ************************************************************************* */
Vector3 AlignedBox3::dimensions() const {
  return (Vector3() << xmax()-xmin(), ymax()-ymin(), zmax()-zmin()).finished();
}

/* ************************************************************************* */
Vector3 AlignedBox3::centroid() const {
  return (Vector3() << xmin()+xmax(), ymin()+ymax(), zmin()+zmax()).finished()/2.0;
}

/* ************************************************************************* */
void AlignedBox3::print(const std::string& s) const {
  cout << s << this->vector().transpose() << endl;  
}

/* ************************************************************************* */
bool AlignedBox3::equals(const AlignedBox3& other, double tol) const {
  return xxyyzz_.isApprox(other.xxyyzz_, tol);
}

} // namespace gtsam
