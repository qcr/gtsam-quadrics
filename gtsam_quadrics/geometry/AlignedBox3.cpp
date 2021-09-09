/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file AlignedBox2.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief an axis aligned 3D bounding box
 */

#include <gtsam_quadrics/geometry/AlignedBox3.h>

using namespace std;

namespace gtsam_quadrics {

/* ************************************************************************* */
AlignedBox3::AlignedBox3(const double& xmin, const double& xmax,
                         const double& ymin, const double& ymax,
                         const double& zmin, const double& zmax) {
  xxyyzz_ = (gtsam::Vector6() << xmin, xmax, ymin, ymax, zmin, zmax).finished();
}

/* ************************************************************************* */
gtsam::Vector3 AlignedBox3::dimensions() const {
  return (gtsam::Vector3() << xmax() - xmin(), ymax() - ymin(), zmax() - zmin())
      .finished();
}

/* ************************************************************************* */
gtsam::Vector3 AlignedBox3::centroid() const {
  return (gtsam::Vector3() << xmin() + xmax(), ymin() + ymax(), zmin() + zmax())
             .finished() /
         2.0;
}

/* ************************************************************************* */
double AlignedBox3::iou(const AlignedBox3& other) const {
  AlignedBox3 inter_box(std::max(this->xmin(), other.xmin()),
                        std::min(this->xmax(), other.xmax()),
                        std::max(this->ymin(), other.ymin()),
                        std::min(this->ymax(), other.ymax()),
                        std::max(this->zmin(), other.zmin()),
                        std::min(this->zmax(), other.zmax()));

  if ((inter_box.xmax() < inter_box.xmin()) ||
      (inter_box.ymax() < inter_box.ymin()) ||
      (inter_box.zmax() < inter_box.zmin())) {
    return 0.0;
  }

  double inter_volume = inter_box.volume();
  double iou = inter_volume / (this->volume() + other.volume() - inter_volume);

  assert(iou >= 0.0);
  assert(iou <= 1.0);
  return iou;
}

/* ************************************************************************* */
void AlignedBox3::print(const std::string& s) const {
  cout << s << this->vector().transpose() << endl;
}

/* ************************************************************************* */
bool AlignedBox3::equals(const AlignedBox3& other, double tol) const {
  return xxyyzz_.isApprox(other.xxyyzz_, tol);
}

}  // namespace gtsam_quadrics
