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

using namespace std;

namespace gtsam {

/* ************************************************************************* */
std::vector<Vector3> AlignedBox2::lines() const {
  std::vector<Vector3> mLines; 
  mLines.push_back(Vector3(1, 0, -tlbr_[0]));
  mLines.push_back(Vector3(0, 1, -tlbr_[1]));
  mLines.push_back(Vector3(1, 0, -tlbr_[2]));
  mLines.push_back(Vector3(0, 1, -tlbr_[3]));
  return mLines;
}

/* ************************************************************************* */
bool AlignedBox2::contains(const Point2& point) const {
  if (point.x() >= xmin() && point.x() <= xmax() 
    && point.y() >= ymin() && point.y() <= ymax()) {
    return true;
  }
  return false;
}

/* ************************************************************************* */
bool AlignedBox2::contains(const AlignedBox2& other) const {
  return (other.xmin() >= this->xmin()
        && other.xmax() <= this->xmax()
        && other.ymin() >= this->ymin()
        && other.ymax() <= this->ymax());
}

/* ************************************************************************* */
bool AlignedBox2::intersects(const AlignedBox2& other) const {
  if (this->contains(other) || other.contains(*this)) { return false;}
  return !(this->xmin() > other.xmax() 
        || this->xmax() < other.xmin()
        || this->ymin() > other.ymax()
        || this->ymax() < other.ymin()); 
}

/* ************************************************************************* */
void AlignedBox2::print(const std::string& s) const {
  cout << s << this->vector().transpose() << endl;  
}

/* ************************************************************************* */
bool AlignedBox2::equals(const AlignedBox2& other, double tol) const {
  return tlbr_.isApprox(other.tlbr_, tol);
}

} // namespace gtsam
