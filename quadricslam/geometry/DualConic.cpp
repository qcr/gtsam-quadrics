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
#include <quadricslam/geometry/QuadricCamera.h>
#include <quadricslam/base/Utilities.h>
#include <quadricslam/base/QuadricProjectionException.h>

#include <gtsam/base/numericalDerivative.h>

#include <iostream>
#include <iomanip>
#include <cmath>

using namespace std;

#define SIGN2STR(n) (n >= 0 ? " + " : " - ")

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
DualConic::DualConic(const Pose2& pose, const Vector2& radii) {
  Matrix33 Z = pose.matrix();
  Matrix33 Cc = (Vector3() << (radii).array().pow(2), -1.0).finished().asDiagonal();
  Matrix33 dC = Z * Cc * Z.transpose();
  dC_ = dC;
}

/* ************************************************************************* */
Matrix33 DualConic::matrix(void) const {
  return dC_;
}

/* ************************************************************************* */
DualConic DualConic::normalize(void) const {
  return DualConic(dC_/dC_(2,2));
}

/* ************************************************************************* */
// TODO: assert conic is closed (eccentricity)
// assert bounds are real-valued
// normalize conic
AlignedBox2 DualConic::bounds(OptionalJacobian<4,9> H) const {
  double xmin = (dC_(0,2) + sqrt(dC_(0,2)*dC_(0,2)-dC_(2,2)*dC_(0,0))) / dC_(2,2);
  double xmax = (dC_(0,2) - sqrt(dC_(0,2)*dC_(0,2)-dC_(2,2)*dC_(0,0))) / dC_(2,2);
  double ymin = (dC_(1,2) + sqrt(dC_(1,2)*dC_(1,2)-dC_(2,2)*dC_(1,1))) / dC_(2,2);
  double ymax = (dC_(1,2) - sqrt(dC_(1,2)*dC_(1,2)-dC_(2,2)*dC_(1,1))) / dC_(2,2);

  if (H) {
    Eigen::Matrix<double, 4,9> db_dC = Matrix::Zero(4,9);
    double f = sqrt(dC_(0,2)*dC_(0,2)-dC_(0,0)*dC_(2,2));
    double g = sqrt(dC_(1,2)*dC_(1,2)-dC_(1,1)*dC_(2,2));
    db_dC(0,0) = 1.0/f*(-1.0/2.0);
    db_dC(0,6) = (dC_(0,2)*1.0/f+1.0)/dC_(2,2);
    db_dC(0,8) = -1.0/(dC_(2,2)*dC_(2,2))*(dC_(0,2)+f)-(dC_(0,0)*1.0/f*(1.0/2.0))/dC_(2,2);
    db_dC(1,4) = 1.0/g*(-1.0/2.0);
    db_dC(1,7) = (dC_(1,2)*1.0/g+1.0)/dC_(2,2);
    db_dC(1,8) = -1.0/(dC_(2,2)*dC_(2,2))*(dC_(1,2)+g)-(dC_(1,1)*1.0/g*(1.0/2.0))/dC_(2,2);
    db_dC(2,0) = 1.0/f*(1.0/2.0);
    db_dC(2,6) = -(dC_(0,2)*1.0/f-1.0)/dC_(2,2);
    db_dC(2,8) = -1.0/(dC_(2,2)*dC_(2,2))*(dC_(0,2)-f)+(dC_(0,0)*1.0/f*(1.0/2.0))/dC_(2,2);
    db_dC(3,4) = 1.0/g*(1.0/2.0);
    db_dC(3,7) = -(dC_(1,2)*1.0/g-1.0)/dC_(2,2);
    db_dC(3,8) = -1.0/(dC_(2,2)*dC_(2,2))*(dC_(1,2)-g)+(dC_(1,1)*1.0/g*(1.0/2.0))/dC_(2,2);
    *H = db_dC;
  }

  return AlignedBox2(xmin, ymin, xmax, ymax);
}

/* ************************************************************************* */
/// NOTE: assumes you've checked the conic is elliptical
AlignedBox2 DualConic::smartBounds(const boost::shared_ptr<Cal3_S2>& calibration, OptionalJacobian<4,9> H) const {

  // calculate point conic
  Matrix3 dC = dC_/dC_(2,2);
  Matrix3 C = dC.inverse();

  // normalize conic so polynomials behave 
  C = C/C(2,2);
  
  // get conic extrema points
  /// NOTE: because using bounds to find corrosponding points
  /// and inaccuracies in inversion, we solve the poly
  /// with a tolerance for small negative discrimnants
  AlignedBox2 bounds = this->bounds();
  double p1_x = bounds.xmin();
  double p2_y = bounds.ymin();
  double p3_x = bounds.xmax();
  double p4_y = bounds.ymax();

  AlignedBox2 fov_box(0, 0, 2.0*calibration->px(), 2.0*calibration->py());
  if (!fov_box.contains(bounds)) {
    throw QuadricProjectionException("no valid conic points inside screen dimensions, implies quadric not visible");
  }
  
  // calculate corrosponding points of conic extrema
  /// NOTE: poly should return the same item twice
  double p1_y, p3_y, p2_x, p4_x;
  try {
    p1_y = gtsam::utils::getConicPointsAtX(C, p1_x)[0];
    p3_y = gtsam::utils::getConicPointsAtX(C, p3_x)[0];
    p2_x = gtsam::utils::getConicPointsAtY(C, p2_y)[0];
    p4_x = gtsam::utils::getConicPointsAtY(C, p4_y)[0];
  } catch (std::runtime_error& e) {
    cout << "bounds: " << this->bounds().vector().transpose() << endl;
    cout << "isEllipse: " << this->isEllipse() << endl;
    cout << "isDegenerate: " << this->isDegenerate() << endl;
    throw std::runtime_error("unable to get conic bound points, complex values, implies inaccurate simple bounds");
  }

  // append to set of points
  std::vector<Point2> points; 
  points.push_back(Point2(p1_x, p1_y));
  points.push_back(Point2(p2_x, p2_y));
  points.push_back(Point2(p3_x, p3_y));
  points.push_back(Point2(p4_x, p4_y));

  // calculate image dimensions from calibration
  double image_width = calibration->px() * 2.0;
  double image_height = calibration->py() * 2.0;

  // intersection of conic and line at X = 0 
  try {
    Vector2 ys = gtsam::utils::getConicPointsAtX(C, 0.0);
    points.push_back(Point2(0.0, ys[0]));
    points.push_back(Point2(0.0, ys[1]));
  } catch(std::runtime_error& e) { }

  // intersection of conic and line at X = width
  try {
    Vector2 ys = gtsam::utils::getConicPointsAtX(C, image_width);
    points.push_back(Point2(image_width, ys[0]));
    points.push_back(Point2(image_width, ys[1]));
  } catch(std::runtime_error& e) { }

  // intersection of conic and line at Y = 0 
  try {
    Vector2 xs = gtsam::utils::getConicPointsAtY(C, 0.0);
    points.push_back(Point2(xs[0], 0.0));
    points.push_back(Point2(xs[1], 0.0));
  } catch(std::runtime_error& e) { }

  // intersection of conic and line at Y = height
  try {
    Vector2 xs = gtsam::utils::getConicPointsAtY(C, image_height);
    points.push_back(Point2(xs[0], image_height));
    points.push_back(Point2(xs[1], image_height));
  } catch(std::runtime_error& e) { }

  // only accept non-imaginary points within image boundaries
  /// NOTE: it's important that contains includes points on the boundary
  /// ^ such that the fov intersect points count as valid
  AlignedBox2 imageBounds(0.0, 0.0, image_width, image_height);
  std::vector<Point2> validPoints;
  for (auto point : points) {
    if (imageBounds.contains(point)) {
      validPoints.push_back(point);
    }
  }

  if (validPoints.size() < 1) {
    throw std::runtime_error("no valid conic points inside image dimensions");
  }
  auto minMaxX = std::minmax_element(validPoints.begin(), validPoints.end(), 
    [] (const Point2& lhs, const Point2& rhs) {return lhs.x() < rhs.x();});
  auto minMaxY = std::minmax_element(validPoints.begin(), validPoints.end(), 
    [] (const Point2& lhs, const Point2& rhs) {return lhs.y() < rhs.y();});

  // take the max/min of remaining points
  AlignedBox2 smartBounds(minMaxX.first->x(), minMaxY.first->y(), minMaxX.second->x(), minMaxY.second->y());

  // calculate jacobians
  if (H) {
    // boost::function<Vector(const DualConic&)> funPtr(boost::bind(&DualConic::vectorSmartBounds, _1, calibration, boost::none));
    // Eigen::Matrix<double, 4,9> db_dC = numericalDerivative11(funPtr, *this, 1e-6);
    *H = Matrix::Zero(4,9);
  }
  return smartBounds;
}

/* ************************************************************************* */
/// TODO: float compare, what eps?
/// ^ det should be exactly 0.0 in degenerate case
/// ^ and is often 1e-15 unnormalized and 1e-10 normalized
bool DualConic::isDegenerate(void) const {
  Matrix33 C = dC_.inverse();
  return C.determinant() == 0.0;
}

/* ************************************************************************* */
bool DualConic::isEllipse(void) const {
  Matrix33 C = dC_.inverse();
  C = C/C(2,2);
  bool isDegenerate = C.determinant() == 0.0;
  if (!isDegenerate) {
    Matrix22 A33 = C.block(0,0,2,2);
    return (A33.determinant() > 0);
  }
  return false;
}

/* ************************************************************************* */
string DualConic::polynomial(void) const {
  Matrix33 C = dC_.inverse();
  stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << C(0,0)*1 << "*x^2";
  ss << SIGN2STR(C(0,1)*2) << fabs(C(0,1)*2) << "*x*y";
  ss << SIGN2STR(C(1,1)*1) << fabs(C(1,1)*1) << "*y^2";
  ss << SIGN2STR(C(0,2)*2) << fabs(C(0,2)*2) << "*x";
  ss << SIGN2STR(C(1,2)*2) << fabs(C(1,2)*2) << "*y";
  ss << SIGN2STR(C(2,2)*1) << fabs(C(2,2)*1) << " = 0";
  return ss.str();
}

/* ************************************************************************* */
void DualConic::print(const string& s) const {
  cout << s << " : \n" << dC_ << endl;
}

/* ************************************************************************* */
bool DualConic::equals(const DualConic& other, double tol) const {
  return this->normalize().matrix().isApprox(other.normalize().matrix(), tol);
}
    
} // namespace gtsam
