/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
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

#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/base/QuadricProjectionException.h>
#include <gtsam_quadrics/base/Utilities.h>
#include <gtsam_quadrics/geometry/DualConic.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>

#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

namespace gtsam_quadrics {

/* ************************************************************************* */
DualConic::DualConic() {
  dC_ = (gtsam::Matrix33() << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0)
            .finished();
}

/* ************************************************************************* */
DualConic::DualConic(const gtsam::Pose2& pose, const gtsam::Vector2& radii) {
  gtsam::Matrix33 Z = pose.matrix();
  gtsam::Matrix33 Cc = (gtsam::Vector3() << (radii).array().pow(2), -1.0)
                           .finished()
                           .asDiagonal();
  gtsam::Matrix33 dC = Z * Cc * Z.transpose();
  dC_ = dC;
}

/* ************************************************************************* */
DualConic DualConic::normalize(void) const {
  return DualConic(dC_ / dC_(2, 2));
}

/* ************************************************************************* */
/// TODO: assert conic is closed (eccentricity)
/// assert bounds are real-valued
/// normalize conic
AlignedBox2 DualConic::bounds(gtsam::OptionalJacobian<4, 9> H) const {
  double xmin =
      (dC_(0, 2) + sqrt(dC_(0, 2) * dC_(0, 2) - dC_(2, 2) * dC_(0, 0))) /
      dC_(2, 2);
  double xmax =
      (dC_(0, 2) - sqrt(dC_(0, 2) * dC_(0, 2) - dC_(2, 2) * dC_(0, 0))) /
      dC_(2, 2);
  double ymin =
      (dC_(1, 2) + sqrt(dC_(1, 2) * dC_(1, 2) - dC_(2, 2) * dC_(1, 1))) /
      dC_(2, 2);
  double ymax =
      (dC_(1, 2) - sqrt(dC_(1, 2) * dC_(1, 2) - dC_(2, 2) * dC_(1, 1))) /
      dC_(2, 2);

  if (H) {
    Eigen::Matrix<double, 4, 9> db_dC = gtsam::Matrix::Zero(4, 9);
    double f = sqrt(dC_(0, 2) * dC_(0, 2) - dC_(0, 0) * dC_(2, 2));
    double g = sqrt(dC_(1, 2) * dC_(1, 2) - dC_(1, 1) * dC_(2, 2));
    db_dC(0, 0) = 1.0 / f * (-1.0 / 2.0);
    db_dC(0, 6) = (dC_(0, 2) * 1.0 / f + 1.0) / dC_(2, 2);
    db_dC(0, 8) = -1.0 / (dC_(2, 2) * dC_(2, 2)) * (dC_(0, 2) + f) -
                  (dC_(0, 0) * 1.0 / f * (1.0 / 2.0)) / dC_(2, 2);
    db_dC(1, 4) = 1.0 / g * (-1.0 / 2.0);
    db_dC(1, 7) = (dC_(1, 2) * 1.0 / g + 1.0) / dC_(2, 2);
    db_dC(1, 8) = -1.0 / (dC_(2, 2) * dC_(2, 2)) * (dC_(1, 2) + g) -
                  (dC_(1, 1) * 1.0 / g * (1.0 / 2.0)) / dC_(2, 2);
    db_dC(2, 0) = 1.0 / f * (1.0 / 2.0);
    db_dC(2, 6) = -(dC_(0, 2) * 1.0 / f - 1.0) / dC_(2, 2);
    db_dC(2, 8) = -1.0 / (dC_(2, 2) * dC_(2, 2)) * (dC_(0, 2) - f) +
                  (dC_(0, 0) * 1.0 / f * (1.0 / 2.0)) / dC_(2, 2);
    db_dC(3, 4) = 1.0 / g * (1.0 / 2.0);
    db_dC(3, 7) = -(dC_(1, 2) * 1.0 / g - 1.0) / dC_(2, 2);
    db_dC(3, 8) = -1.0 / (dC_(2, 2) * dC_(2, 2)) * (dC_(1, 2) - g) +
                  (dC_(1, 1) * 1.0 / g * (1.0 / 2.0)) / dC_(2, 2);
    *H = db_dC;
  }

  return AlignedBox2(xmin, ymin, xmax, ymax);
}

/* ************************************************************************* */
AlignedBox2 DualConic::smartBounds(
    const boost::shared_ptr<gtsam::Cal3_S2>& calibration,
    gtsam::OptionalJacobian<4, 9> H) const {
  // calculate image dimensions from calibration
  // double imageWidth = calibration->px() * 2.0;
  // double imageHeight = calibration->py() * 2.0;
  double imageWidth = 640.0;
  double imageHeight = 480.0;
  AlignedBox2 imageBounds(0.0, 0.0, imageWidth, imageHeight);

  // if quadric is fully visible, use the faster simple bounds
  Eigen::Matrix<double, 4, 9> simpleJacobian;
  AlignedBox2 simpleBounds = this->bounds(H ? &simpleJacobian : 0);
  if (imageBounds.contains(simpleBounds)) {
    if (H) {
      *H = simpleJacobian;
    }
    return simpleBounds;
  }

  // ensure quadric is at least partially visible
  // NOTE: this will not work because bounds can be inside whilst conic is
  // completely outside imagebounds if (!imageBounds.contains(simpleBounds) &&
  // !imageBounds.intersects(simpleBounds)) {
  //   throw QuadricProjectionException("SimpleBounds outside ImageBounds,
  //   implies quadric not visible");
  // }

  // cast dual conic to long double before inverting
  /// NOTE: this helps keep polynomials from having -ve desc
  Eigen::Matrix<long double, 3, 3> dualConic = dC_.cast<long double>();
  dualConic = dualConic / dualConic(2, 2);

  // calculate point conic
  Eigen::Matrix<long double, 3, 3> C = dualConic.inverse();

  // normalize conic so polynomials behave
  C = C / C(2, 2);

  std::vector<gtsam::Point2> points;

  try {
    // solve intersection of dC/dx and conic C (solving y values first)
    gtsam::Vector2 ys = utils::solvePolynomial(
        C(1, 1) - pow(C(1, 0) * 2, 2) / (4 * C(0, 0)),
        C(2, 1) * 2 - (C(1, 0) * 2 * C(2, 0) * 2) / (2 * C(0, 0)),
        C(2, 2) - pow(C(2, 0) * 2, 2) / (4 * C(0, 0)));
    gtsam::Point2 p1((-C(1, 0) * 2 * ys[0] - C(2, 0) * 2) / (2 * C(0, 0)),
                     ys[0]);
    gtsam::Point2 p3((-C(1, 0) * 2 * ys[1] - C(2, 0) * 2) / (2 * C(0, 0)),
                     ys[1]);

    // solve intersection of dC/dx and conic C (solving x values first)
    // Vector2 xs = gtsam::utils::solvePolynomial(
    //   4*pow(C(0,0),2)*C(1,1)/pow(C(1,0)*2,2) - C(0,0),
    //   4*C(0,0)*C(1,1)*C(2,0)*2/pow(C(1,0)*2,2) - 2*C(0,0)*C(2,1)*2/C(1,0)*2,
    //   C(1,1)*pow(C(2,0)*2,2)/pow(C(1,0)*2,2) - C(2,0)*2*C(2,1)*2/C(1,0)*2 +
    //   C(2,2)
    // );

    // solve intersection of dC/dy and conic C (solving y values first)
    // Vector2 ys = gtsam::utils::solvePolynomial(
    //   4*C(0,0)*pow(C(1,1),2)/pow(C(1,0)*2,2) - C(1,1),
    //   4*C(0,0)*C(1,1)*C(2,1)*2/pow(C(1,0)*2,2) -
    //   (2*C(1,1)*C(2,0)*2/C(1,0)*2), C(0,0)*pow(C(2,1)*2,2)/pow(C(1,0)*2,2) -
    //   (C(2,0)*2*C(2,1)*2/C(1,0)*2) + C(2,2)
    // );

    // solve intersection of dC/dy and conic C (solving x values first)
    gtsam::Vector2 xs = utils::solvePolynomial(
        C(0, 0) - pow(C(1, 0) * 2, 2) / (4 * C(1, 1)),
        C(2, 0) * 2 - (C(1, 0) * 2 * C(2, 1) * 2) / (2 * C(1, 1)),
        C(2, 2) - pow(C(2, 1) * 2, 2) / (4 * C(1, 1)));
    gtsam::Point2 p0(xs[0],
                     (-C(1, 0) * 2 * xs[0] - C(2, 1) * 2) / (2 * C(1, 1)));
    gtsam::Point2 p2(xs[1],
                     (-C(1, 0) * 2 * xs[1] - C(2, 1) * 2) / (2 * C(1, 1)));

    // append extrema to set of points
    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
  } catch (std::runtime_error& e) {
    throw e;
  }

  // intersection of conic and line at X = 0
  try {
    gtsam::Vector2 ys = utils::getConicPointsAtX(C, 0.0);
    points.push_back(gtsam::Point2(0.0, ys[0]));
    points.push_back(gtsam::Point2(0.0, ys[1]));
  } catch (std::runtime_error& e) {
  }

  // intersection of conic and line at X = width
  try {
    gtsam::Vector2 ys = utils::getConicPointsAtX(C, imageWidth);
    points.push_back(gtsam::Point2(imageWidth, ys[0]));
    points.push_back(gtsam::Point2(imageWidth, ys[1]));
  } catch (std::runtime_error& e) {
  }

  // intersection of conic and line at Y = 0
  try {
    gtsam::Vector2 xs = utils::getConicPointsAtY(C, 0.0);
    points.push_back(gtsam::Point2(xs[0], 0.0));
    points.push_back(gtsam::Point2(xs[1], 0.0));
  } catch (std::runtime_error& e) {
  }

  // intersection of conic and line at Y = height
  try {
    gtsam::Vector2 xs = utils::getConicPointsAtY(C, imageHeight);
    points.push_back(gtsam::Point2(xs[0], imageHeight));
    points.push_back(gtsam::Point2(xs[1], imageHeight));
  } catch (std::runtime_error& e) {
  }

  // push back any captured image boundaries
  gtsam::Point2 i1(0.0, 0.0);
  gtsam::Point2 i2(0.0, imageHeight);
  gtsam::Point2 i3(imageWidth, 0.0);
  gtsam::Point2 i4(imageWidth, imageHeight);
  if (this->contains(i1)) {
    points.push_back(i1);
  }
  if (this->contains(i2)) {
    points.push_back(i2);
  }
  if (this->contains(i3)) {
    points.push_back(i3);
  }
  if (this->contains(i4)) {
    points.push_back(i4);
  }

  // only accept non-imaginary points within image boundaries
  /// NOTE: it's important that contains includes points on the boundary
  /// ^ such that the fov intersect points count as valid
  std::vector<gtsam::Point2> validPoints;
  for (auto point : points) {
    if (imageBounds.contains(point)) {
      validPoints.push_back(point);
    }
  }

  if (validPoints.size() < 1) {
    throw std::runtime_error(
        "no valid conic points inside image dimensions, implies quadric not "
        "visible");
    // simpleBounds.print("Failed SimpleBounds:");
    // throw QuadricProjectionException("No valid conic points inside image
    // dimensions, implies quadric not visible");
  }
  auto minMaxX = std::minmax_element(
      validPoints.begin(), validPoints.end(),
      [](const gtsam::Point2& lhs, const gtsam::Point2& rhs) {
        return lhs.x() < rhs.x();
      });
  auto minMaxY = std::minmax_element(
      validPoints.begin(), validPoints.end(),
      [](const gtsam::Point2& lhs, const gtsam::Point2& rhs) {
        return lhs.y() < rhs.y();
      });

  // take the max/min of remaining points
  AlignedBox2 smartBounds(minMaxX.first->x(), minMaxY.first->y(),
                          minMaxX.second->x(), minMaxY.second->y());

  // calculate jacobians
  if (H) {
    // we want to derive wrt vector output and matrix input
    auto bounds_funptr =
        [&](const gtsam::Matrix33& conic_matrix) -> gtsam::Vector {
      return DualConic(conic_matrix)
          .smartBounds(calibration, boost::none)
          .vector();
    };

    // cast to boost::function for numericalDerivative
    auto boost_funptr(
        static_cast<std::function<gtsam::Vector(const gtsam::Matrix33&)>>(
            bounds_funptr));

    // calculate derivative of conic_matrix wrt quadric vector
    Eigen::Matrix<double, 4, 9> db_dC =
        gtsam::numericalDerivative11(boost_funptr, this->matrix(), 1e-6);

    // set jacobian to numerical derivative
    *H = db_dC;
  }
  return smartBounds;
}

/* ************************************************************************* */
bool DualConic::isDegenerate(void) const {
  gtsam::Matrix33 C = dC_.inverse();
  if (C.determinant() == 0.0 || C.array().isInf().any() ||
      C.array().isNaN().any()) {
    return true;
  }
  return false;
}

/* ************************************************************************* */
bool DualConic::isEllipse(void) const {
  gtsam::Matrix33 C = dC_.inverse();
  C = C / C(2, 2);
  bool isDegenerate = C.determinant() == 0.0;
  if (!isDegenerate) {
    gtsam::Matrix22 A33 = C.block(0, 0, 2, 2);
    return (A33.determinant() > 0);
  }
  return false;
}

/* ************************************************************************* */
bool DualConic::contains(const gtsam::Point2& p) const {
  gtsam::Vector3 point = (gtsam::Vector3() << p, 1.0).finished();
  double pointError = point.transpose() * this->matrix().inverse() * point;

  // apply a threshold due to noisy matrix inversion
  double thresh = 1e-10;
  if (pointError <= thresh) {
    return true;
  }
  return false;
}

/* ************************************************************************* */
void DualConic::print(const string& s) const {
  cout << s << " : \n" << dC_ << endl;
}

/* ************************************************************************* */
bool DualConic::equals(const DualConic& other, double tol) const {
  return this->normalize().matrix().isApprox(other.normalize().matrix(), tol);
}

}  // namespace gtsam_quadrics
