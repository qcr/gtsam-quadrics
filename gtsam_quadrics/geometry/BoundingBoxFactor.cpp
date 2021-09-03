/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BoundingBoxFactor.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/base/QuadricProjectionException.h>
#include <gtsam_quadrics/geometry/BoundingBoxFactor.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>

#define NUMERICAL_DERIVATIVE false

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Vector BoundingBoxFactor::evaluateError(const Pose3& pose,
                                        const ConstrainedDualQuadric& quadric,
                                        boost::optional<Matrix&> H1,
                                        boost::optional<Matrix&> H2) const {
  try {
    // check pose-quadric pair
    if (quadric.isBehind(pose)) {
      throw QuadricProjectionException("Quadric is behind camera");
    }
    if (quadric.contains(pose)) {
      throw QuadricProjectionException("Camera is inside quadric");
    }

    // project quadric taking into account partial derivatives
    Eigen::Matrix<double, 9, 6> dC_dx;
    Eigen::Matrix<double, 9, 9> dC_dq;
    DualConic dualConic;
    if (!NUMERICAL_DERIVATIVE) {
      dualConic = QuadricCamera::project(quadric, pose, calibration_,
                                         H2 ? &dC_dq : 0, H1 ? &dC_dx : 0);
    } else {
      dualConic = QuadricCamera::project(quadric, pose, calibration_);
    }

    // check dual conic is valid for error function
    if (!dualConic.isEllipse()) {
      throw QuadricProjectionException("Projected Conic is non-ellipse");
    }

    // calculate conic bounds with derivatives
    bool computeJacobians = bool(H1 || H2) && !NUMERICAL_DERIVATIVE;
    Eigen::Matrix<double, 4, 9> db_dC;
    AlignedBox2 predictedBounds;
    if (measurementModel_ == STANDARD) {
      predictedBounds = dualConic.bounds(computeJacobians ? &db_dC : 0);
    } else if (measurementModel_ == TRUNCATED) {
      try {
        predictedBounds =
            dualConic.smartBounds(calibration_, computeJacobians ? &db_dC : 0);
      } catch (std::runtime_error& e) {
        throw QuadricProjectionException("smartbounds failed");
      }
    }

    // evaluate error
    Vector4 error = predictedBounds.vector() - measured_.vector();

    if (NUMERICAL_DERIVATIVE) {
      std::function<Vector(const Pose3&, const ConstrainedDualQuadric&)> funPtr(
          boost::bind(&BoundingBoxFactor::evaluateError, this, _1, _2,
                      boost::none, boost::none));
      if (H1) {
        Eigen::Matrix<double, 4, 6> db_dx_ =
            numericalDerivative21(funPtr, pose, quadric, 1e-6);
        *H1 = db_dx_;
      }
      if (H2) {
        Eigen::Matrix<double, 4, 9> db_dq_ =
            numericalDerivative22(funPtr, pose, quadric, 1e-6);
        *H2 = db_dq_;
      }
    } else {
      // calculate derivative of error wrt pose
      if (H1) {
        // combine partial derivatives
        *H1 = db_dC * dC_dx;
      }

      // calculate derivative of error wrt quadric
      if (H2) {
        // combine partial derivatives
        *H2 = db_dC * dC_dq;
      }
    }

    return error;

    // check for nans
    if (error.array().isInf().any() || error.array().isNaN().any() ||
        (H1 && (H1->array().isInf().any() || H1->array().isNaN().any())) ||
        (H2 && (H2->array().isInf().any() || H2->array().isNaN().any()))) {
      throw std::runtime_error("nan/inf error in bbf");
    }

    // handle projection failures
  } catch (QuadricProjectionException& e) {
    // std::cout << "  Landmark " << symbolIndex(this->objectKey()) << "
    // received: " << e.what() << std::endl;

    // if error cannot be calculated
    // set error vector and jacobians to zero
    Vector4 error = Vector4::Ones() * 1000.0;
    if (H1) {
      *H1 = Matrix::Zero(4, 6);
    }
    if (H2) {
      *H2 = Matrix::Zero(4, 9);
    }

    return error;
  }
}

/* ************************************************************************* */
Matrix BoundingBoxFactor::evaluateH1(
    const Pose3& pose, const ConstrainedDualQuadric& quadric) const {
  Matrix H1;
  this->evaluateError(pose, quadric, H1, boost::none);
  return H1;
}

/* ************************************************************************* */
Matrix BoundingBoxFactor::evaluateH2(
    const Pose3& pose, const ConstrainedDualQuadric& quadric) const {
  Matrix H2;
  this->evaluateError(pose, quadric, boost::none, H2);
  return H2;
}

/* ************************************************************************* */
Matrix BoundingBoxFactor::evaluateH1(const Values& x) const {
  const Pose3 pose = x.at<Pose3>(this->poseKey());
  const ConstrainedDualQuadric quadric =
      x.at<ConstrainedDualQuadric>(this->objectKey());
  return this->evaluateH1(pose, quadric);
}

/* ************************************************************************* */
Matrix BoundingBoxFactor::evaluateH2(const Values& x) const {
  const Pose3 pose = x.at<Pose3>(this->poseKey());
  const ConstrainedDualQuadric quadric =
      x.at<ConstrainedDualQuadric>(this->objectKey());
  return this->evaluateH2(pose, quadric);
}

/* ************************************************************************* */
void BoundingBoxFactor::print(const std::string& s,
                              const KeyFormatter& keyFormatter) const {
  cout << s << "BoundingBoxFactor(" << keyFormatter(key1()) << ","
       << keyFormatter(key2()) << ")" << endl;
  measured_.print("    Measured: ");
  cout << "    NoiseModel: ";
  noiseModel()->print();
  cout << endl;
}

/* ************************************************************************* */
bool BoundingBoxFactor::equals(const BoundingBoxFactor& other,
                               double tol) const {
  bool equal = measured_.equals(other.measured_, tol) &&
               calibration_->equals(*other.calibration_, tol) &&
               noiseModel()->equals(*other.noiseModel(), tol) &&
               key1() == other.key1() && key2() == other.key2();
  return equal;
}

}  // namespace gtsam
