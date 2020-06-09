/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
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

#include <quadricslam/geometry/BoundingBoxFactor.h>
#include <quadricslam/geometry/QuadricCamera.h>
#include <quadricslam/base/QuadricProjectionException.h>

#include <gtsam/base/numericalDerivative.h>

#define NUMERICAL_DERIVATIVE false

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Vector BoundingBoxFactor::evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric,
  boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const {

  try {

    // check pose-quadric pair
    if (quadric.isBehind(pose)) {
      throw QuadricProjectionException("Quadric is behind camera");
    } if (quadric.contains(pose)) {
      throw QuadricProjectionException("Camera is inside quadric");
    }

    // project quadric taking into account partial derivatives 
    Eigen::Matrix<double, 9,6> dC_dx; Eigen::Matrix<double, 9,9> dC_dq;
    DualConic dualConic;
    if (!NUMERICAL_DERIVATIVE) {
      dualConic = QuadricCamera::project(quadric, pose, calibration_, H2?&dC_dq:0, H1?&dC_dx:0);
    } else {
      dualConic = QuadricCamera::project(quadric, pose, calibration_);
    }

    // check dual conic is valid for error function
    if (!dualConic.isEllipse()) {
      throw QuadricProjectionException("Projected Conic is non-ellipse");
    }

    // calculate conic bounds with derivatives
    Eigen::Matrix<double, 4,9> db_dC;
    AlignedBox2 predictedBounds; 
    if (!NUMERICAL_DERIVATIVE) {
      predictedBounds = dualConic.bounds(H1||H2?&db_dC:0);
    } else {
      predictedBounds = dualConic.bounds();
    }

    // evaluate error 
    Vector4 error = predictedBounds.vector() - measured_.vector();

    if (NUMERICAL_DERIVATIVE) {
      boost::function<Vector(const Pose3&, const ConstrainedDualQuadric&)> funPtr(boost::bind(&BoundingBoxFactor::evaluateError, this, _1, _2, boost::none, boost::none));
      if (H1) {
        Eigen::Matrix<double, 4,6> db_dx_ = numericalDerivative21(funPtr, pose, quadric, 1e-6);
        *H1 = db_dx_;
      } if (H2) {
        Eigen::Matrix<double, 4,9> db_dq_ = numericalDerivative22(funPtr, pose, quadric, 1e-6);
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


  // handle projection failures
  } catch(QuadricProjectionException& e) {
    
    // if error cannot be calculated
    // set error vector and jacobians to zero
    Vector4 error = Vector4::Zero();
    if (H1) {*H1 = Matrix::Zero(4,6);}
    if (H2) {*H2 = Matrix::Zero(4,9);}
    return error;

  }
}

/* ************************************************************************* */
void BoundingBoxFactor::addToGraph(NonlinearFactorGraph& graph) { 
  graph.add(*this);
}

/* ************************************************************************* */
BoundingBoxFactor BoundingBoxFactor::getFromGraph(const NonlinearFactorGraph& graph, size_t idx) { 
  return *boost::dynamic_pointer_cast<BoundingBoxFactor>(graph.at(idx)).get();
}

/* ************************************************************************* */
void BoundingBoxFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "BoundingBoxFactor(" << keyFormatter(key1()) << "," << keyFormatter(key2()) << ")" << endl;
  measured_.print("    Measured: ");
  cout << "    NoiseModel: "; noiseModel()->print(); cout << endl;
}

/* ************************************************************************* */
bool BoundingBoxFactor::equals(const BoundingBoxFactor& other, double tol) const {
  bool equal = measured_.equals(other.measured_, tol)
    && calibration_->equals(*other.calibration_, tol)
    && noiseModel()->equals(*other.noiseModel(), tol)
    && key1() == other.key1() && key2() == other.key2();
  return equal;
}

} // namespace gtsam
