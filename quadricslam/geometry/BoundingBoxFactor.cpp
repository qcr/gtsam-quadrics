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

#define NUMERICAL_DERIVATIVE true

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

    // if (quadric.notVisible(pose, calibration_)) {
    //   throw QuadricProjectionException("Quadric is fully outside fov w/ smartBounds on");
    // }

    // project quadric taking into account partial derivatives 
    Eigen::Matrix<double, 9,6> dC_dx; Eigen::Matrix<double, 9,9> dC_dq;
    DualConic dualConic = QuadricCamera::project(quadric, pose, calibration_, H1?&dC_dq:0, H2?&dC_dx:0);

    // check dual conic is valid for error function
    if (!dualConic.isEllipse()) {
      throw QuadricProjectionException("Projected Conic is non-ellipse");
    }

    // calculate conic bounds with derivatives
    Eigen::Matrix<double, 4,9> db_dC;
    AlignedBox2 predictedBounds;


    try {
      predictedBounds = dualConic.bounds(H1||H2?&db_dC:0);
      // predictedBounds = dualConic.smartBounds(calibration_, H1||H2?&db_dC:0);

    } catch (QuadricProjectionException& e) {
      throw QuadricProjectionException("Quadric is fully outside fov w/ smartBounds on");
    }

    // evaluate error 
    Vector4 error = predictedBounds.vector() - measured_.vector();

    // ensure error is never invalid
    if (error.array().isInf().any() or error.array().isNaN().any()) {
      cout << "Infinite error inside BBF" << endl;
      cout << "Dual Conic:\n" << dualConic.matrix() << endl;
      cout << "error: " << error.transpose() << endl;
      pose.print("pose:\n");
      quadric.print();
      throw std::runtime_error("Infinite error inside BBF");
    }


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
        if ((*H1).array().isInf().any() or (*H1).array().isNaN().any()) {
          cout << "\nWARNING: (*H1) inf/nan\nH1:\n" << (*H1) << endl << endl;  
          throw std::runtime_error("Infinite inside BBF H1");
        }

        if (TEST_ANALYTICAL) {
          boost::function<Vector(const Pose3&, const ConstrainedDualQuadric&)> funPtr(boost::bind(&BoundingBoxFactor::evaluateError, this, _1, _2, boost::none, boost::none));
          Eigen::Matrix<double, 4,6> db_dx_ = numericalDerivative21(funPtr, pose, quadric, 1e-6);
          if (!db_dx_.isApprox(*H1, 1e-06)) {
            throw std::runtime_error("BoundingBoxFactor db_dx_ numerical != analytical");
          }
        }
      } 
      
      // calculate derivative of error wrt quadric
      if (H2) {

        // combine partial derivatives 
        *H2 = db_dC * dC_dq; 
        if ((*H2).array().isInf().any() or (*H2).array().isNaN().any()) {
          cout << "\nWARNING: (*H2) inf/nan\nH1:\n" << (*H2) << endl << endl;  
          throw std::runtime_error("Infinite inside BBF H2");
        }
        
        if (TEST_ANALYTICAL) {
          boost::function<Vector(const Pose3&,  const ConstrainedDualQuadric&)> funPtr(boost::bind(&BoundingBoxFactor::evaluateError, this, _1, _2, boost::none, boost::none));
          Eigen::Matrix<double, 4,9> db_dq_ = numericalDerivative22(funPtr, pose, quadric, 1e-6);
          if (!db_dq_.isApprox(*H2, 1e-06)) {
            throw std::runtime_error("BoundingBoxFactor db_dq_ numerical != analytical");
          }
        }
      }

    }
    return error;


  // handle projection failures
  } catch(QuadricProjectionException& e) {
    
    // cout << e.what() << ": Quadric " << DefaultKeyFormatter(this->key2());
    // cout << " and pose " << DefaultKeyFormatter(this->key1()) << endl;
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
    && imageDimensions_.isApprox(other.imageDimensions_, tol)
    && noiseModel()->equals(*other.noiseModel(), tol)
    && key1() == other.key1() && key2() == other.key2();
  return equal;
}

} // namespace gtsam