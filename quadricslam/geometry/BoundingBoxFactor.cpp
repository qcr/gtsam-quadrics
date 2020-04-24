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
#include <quadricslam/base/NotImplementedException.h>
#include <gtsam/base/numericalDerivative.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Vector BoundingBoxFactor::evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric,
  boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const {

  try {

    Eigen::Matrix<double, 9,6> dC_dx;
    Eigen::Matrix<double, 9,9> dC_dq;
    DualConic dC = QuadricCamera::project(quadric, pose, calibration_, H1?&dC_dq:0, H2?&dC_dx:0);

    Eigen::Matrix<double, 4,9> db_dC;
    AlignedBox2 predictedBounds = dC.bounds(H1||H2?&db_dC:0);
    Vector4 error = measured_.vector() - predictedBounds.vector();
    if (H1) {
      *H1 = db_dC * dC_dx;
      if (CHECK_ANALYTICAL) {
        boost::function<Vector(const Pose3&, const ConstrainedDualQuadric&)> funPtr(boost::bind(&BoundingBoxFactor::evaluateError, this, _1, _2, boost::none, boost::none));
				Eigen::Matrix<double, 4,6> db_dx_ = numericalDerivative21(funPtr, pose, quadric, 1e-6);
        if (!db_dx_.isApprox(*H1)) {
          cout << "WARNING: numerical != analytical" << endl;
          cout << "Analytical db_dx_:\n" << *H1 << endl;
          cout << "Numerical db_dx_:\n" << db_dx_ << endl << endl;
        }
      }
    } if (H2) {
      *H2 = db_dC * dC_dq; 
      if (CHECK_ANALYTICAL) {
        boost::function<Vector(const Pose3&,  const ConstrainedDualQuadric&)> funPtr(boost::bind(&BoundingBoxFactor::evaluateError, this, _1, _2, boost::none, boost::none));
				Eigen::Matrix<double, 4,9> db_dq_ = numericalDerivative22(funPtr, pose, quadric, 1e-6);
        if (!db_dq_.isApprox(*H2)) {
          cout << "WARNING: numerical != analytical" << endl;
          cout << "Analytical db_dq_:\n" << *H2 << endl;
          cout << "Numerical db_dq_:\n" << db_dq_ << endl << endl;
        }
      }
    }
    return error;

  } catch(QuadricProjectionException& e) {
    
    cout << e.what() << ": Quadric " << DefaultKeyFormatter(this->key2());
    cout << " moved behind camera " << DefaultKeyFormatter(this->key1()) << endl;
    throw NotImplementedException();

  }
}


Expression<AlignedBox2> BoundingBoxFactor::expression(const Expression<Pose3>& pose, const Expression<ConstrainedDualQuadric>& quadric) const {

  Expression<boost::shared_ptr<Cal3_S2>> calibration(calibration_); // constant calibration
  Expression<DualConic> dualConic(&QuadricCamera::project, quadric, pose, calibration); 
  Expression<AlignedBox2> predictedBounds(dualConic, &DualConic::bounds);
  return predictedBounds;

}


} // namespace gtsam