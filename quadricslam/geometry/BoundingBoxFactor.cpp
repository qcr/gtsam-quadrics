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

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Vector BoundingBoxFactor::evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric,
  boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const {

  try {

    QuadricCamera camera(pose, calibration_);
    DualConic dC = camera.project(quadric);
    AlignedBox2 predictedBounds = dC.bounds();
    Vector4 error = measured_.vector() - predictedBounds.vector();
    return error;

  } catch(QuadricProjectionException& e) {
    
    cout << e.what() << ": Quadric " << DefaultKeyFormatter(this->key2());
    cout << " moved behind camera " << DefaultKeyFormatter(this->key1()) << endl;
    throw NotImplementedException();

  }

}


Expression<AlignedBox2> BoundingBoxFactor::expression(const Expression<Pose3>& pose, const Expression<ConstrainedDualQuadric>& quadric) const {

  // create pose_matrix (pose6v)
  // create quadric_matrix (quadric6v)
  // create projection matrix (pose_matrix, calibration)
  // create conic_matrix (projection_matrix)
  // create bounds (conic)

  // GOALS
  // express error wrt pose and quadric
  // use only functions that have jacobians
  // use class methods with Expression<res>(obj, func, arg)
  // obj has to be an expression tho

  // WHAT AM I ACTUALLY DOING / CAN I WRITE AS MATH
  // 1. create projection matrix (pose, calibration)
  // 2. project conic (P, Q)
  // 3. calculate bounds (C)


  // current jacobians that need to be calculated:
  // QuadricCamera::Create (6,6) [eye], (6,5) [eye?]
  // QuadricCamera::project (5,6), (5,9)
  // DualConic::bounds (4,5) 

  // - alternatively could use a static ::project(pose,quad,calib)
  // P = projection(pose, calib)
  // C = P*Q*P.T
  // box = bounds(C)

  // can I find somewhere that they calculate projection matrix?
  Expression<boost::shared_ptr<Cal3_S2>> calibration(calibration_); // constant calibration
  Expression<QuadricCamera> camera(&QuadricCamera::Create, pose, calibration); 
  Expression<DualConic> dualConic(camera, &QuadricCamera::project, quadric);
  Expression<AlignedBox2> predictedBounds(dualConic, &DualConic::bounds);
  return predictedBounds;



  // Expression<boost::shared_ptr<Cal3_S2>> calibration(calibration_); // constant calibration
  // Expression<DualConic> dualConic(QuadricCamera::project, quadric, pose, calibration); 
  // dc_dpose 5x6, dc_dquadric 5x9
  // Expression<AlignedBox2> predictedBounds(dualConic, &DualConic::bounds);
  // return predictedBounds;


}


} // namespace gtsam