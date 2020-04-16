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
Vector BoundingBoxFactor::evaluateError(const Pose3 &pose, const ConstrainedDualQuadric &quadric,
  boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const {

  try {

    QuadricCamera camera(pose, calibration_);
    DualConic dC = camera.project(quadric);
    AlignedBox2 predictedBounds = dC.bounds();
    Vector4 error = measured_.vector() - predictedBounds.vector();

  } catch(QuadricProjectionException& e) {
    
    cout << e.what() << ": Quadric " << DefaultKeyFormatter(this->key2());
    cout << " moved behind camera " << DefaultKeyFormatter(this->key1()) << endl;
    throw NotImplementedException();

  }

}
  

} // namespace gtsam