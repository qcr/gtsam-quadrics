/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Evaluation.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a class with methods to evaluate the System
 */

#include <quadricslam/base/Evaluation.h>

namespace gtsam {
  
/* ************************************************************************* */
Vector4 Evaluation::evaluateResult(const vector<Pose3>& estimatedTrajectory, const vector<ConstrainedDualQuadric>& estimatedQuadrics,
  const vector<Pose3>& trueTrajectory, const vector<ConstrainedDualQuadric>& trueQuadrics) {
  
  // evaluate trajectory

  // evaluate quadrics

  // return metrics
}

/* ************************************************************************* */
Vector2 Evaluation::evaluateTrajectory(const vector<Pose3>& estimatedTrajectory, const vector<Pose3>& trueTrajectory) {

  // evaluate ATE_trans 
}


} // namespace gtsam