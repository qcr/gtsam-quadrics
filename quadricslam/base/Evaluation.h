/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Evaluation.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a class with methods to evaluate the System
 */

#pragma once

#include <quadricslam/geometry/ConstrainedDualQuadric.h>

#include <gtsam/geometry/Pose3.h>

#include <vector>

using namespace std;

namespace gtsam {

/**
 * @class Evaluation
 * containing metods to evaluate system results
 */
class Evaluation {
  public:

    /** Compares trajectory + map against ground truth */
    static Vector4 evaluateResult(const vector<Pose3>& estimatedTrajectory, const vector<ConstrainedDualQuadric>& estimatedQuadrics,
      const vector<Pose3>& trueTrajectory, const vector<ConstrainedDualQuadric>& trueQuadrics);

    /** Compares trajectory against ground truth */
    static Vector2 evaluateTrajectory(const vector<Pose3>& estimatedTrajectory, const vector<Pose3>& trueTrajectory);
};

} // namespace gtsam