/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Odometry.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief Odometry contains the relative movements between poses in a trajectory
 */

#pragma once

#include <quadricslam/containers/Trajectory.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <vector>
#include <map>
#include <utility>

using std::vector;
using std::map;
using std::pair;

namespace gtsam {

/** Forward declare for circular includes */
class Trajectory;

/**
 * @class Odometry
 * Holds a set of relative poses (Pose3, key1, key2)
 * Assumes sequential poses and implicit data-association.
 */
class Odometry {
  private:
    map<Key, Pose3> trajectory_;

  public:

    /// @name Constructors and named constructors
    /// @{

    /** Default constructor */
    Odometry() : trajectory_() {};

    /** Constructor from poses */
    Odometry(const vector<Pose3>& poses);

    /** Convert the trajectory to relative poses */
    Trajectory asTrajectory(void) const;
    /// @}
};

} // namespace gtsam