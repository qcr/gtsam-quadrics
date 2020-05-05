/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Trajectory.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief contains a class for a camera trajectory
 */

#pragma once

#include <quadricslam/base/Noise.h>
#include <quadricslam/containers/Odometry.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <vector>
#include <map>
#include <iterator>

using std::vector;
using std::map;

namespace gtsam {

/** Forward declare for circular includes */
class Odometry;

/**
 * @class Trajectory
 * Holds a trajectory (Pose3, key)
 */
class Trajectory {
  private:
    map<Key, Pose3> trajectory_;

  public:

    /// @name Constructors and named constructors
    /// @{

    /** Default constructor */
    Trajectory() : trajectory_() {};

    /** Constructor from poses */
    Trajectory(const vector<Pose3>& poses);

    /// @}
    /// @name Misc
    /// @{
    
    /** Retrieve pose at key */
    Pose3 at(const Key& key) const;

    /** Convert the trajectory to relative poses */
    gtsam::Odometry asOdometry(void) const;

    /** Transform trajectory to provided coordinate frame */
    Trajectory transformTo(const Pose3& reference) const;

    /** Interpolate poses and provide new keys */
    Trajectory interpolate(const int& n) const;

    /// @}
    /// @name Iterable
    /// @{

    using iterator = map<Key, Pose3>::iterator;
    using const_iterator = map<Key, Pose3>::const_iterator;
    iterator begin() { return trajectory_.begin();};
    iterator end() { return trajectory_.end();};
    const_iterator begin() const { return trajectory_.begin();};
    const_iterator end() const { return trajectory_.end();};

    /// @}
};

} // namespace gtsam