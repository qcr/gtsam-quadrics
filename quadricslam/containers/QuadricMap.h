/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file QuadricMap.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief contains a class for a camera trajectory
 */

#pragma once

#include <quadricslam/geometry/ConstrainedDualQuadric.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <vector>
#include <map>
#include <utility>

using std::vector;
using std::map;
using std::pair;

namespace gtsam {

/**
 * @class QuadricMap
 * Holds a set of quadrics (Quadric, landmarkKey)
 */
class QuadricMap {
  private:
    map<Key, ConstrainedDualQuadric> quadrics_;

  public:

    /// @name Constructors and named constructors
    /// @{

    /** Default constructor */
    QuadricMap() : quadrics_() {};

    /** Constructor from quadrics */
    QuadricMap(const vector<ConstrainedDualQuadric>& quadrics);

    /// @}
    /// @name Standard interface
    /// @{

    /** Retrieve pose at key */
    ConstrainedDualQuadric at(const Key& key) const;

    /** Transform map to provided coordinate frame */
    QuadricMap transformTo(const Pose3& reference) const;

    /// @}
    /// @name Iterable
    /// @{

    using iterator = map<Key, ConstrainedDualQuadric>::iterator;
    using const_iterator = map<Key, ConstrainedDualQuadric>::const_iterator;
    iterator begin() { return quadrics_.begin();};
    iterator end() { return quadrics_.end();};
    const_iterator begin() const { return quadrics_.begin();};
    const_iterator end() const { return quadrics_.end();};

    /// @}
};

} // namespace gtsam