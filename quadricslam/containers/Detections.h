/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Detections.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief contains a class which holds detections for a video sequence
 */

#pragma once

#include <quadricslam/geometry/AlignedBox2.h>

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
 * @class Detections
 * Holds a set of alignedboxes (box, quadricKey, poseKey)
 */
class Detections {
  private:
    // how can we store boxes,key1s,key2s
  
  public:

    /// @name Constructors and named constructors
    /// @{

    /** Default constructor */
    Detections() {};

    /** Constructor from boxes and keys */
    Detections(const vector<AlignedBox2>& boxes, const vector<Key>& poseKeys, const vector<Key>& objectKeys);

    /// @}
    /// @name Standard interface
    /// @{

    /** Retrieve detection at key */
    AlignedBox2 at(const Key& poseKey, const Key& objectKeys) const;

    /** Retrieve all detections at pose */
    vector<AlignedBox2> atPose(const Key& poseKey) const;

    /** Retrieve all detections of landmark */
    vector<AlignedBox2> atObject(const Key& objectKey) const;

    /** Add gaussian noise to each aligned box */
    Detections addNoise(const double& sd) const;

    /// @}
};

} // namespace gtsam