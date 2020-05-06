/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Trajectory.cpp
 * @date May 5, 2020
 * @author Lachlan Nicholson
 * @brief contains a class for a camera trajectory
 */

#include <quadricslam/containers/Trajectory.h>

using namespace std;


namespace gtsam {
  
/* ************************************************************************* */
Trajectory::Trajectory(const vector<Pose3>& poses) {
  for (unsigned i = 0; i < poses.size(); i++) {
    trajectory_[symbol('x', i)] = poses[i];
  }
}

/* ************************************************************************* */

}