/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file QuadricCamera.cpp
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief a class responsible for projecting quadrics
 */

#include <quadricslam/geometry/QuadricCamera.h>
#include <quadricslam/base/NotImplementedException.h>

namespace gtsam {

/* ************************************************************************* */
DualConic QuadricCamera::project(const ConstrainedDualQuadric& quadric) {
  throw QuadricProjectionException();
}
    
} // namespace gtsam
