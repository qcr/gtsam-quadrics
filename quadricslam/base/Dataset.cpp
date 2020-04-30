/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Dataset.cpp
 * @date Apr 30, 2020
 * @author Lachlan Nicholson
 * @brief abstract base dataset, interface to run system
 */

#include <quadricslam/base/Dataset.h>

namespace gtsam {
  
double Dataset::doWork(double y) {
  return x_*y;
}

}