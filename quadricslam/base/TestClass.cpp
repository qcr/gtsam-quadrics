/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TestClass.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a simple class used to test wrapper, etc
 */

#include <quadricslam/base/TestClass.h>

namespace gtsam {
  
double TestClass::doWork(double y) {
  return x_*y;
}

}