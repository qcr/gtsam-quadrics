/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file main.cpp
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief a simple main function to facilitate debugging
 */

#include <quadricslam/geometry/ConstrainedDualQuadric.h>
#include <quadricslam/geometry/BoundingBoxFactor.h>
#include <quadricslam/geometry/QuadricCamera.h>
#include <quadricslam/geometry/AlignedBox2.h>
#include <quadricslam/geometry/DualConic.h>
#include <quadricslam/base/TestClass.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>

using namespace std;
using namespace gtsam;

int main() {

  cout << TEST_ANALYTICAL << endl;

  cout << "done" << endl;
  return 1;
}

