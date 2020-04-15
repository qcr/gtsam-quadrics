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
#include <quadricslam/geometry/AlignedBox2.h>
#include <quadricslam/base/TestClass.h>

using namespace std;
using namespace gtsam;

int main() {

  // run simple test to ensure compiling correctly
  ConstrainedDualQuadric x; 
  AlignedBox2 box(1.2, 3.4, 5.6, 7.8);
  box.print("myboxis");

  TestClass t(5);
  double r = t.doWork(2);
  // cout << r << endl;

  // Vector4 y(1.0,2.0,3.0,4.0);
  // cout << y.pow(2) << endl;

  // power of a matrix
  Matrix22 z = (Matrix22() << 1.0,2.0,3.0,4.0).finished();
  print((Matrix)z, "z:");
  print((Matrix)z.array().pow(2).matrix(), "z.array():");

  return 1;
}

