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
#include <quadricslam/geometry/BoundingBoxFactor.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;

int main() {

  ConstrainedDualQuadric x; 
  AlignedBox2 box(1.2, 3.4, 5.6, 7.8);
  TestClass t(5);

  // // power of a matrix
  // Matrix22 z = (Matrix22() << 1.0,2.0,3.0,4.0).finished();
  // print((Matrix)z, "z:");
  // print((Matrix)z.array().pow(2).matrix(), "z.array():");

  // create and use bbf
  AlignedBox2 measured(15.2, 18.5, 120.5, 230.2);
  boost::shared_ptr<Cal3_S2> calibration(new Cal3_S2(525.0, 525.0, 0.0, 320.0, 240.0));
  boost::shared_ptr<Vector2> imageDimensions(new Vector2(320.0, 240.0));
  Key poseKey(Symbol('x', 1));
  Key quadricKey(Symbol('q', 1));
  boost::shared_ptr<noiseModel::Diagonal> model = noiseModel::Diagonal::Sigmas(Vector4(0.2,0.2,0.2,0.2));
  Pose3 pose(Rot3(), Point3(0,0,-3));
  ConstrainedDualQuadric quadric;

  BoundingBoxFactor bbf(measured, calibration, imageDimensions, poseKey, quadricKey, model);
  Vector4 error = bbf.evaluateError(pose, quadric);

  cout << "done" << endl;
  return 1;
}

