/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBoundingBoxFactor.cpp
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief test cases for BoundingBoxFactor
 */

#include <CppUnitLite/TestHarness.h>

#include <quadricslam/geometry/BoundingBoxFactor.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;


TEST(BoundingBoxFactor, constructors) {
  BoundingBoxFactor bbf;
}

TEST(BoundingBoxFactor, error) {
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
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
