/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testConstrainedDualQuadric.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief test cases for ConstrainedDualQuadric
 */


#include <CppUnitLite/TestHarness.h>

#include <quadricslam/geometry/ConstrainedDualQuadric.h>
#include <quadricslam/geometry/DualConic.h>
#include <quadricslam/geometry/QuadricCamera.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

// test constructor from only shape vector
TEST(ConstrainedDualQuadric, radii_constructor) {
  const ConstrainedDualQuadric Q(Pose3(), Vector3(0.9,1.0,1.1));
  Matrix44 expected = (Matrix44() <<  0.9*0.9, 0.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0, 0.0,
                                      0.0, 0.0, 1.1*1.1, 0.0,
                                      0.0, 0.0, 0.0, -1.0).finished();
  Matrix44 actual = Q.matrix();
  EXPECT(assert_equal(expected, actual));
}

// test bounds calculation for quadric with no rotation
TEST(ConstrainedDualQuadric, translation_bounds) {
  const ConstrainedDualQuadric Q(Pose3(Rot3(), Point3(2,3,-4.1)), Vector3(0.9,1.0,1.1));

  Vector6 expected = (Vector6() << 1.1, 2.0, -5.2, 2.9, 4.0, -3.0).finished();
  Vector6 actual = Q.bounds();

  EXPECT(assert_equal(expected, actual));
}

TEST(ConstrainedDualQuadric, visibility) {

  Pose3 newFrame = Pose3::Retract((Vector6() << 1.,2.,3.,4.,5.,6.).finished());

  Pose3 cameraPose(Rot3(), Point3(0,0,-5));
  cameraPose = newFrame.compose(cameraPose);
  double image_width = 320.0;
  double image_height = 240.0;
  boost::shared_ptr<Cal3_S2> calibration(new Cal3_S2(525.0, 525.0, 0.0, image_width/2.0, image_height/2.0));
  ConstrainedDualQuadric qVisible(newFrame.compose(Pose3(Rot3(), Point3(0,0,0))), Vector3(0.3,0.3,0.3));
  ConstrainedDualQuadric qPartial(newFrame.compose(Pose3(Rot3(), Point3(0,1.2,0))), Vector3(0.3,0.3,0.3));
  ConstrainedDualQuadric qBehind(newFrame.compose(Pose3(Rot3(), Point3(0,0,-10))), Vector3(0.3,0.3,0.3));

  EXPECT(true   == qVisible.fullyVisible(cameraPose, calibration));
  EXPECT(false  == qPartial.fullyVisible(cameraPose, calibration));
  EXPECT(false  == qBehind.fullyVisible(cameraPose, calibration));

  EXPECT(false  == qVisible.partiallyVisible(cameraPose, calibration));
  EXPECT(true   == qPartial.partiallyVisible(cameraPose, calibration));
  EXPECT(false  == qBehind.partiallyVisible(cameraPose, calibration));

  EXPECT(false  == qVisible.notVisible(cameraPose, calibration));
  EXPECT(false  == qPartial.notVisible(cameraPose, calibration));
  EXPECT(true   == qBehind.notVisible(cameraPose, calibration));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
