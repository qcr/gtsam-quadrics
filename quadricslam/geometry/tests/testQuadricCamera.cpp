/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testQuadricCamera.cpp
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief test cases for QuadricCamera
 */

#include <CppUnitLite/TestHarness.h>

#include <quadricslam/geometry/QuadricCamera.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

static const DualConic conic1 = DualConic((Matrix33() << 2.181975e+06, 1.843200e+06, 7.680000e+03,
                                              1.843200e+06, 1.106775e+06, 5.760000e+03,
                                              7.680000e+03, 5.760000e+03, 2.400000e+01).finished());

TEST(QuadricCamera, Constructors) {
  QuadricCamera camera1;
}

TEST(QuadricCamera, ProjectQuadric) {
  Pose3 cameraPose(Rot3(), Point3(0,0,-5));
  boost::shared_ptr<Cal3_S2> K(new Cal3_S2(525.0,525.0,0.0,320.0,240.0));
  QuadricCamera camera1(cameraPose, K);
  Matrix34 P = camera1.transformToImage();
  ConstrainedDualQuadric Q;
  DualConic C = camera1.project(Q);

  DualConic C2 = QuadricCamera::project(Q, cameraPose, K);
  EXPECT(assert_equal(conic1, C));
  EXPECT(assert_equal(conic1, C2));
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
