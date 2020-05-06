/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testTestClass.cpp
 * @date May 1, 2020
 * @author Lachlan Nicholson
 * @brief test cases for TestClass
 */

#include <CppUnitLite/TestHarness.h>

#include <quadricslam/base/TestClass.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

static const double double1 = 2.5;

TEST(TestClass, constructors) {
  TestClass tc1;
  TestClass tc2(2.5);

  EXPECT(assert_equal(double1, tc2.getx()));
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
