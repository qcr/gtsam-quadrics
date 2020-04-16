/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testAlignedBox2.cpp
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief test cases for AlignedBox2
 */

#include <CppUnitLite/TestHarness.h>

#include <quadricslam/geometry/AlignedBox2.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

static const Vector4 vector1(0,0,0,0);
static const Vector4 vector2(2.0,3.0,4.0,5.0);
static const AlignedBox2 b(0.0,1.3,2.0,3.5);

TEST(AlignedBox2, constructors) {
  AlignedBox2 box1;
  AlignedBox2 box2(2.0,3.0,4.0,5.0);
  AlignedBox2 box3(vector2);

  EXPECT(assert_equal(box1.vector(), vector1));
  EXPECT(assert_equal(box2.vector(), vector2));
  EXPECT(assert_equal(box3.vector(), vector2));
}

TEST(AlignedBox2, equals) {
  CHECK(b.equals(b));
  AlignedBox2 q(0.0,0.0,0.0,0.0);
  CHECK(!b.equals(q));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
