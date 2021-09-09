/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testAlignedBox3.cpp
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief test cases for AlignedBox3
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_quadrics/geometry/AlignedBox3.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam_quadrics;

static const Vector6 v = (Vector6() << 1.,2.,3.,4.,5.,6.).finished();
static const Vector6 v0 = (Vector6().setZero());

TEST(AlignedBox3, Constructors) {
  AlignedBox3 box1;
  AlignedBox3 box2(1.,2.,3.,4.,5.,6.);
  AlignedBox3 box3(v);
  AlignedBox3 box4(box3);

  EXPECT(assert_equal(v0, box1.vector()));
  EXPECT(assert_equal(v, box2.vector()));
  EXPECT(assert_equal(v, box3.vector()));
  EXPECT(assert_equal(v, box4.vector()));
}

TEST(AlignedBox3, Accessors) {
  AlignedBox3 box(1.,2.,3.,4.,5.,6.);

  EXPECT(assert_equal(1., box.xmin()));
  EXPECT(assert_equal(2., box.xmax()));
  EXPECT(assert_equal(3., box.ymin()));
  EXPECT(assert_equal(4., box.ymax()));
  EXPECT(assert_equal(5., box.zmin()));
  EXPECT(assert_equal(6., box.zmax()));
  EXPECT(assert_equal(v, box.vector()));
}

TEST(AlignedBox3, DimensionsAndCentroid) {
  AlignedBox3 box(1.,2.,3.,4.,5.,6.);
  Vector3 expectedDimensions(1,1,1);
  Vector3 expectedCentroid(1.5, 3.5, 5.5);

  EXPECT(assert_equal(expectedDimensions, box.dimensions()));
  EXPECT(assert_equal(expectedCentroid, box.centroid()));
}

TEST(AlignedBox3, Equals) {
  AlignedBox3 b1(v0);
  AlignedBox3 b2(v);
  EXPECT(b2.equals(b2));
  EXPECT(!b2.equals(b1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
