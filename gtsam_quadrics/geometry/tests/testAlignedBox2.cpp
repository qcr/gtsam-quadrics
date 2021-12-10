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

#include <gtsam_quadrics/geometry/AlignedBox2.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam_quadrics;

static const Vector4 vector1(0,0,0,0);
static const Vector4 vector2(2.0,3.0,4.0,5.0);
static const AlignedBox2 b(0.0,1.3,2.0,3.5);

TEST(AlignedBox2, Constructors) {
  AlignedBox2 box1;
  AlignedBox2 box2(2.0,3.0,4.0,5.0);
  AlignedBox2 box3(vector2);
  AlignedBox2 box4(box3);

  EXPECT(assert_equal(vector1, box1.vector()));
  EXPECT(assert_equal(vector2, box2.vector()));
  EXPECT(assert_equal(vector2, box3.vector()));
  EXPECT(assert_equal(vector2, box4.vector()));
}

TEST(AlignedBox2, Accessors) {
  AlignedBox2 box(1.,2.,3.,4.);

  EXPECT(assert_equal(1., box.xmin()));
  EXPECT(assert_equal(2., box.ymin()));
  EXPECT(assert_equal(3., box.xmax()));
  EXPECT(assert_equal(4., box.ymax()));
  EXPECT(assert_equal(Vector4(1.,2.,3.,4.), box.vector()));
  EXPECT(assert_equal(Point2(1.,2.), box.minPoint()));
  EXPECT(assert_equal(Point2(3.,4.), box.maxPoint()));
}

TEST(AlignedBox2, WidthHeight) {
  AlignedBox2 box(1.5, 2.7, 3.5, 12.1);

  EXPECT(assert_equal(3.5-1.5, box.width()));
  EXPECT(assert_equal(12.1-2.7, box.height()));
}

TEST(AlignedBox2, Lines) {
  AlignedBox2 box(2.,3.,4.,5.);
  std::vector<Vector3> lines = box.lines();
  
  std::vector<Vector3> expected;
  expected.push_back(Vector3(-1/2., 0, 1));
  expected.push_back(Vector3(0, -1/3., 1));
  expected.push_back(Vector3(-1/4., 0, 1));
  expected.push_back(Vector3(0, -1/5., 1));

  for (unsigned i = 0; i < lines.size(); i++) {
    // test that the normalized homog representation is the same
    Vector3 normExpected = expected[i]/expected[i].tail<1>()(0);
    Vector3 normActual = lines[i]/lines[i].tail<1>()(0);
    EXPECT(assert_equal(normExpected, normActual));
  }
}

TEST(AlignedBox2, Contains) {
  AlignedBox2 box1(0, 0, 10, 10);
  AlignedBox2 fullyContained(3,3,7,7);
  AlignedBox2 notContained(15,15, 20,20);
  AlignedBox2 surrounding(-3,-3, 13,13);
  AlignedBox2 partiallyContained(5,5, 15,15);
  AlignedBox2 edgeIntersects(-5,-5, 5,15);
  AlignedBox2 leftEdge(-3,5, 0, 7);
  AlignedBox2 rightEdge(10,5, 15, 7);

  EXPECT((true == box1.contains(fullyContained)));
  EXPECT((false == box1.intersects(fullyContained)));

  EXPECT((false == box1.contains(notContained)));
  EXPECT((false == box1.intersects(notContained)));

  EXPECT((false == box1.contains(surrounding)));
  EXPECT((false == box1.intersects(surrounding)));

  EXPECT((false == box1.contains(partiallyContained)));
  EXPECT((true == box1.intersects(partiallyContained)));

  EXPECT((false == box1.contains(edgeIntersects)));
  EXPECT((true == box1.intersects(edgeIntersects)));

  EXPECT((false == box1.contains(leftEdge)));
  EXPECT((true == box1.intersects(leftEdge)));

  EXPECT((false == box1.contains(rightEdge)));
  EXPECT((true == box1.intersects(rightEdge)));
}

TEST(AlignedBox2, Equals) {
  AlignedBox2 q(0.0,0.0,0.0,0.0);
  EXPECT(b.equals(b));
  EXPECT(!b.equals(q));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
