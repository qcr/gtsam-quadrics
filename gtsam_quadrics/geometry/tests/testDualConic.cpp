/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testDualConic.cpp
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief test cases for DualConic
 */

/**
 * TODO:
 * - update tests to account for symmetric requirements
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_quadrics/geometry/DualConic.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam_quadrics;

static const Matrix33 expectedDefault((Matrix33() << 1.0, 0.0, 0.0, 
                                                     0.0, 1.0, 0.0,
                                                     0.0, 0.0, -1.0).finished());
static const Matrix33 matrix1((Matrix33() << 1,2,3,4,5,6,7,8,9).finished());
static const DualConic conic((Matrix33() << 3,4,5,6,7,8,9,10,11).finished());

TEST(DualConic, Constructors) {
  DualConic conic1;
  DualConic conic2((Matrix33() << 1,2,3,4,5,6,7,8,9).finished());
  DualConic conic3(Pose2(), Vector2(1.1,3.2));
  DualConic conic4(conic2);

  EXPECT(assert_equal(expectedDefault, conic1.matrix()));
  EXPECT(assert_equal(matrix1, conic2.matrix()));
  EXPECT(assert_equal(matrix1, conic4.matrix()));
}

TEST(DualConic, MatrixAndNormalize) {
  DualConic conic1(matrix1); 
  Matrix33 expectedNorm = matrix1/matrix1(2,2);
  EXPECT(assert_equal(matrix1, conic1.matrix()))
  EXPECT(assert_equal(expectedNorm, conic1.normalize().matrix()))
}

/// TODO: visibility classification 
TEST(DualConic, Bounds) {
  DualConic fullyVisible(Pose2(Rot2(), Point2(45.2,13.8)), (Vector2() << 1.0,3.0).finished());
  DualConic partiallyVisible(Pose2(Rot2(), Point2(45.2,13.8)), (Vector2() << 1.0,3.0).finished());
  DualConic notVisible(Pose2(Rot2(), Point2(45.2,13.8)), (Vector2() << 1.0,3.0).finished());

  Vector4 expectedBounds = Vector4(45.2,13.8,45.2,13.8) + Vector4(-1.0, -3.0, 1.0, 3.0);
  EXPECT(assert_equal(expectedBounds, fullyVisible.bounds().vector()));
}

/// TODO: test degeneracy without inversion
TEST(DualConic, Classification) {
  DualConic conic1;
  DualConic conic2((Matrix33() << 1.0, 0.0, 0.0, 
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, 0.0).finished());
  DualConic conic3((Matrix33() << 0.0, 0.0, 4.0, 
                                    0.0, 1.0, 0.0,
                                    4.0, 0.0, -1.0).finished());
  EXPECT(conic1.isDegenerate() == false);
  EXPECT(conic1.isEllipse() == true);
  EXPECT(conic2.isDegenerate() == true);
  EXPECT(conic2.isEllipse() == false);
  EXPECT(conic3.isDegenerate() == false);
  EXPECT(conic3.isEllipse() == false);
}

TEST(DualConic, Equals) {
  DualConic conic1((Matrix33() << 3,4,5,6,7,8,9,10,11).finished());
  DualConic conic2((Matrix33() << 3,4,12,6,7,8,9,10,11).finished());

  EXPECT(conic1.equals(conic));
  EXPECT(!conic1.equals(conic2));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
