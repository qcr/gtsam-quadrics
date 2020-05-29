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


#include <CppUnitLite/TestHarness.h>

#include <quadricslam/geometry/DualConic.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

static const Matrix33 matrix1((Matrix33() << 1.0, 0.0, 0.0, 
                                            0.0, 1.0, 0.0,
                                            0.0, 0.0, -1.0).finished());

static const Matrix33 matrix2((Matrix33() << 1,2,3,4,5,6,7,8,9).finished());
static const DualConic conic((Matrix33() << 3,4,5,6,7,8,9,10,11).finished());


TEST(DualConic, constructors) {
  DualConic conic1;
  DualConic conic2((Matrix33() << 1,2,3,4,5,6,7,8,9).finished());

  EXPECT(assert_equal(matrix1, conic1.matrix()));
  EXPECT(assert_equal(matrix2, conic2.matrix()));
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
