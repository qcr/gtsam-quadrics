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

#include <gtsam_quadrics/geometry/BoundingBoxFactor.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace std;
using namespace gtsam_quadrics;

static NonlinearFactorGraph graph;
static AlignedBox2 measured(15.2, 18.5, 120.5, 230.2);
static boost::shared_ptr<Cal3_S2> calibration(new Cal3_S2(525.0, 525.0, 0.0, 320.0, 240.0));
static Key poseKey(Symbol('x', 1));
static Key quadricKey(Symbol('q', 1));
static boost::shared_ptr<noiseModel::Diagonal> model = noiseModel::Diagonal::Sigmas(Vector4(0.2,0.2,0.2,0.2));
static Pose3 cameraPose(Rot3(), Point3(0,0,-3));
static ConstrainedDualQuadric quadric;

TEST(BoundingBoxFactor, Constructors) {
  BoundingBoxFactor bbf;
  BoundingBoxFactor bbf2(measured, calibration, poseKey, quadricKey, model);
}

TEST(BoundingBoxFactor, Accessors) {
  BoundingBoxFactor bbf(measured, calibration, poseKey, quadricKey, model);

  EXPECT(assert_equal(poseKey, bbf.poseKey()));
  EXPECT(assert_equal(quadricKey, bbf.objectKey()));
  EXPECT(assert_equal(measured, bbf.measurement()));
}

/// TODO: add specific test case 
TEST(BoundingBoxFactor, Error) {
  BoundingBoxFactor bbf(measured, calibration, poseKey, quadricKey, model);
  bbf.evaluateError(cameraPose, quadric);
}

TEST(BoundingBoxFactor, AddRemoveFromGraph) {
  BoundingBoxFactor bbf(measured, calibration, poseKey, quadricKey, model);
  graph.add(bbf);
  auto nlf = graph.at(0);
  BoundingBoxFactor bbf2 = *boost::dynamic_pointer_cast<BoundingBoxFactor>(graph.at(0)).get();
  EXPECT(assert_equal(bbf, bbf2));
}

TEST(BoundingBoxFactor, ExplicitAddRemoveFromGraph) {
  BoundingBoxFactor bbf(measured, calibration, poseKey, quadricKey, model);
  graph.add(bbf);
  BoundingBoxFactor bbf2 = *boost::dynamic_pointer_cast<BoundingBoxFactor>(graph.at(0)).get();
  EXPECT(assert_equal(bbf, bbf2));
}

TEST(BoundingBoxFactor, Equals) {
  BoundingBoxFactor bbf1(measured, calibration, poseKey, quadricKey, model);
  BoundingBoxFactor bbf2(measured, calibration, Symbol('x', 1), Symbol('q', 1), model);
  BoundingBoxFactor bbf3(measured, calibration, Symbol('x', 2), Symbol('q', 1), model);
  EXPECT(bbf1.equals(bbf1));
  EXPECT(bbf1.equals(bbf2));
  EXPECT(!bbf1.equals(bbf3));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
