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

static AlignedBox2 measured(15.2, 18.5, 120.5, 230.2);
static boost::shared_ptr<Cal3_S2> calibration(new Cal3_S2(525.0, 525.0, 0.0, 320.0, 240.0));
static boost::shared_ptr<Vector2> imageDimensions(new Vector2(320.0, 240.0));
static Key poseKey(Symbol('x', 1));
static Key quadricKey(Symbol('q', 1));
static boost::shared_ptr<noiseModel::Diagonal> model = noiseModel::Diagonal::Sigmas(Vector4(0.2,0.2,0.2,0.2));
static Pose3 cameraPose(Rot3(), Point3(0,0,-3));
static ConstrainedDualQuadric quadric;

TEST(BoundingBoxFactor, Constructors) {
  BoundingBoxFactor bbf;
}

TEST(BoundingBoxFactor, Error) {
  BoundingBoxFactor bbf(measured, calibration, imageDimensions, poseKey, quadricKey, model);
  bbf.evaluateError(cameraPose, quadric);
}

TEST(BoundingBoxFactor, ExpressionError) {
  BoundingBoxFactor bbf(measured, calibration, imageDimensions, poseKey, quadricKey, model);

  Expression<Pose3> cameraPose_('x',1);
  Expression<ConstrainedDualQuadric> quadric_('q',1);
  Values values;
  values.insert(symbol('x',1), cameraPose);
  values.insert(symbol('q',1), quadric);
  std::vector<Matrix> gradients;
  Eigen::Matrix<double, 4,6> db_dx;
  Eigen::Matrix<double, 4,9> db_dq;
  gradients.push_back(db_dq);
  gradients.push_back(db_dx);

  Expression<AlignedBox2> bbfExpression = bbf.expression(cameraPose_, quadric_);
  bbfExpression.value(values, gradients);
  db_dq = gradients[0];
  db_dx = gradients[1];
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
