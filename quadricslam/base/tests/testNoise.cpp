/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testNoise.cpp
 * @date May 1, 2020
 * @author Lachlan Nicholson
 * @brief test cases for Noise
 */

#include <CppUnitLite/TestHarness.h>

#include <quadricslam/base/Noise.h>

#include <cmath>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

static const double expectedMean = 25.8;
static const double expectedStd = 1.3;

TEST(Noise, GaussianNoise) {
  Eigen::Matrix<double, -1,-1> noiseMatrix = Noise::gaussianNoise(1000, 1000, expectedMean, expectedStd);
  auto noiseArray = noiseMatrix.array();
  double actualMean = noiseArray.mean();
  double actualStd = std::sqrt((noiseArray-noiseArray.mean()).square().sum()/(noiseArray.size()-1));

  EXPECT(assert_equal(expectedMean, actualMean, 1e-2));
  EXPECT(assert_equal(expectedStd, actualStd, 1e-2));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
