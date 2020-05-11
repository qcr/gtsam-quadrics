/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Noise.cpp
 * @date May 1, 2020
 * @author Lachlan Nicholson
 * @brief a class to control the generation of noise 
 */

#include <quadricslam/base/Noise.h>
#include <quadricslam/geometry/ConstrainedDualQuadric.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
std::default_random_engine Noise::generator_;

/* ************************************************************************* */
void Noise::srand(double s) {

  // update generator seed
  generator_ = std::default_random_engine(s);
  return;
}

/* ************************************************************************* */
Matrix Noise::gaussianNoise(int n, int m, double mu, double sd) {

  // define random distribution
  std::normal_distribution<double> distribution(mu, sd);

  // define unary noise function
  auto gaussian = [&] {return distribution(Noise::generator_);};

  // apply function to all matrix elements
  auto noiseMatrix = Eigen::MatrixXd::NullaryExpr(n,m, gaussian);

  return noiseMatrix;
}

/* ************************************************************************* */
Values Noise::perturbValues(const Values& values, double sd) {
  Values newValues(values);
  for (auto key : values.keys()) {
    const Value& value = values.at(key);
    auto noiseVector = Noise::gaussianNoise(value.dim(),1, 0.0,sd);
    auto newValue = value.retract_(noiseVector); 
    newValues.update(key, *newValue);
  }
  return newValues;
}

/* ************************************************************************* */
Pose3 Noise::interpolate(const Pose3& p1, const Pose3& p2, const double& percent) {
  return gtsam::interpolate<Pose3>(p1, p2, percent);
}




}