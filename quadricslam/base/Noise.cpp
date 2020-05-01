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

using namespace std;

namespace gtsam {

/* ************************************************************************* */
std::default_random_engine Noise::generator_;

/* ************************************************************************* */
Matrix Noise::gaussianNoise(int n, int m, double mu, double sd) {

  // define random distribution
  std::normal_distribution<double> distribution(mu, sd);

  // define noise matrix
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> noiseMatrix(n, m);

  // define unary noise function
  auto gaussian = [&] (double x) {return distribution(Noise::generator_);};

  // apply function to all matrix elements
  noiseMatrix = noiseMatrix.unaryExpr(gaussian);

  return noiseMatrix;
}

/* ************************************************************************* */
void Noise::srand(double s) {

  // update generator seed
  generator_ = std::default_random_engine(s);
  return;
}


}