/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Noise.h
 * @date May 1, 2020
 * @author Lachlan Nicholson
 * @brief a class to control the generation of noise 
 */

#pragma once

#include <gtsam/base/Matrix.h>

#include <vector>
#include <random>

namespace gtsam {

class Noise {
  private:
    static std::default_random_engine generator_;

  public:
    static Matrix gaussianNoise(int n, int m, double mu, double sd);
    static void srand(double s);
};

}