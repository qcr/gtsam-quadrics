/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TestClass.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a simple class used to test wrapper, etc
 */

#pragma once

#include <quadricslam/base/Noise.h>

#include <gtsam/base/Matrix.h>

#include <vector>

namespace gtsam {

/**
 * @class TestClass
 * A simple test class (double)
 */
class TestClass {
  private:
    double x_; ///< double to be stored

  public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @name Constructors and named constructors
    /// @{

    /** Default constructor */
    TestClass() {};

    /** Constructor from double */
    TestClass(double x) : x_(x) {};

    /// @}
    /// @name Testable group traits
    /// @{

    double getx(void) {return x_;}
    double doWork(double y);

    /// @}
};

} // namespace gtsam