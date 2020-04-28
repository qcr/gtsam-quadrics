/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Jacobians.h
 * @date Apr 22, 2020
 * @author Lachlan Nicholson
 * @brief some helper functions for calculating jacobians 
 */
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>


namespace gtsam {
namespace internal {


Eigen::VectorXd flatten(Matrix X);
Matrix44 matrix(const Pose3& pose, OptionalJacobian<16,6> H = boost::none);
Matrix kron(const Matrix m1, const Matrix m2);
Matrix TVEC(const int m, const int n);

static auto I44 = Matrix::Identity(4,4);
static auto I34 = Matrix::Identity(3,4);
static auto I33 = Matrix::Identity(3,3);

static auto T34 = TVEC(3,4);
static auto T44 = TVEC(4,4);


} // namespace internal
} // namespace gtsam