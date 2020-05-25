/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Utilities.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a namespace providing a number of useful functions
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace gtsam {
namespace utils {

/** Interpolate poses */ 
Pose3 interpolate(const Pose3& p1, const Pose3& p2, const double& percent);

/** 
 * Converts Pose3 to Matrix and provides optional jacobians 
 * https://atmos.washington.edu/~dennis/MatrixCalculus.pdf
 * https://en.wikipedia.org/wiki/Kronecker_product
 * https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
 * https://people.maths.ox.ac.uk/gilesm/files/NA-08-01.pdf
 * Some Theorems on Matrix Differentiation with Special Reference to Kronecker Matrix Products (H. Neudecker, 1969)
 * A tutorial on SE(3) transformation parameterizations and on-manifold optimization Jose-Luis Blanco (p.35)
 * - https://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf
 * http://www.ee.ic.ac.uk/hp/staff/dmb/matrix/special.html#VecTranspose
 */
Matrix44 matrix(const Pose3& pose, OptionalJacobian<16,6> H = boost::none);

/** 
 * Performs the kronecker product 
 * See: https://en.wikipedia.org/wiki/Kronecker_product
 */
Matrix kron(const Matrix m1, const Matrix m2);

/** 
 * Builds the orthogonal transpose vectorization matrix of an m by n matrix 
 * See: http://www.ee.ic.ac.uk/hp/staff/dmb/matrix/special.html#VecTranspose
 */
Matrix TVEC(const int m, const int n);

} // namespace utils
} // namespace gtsam
