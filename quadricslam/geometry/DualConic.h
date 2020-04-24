/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DualConic.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a dual conic
 */

#pragma once

#include<quadricslam/geometry/AlignedBox2.h>

namespace gtsam {

  /**
   * @class DualConic
   * A dual conic (Matrix33)
   */
  class GTSAM_EXPORT DualConic {

    protected:
      Matrix33 dC_; ///< 3x3 dual conic matrix

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /// @name Constructors and named constructors
      /// @{
      
      /** default constructor, unit circle at origin */
      DualConic();

      /** constructor from 3x3 matrix */
      DualConic(const Matrix33& dC);

      /** return 3x3 conic matrix */
      Matrix33 matrix() const;

      /** normalize, inline, the conic */
      void normalize();

      /** return 2D bounds on image plane */
      AlignedBox2 bounds(OptionalJacobian<4,5> H = boost::none) const;

      /** dc/dC */
      static Eigen::Matrix<double, 5,9> dc_dC(void);
      static Eigen::Matrix<double, 9,5> dC_dc(void);


      /// @}
      /// @name Testable group traits
      /// @{
        
      /**
       * Prints the dual quadric with optional string
       */
      void print(const std::string& s = "") const;

      /**
       * Compares two ellipsoids
       */
      bool equals(const DualConic& other, double tol = 1e-9) const;
      /// @}
  };

  template<>
  struct traits<DualConic> {
    enum { dimension = 5};
  };  
    
} // namespace gtsam
