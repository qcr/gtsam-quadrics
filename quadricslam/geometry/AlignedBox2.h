/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file AlignedBox2.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief an axis aligned 2D bounding box
 */

#pragma once

#include <gtsam/geometry/Pose3.h>

namespace gtsam {

  /**
   * @class AlignedBox2
   * A constrained dual quadric (r,t,s): see Nicholson et al. 2019 for details
   */
  class GTSAM_EXPORT AlignedBox2 {

    protected:
      Vector4 tlbr_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /// @name Constructors and named constructors
      /// @{
      /** default constructor */
      AlignedBox2() : tlbr_(0,0,0,0) {};
      
      /**
       * Constructor from parts
       */
      AlignedBox2(const double& xmin, const double& ymin, const double& xmax, const double& ymax);

      /**
       * Constructor from vector
       * @param tlbr vector of xmin,ymin,xmax,ymax (Vector4)
       */
      AlignedBox2(Vector4 tlbr);

      /// @}
      /// @name Testable group traits
      /// @{

      /**
       * returns box in xmin,ymin,xmax,ymax vector
       */
      Vector4 vector() const;
        
      /**
       * Prints the dual quadric with optional string
       */
      void print(const std::string& s = "") const;

      /**
       * Compares two ellipsoids
       */
      bool equals(const AlignedBox2& other, double tol = 1e-9) const;
      /// @}
  };
    
} // namespace gtsam
