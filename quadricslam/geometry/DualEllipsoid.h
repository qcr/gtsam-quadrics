/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DualEllipsoid.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a constrained dual quadric
 */

#pragma once

#include <gtsam/geometry/Pose3.h>

namespace gtsam {

  /**
   * @class DualEllipsoid
   * A constrained dual quadric (r,t,s): see Nicholson et al. 2019 for details
   */
  class GTSAM_EXPORT DualEllipsoid {

    protected:
      Pose3 pose_; ///< 3D pose of ellipsoid
      Vector3 radii_; ///< radii of x,y,z axii

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /// @name Constructors and named constructors
      /// @{
      
      /** default constructor, unit sphere at origin */
      DualEllipsoid();

      /**
       * Constructor from 3x3 Matrix,
       * Here we ensure the quadric is ellipsoidal
       * and constrain manually if it is not. 
       * @param dQ
       */
      DualEllipsoid(const Matrix44& dQ);

      /**
       * Constructor pose and radii
       * @param pose quadric pose (Pose3)
       * @param radii quadric radii (Vector3)
       */
      DualEllipsoid(const Pose3& pose, const Vector3& radii);

      /**
       * Constructor from rotation, translation and shape
       * @param R quadric rotation (Rot3)
       * @param t quadric translation (Point3)
       * @param s quadric shape (Vector3)
       */
      DualEllipsoid(const Rot3& R, const Point3& t, const Vector3& s);

      /**
       * Constructs 4x4 quadric matrix from pose & radii
       * @return 4x4 constrained quadric
       */
      Matrix44 matrix() const;

      /**
       * Calculates the AlignedBox3 bounds of the ellipsoid
       * @return 3D axis aligned bounding box
       */ 
      Vector6 bounds3() const;

      /// @}
      /// @name Manifold group traits
      /// @{
      enum { dimension = 9 };

      /**
       * The retract function
       * @param v vector in tangent space (change in latent dims)
       * @return dual ellipsoid on the manifold
       */
      DualEllipsoid retract(const Vector9& v) const;

      /**
       * The local function
       * @param dE dual ellipsoid on the manifold
       * @return vector between ellipsoids in tangent space
       */
      Vector9 localCoordinates(const DualEllipsoid& other) const;

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
      bool equals(const DualEllipsoid& other, double tol = 1e-9) const;
      /// @}
  };

  // add DualEllipsoid to Manifold group
  template <>
  struct traits<DualEllipsoid> : public internal::Manifold<DualEllipsoid> {};

  template <>
  struct traits<const DualEllipsoid> : public internal::Manifold<DualEllipsoid> {};
    
} // namespace gtsam
