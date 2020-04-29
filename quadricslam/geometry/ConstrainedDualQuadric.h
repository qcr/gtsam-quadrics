/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ConstrainedDualQuadric.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a constrained dual quadric
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <random>

namespace gtsam {

  /**
   * @class ConstrainedDualQuadric
   * A constrained dual quadric (r,t,s): see Nicholson et al. 2019 for details
   */
  class GTSAM_EXPORT ConstrainedDualQuadric {

    protected:
      Pose3 pose_; ///< 3D pose of ellipsoid
      Vector3 radii_; ///< radii of x,y,z axii

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /// @name Constructors and named constructors
      /// @{
      
      /** default constructor, unit sphere at origin */
      ConstrainedDualQuadric();

      /**
       * Constructor from 4x4 Matrix,
       * Here we ensure the quadric is ellipsoidal
       * and constrain manually if it is not. 
       * @param dQ
       */
      ConstrainedDualQuadric(const Matrix44& dQ);

      /**
       * Constructor pose and radii
       * @param pose quadric pose (Pose3)
       * @param radii quadric radii (Vector3)
       */
      ConstrainedDualQuadric(const Pose3& pose, const Vector3& radii);

      /**
       * Constructor from rotation, translation and shape
       * @param R quadric rotation (Rot3)
       * @param t quadric translation (Point3)
       * @param r quadric radii (Vector3)
       */
      ConstrainedDualQuadric(const Rot3& R, const Point3& t, const Vector3& r);

      /**
       * named constructor for Expression
       */
      static ConstrainedDualQuadric Create(const Pose3& pose, const Vector3& radii, OptionalJacobian<9,6> H1, OptionalJacobian<9,3> H2) {
        return ConstrainedDualQuadric(pose, radii);
      }

      /**
       * Constructs 4x4 quadric matrix from pose & radii
       * Q = Z * Qc * Z.T
       * Z = quadric pose in global frame
       * Qc = centered dualquadric
       *     diagonal matrix of shape (s1^2, s2^2, s3^2, -1)
       * where s1,s2,s3 are the radius of each axis on the ellipse
       * hence (s1,s2,s3) = (r1,r2,r3) 
       * see Nicholson et. al 2019 QuadricSLAM for full details
       * @return 4x4 constrained quadric
       */
      Matrix44 matrix(OptionalJacobian<16,9> H = boost::none) const;

      /**
       * Calculates the AlignedBox3 bounds of the ellipsoid
       * @return 3D axis aligned bounding box
       */ 
      Vector6 bounds() const;

      /** 
       * Returns a quadric with noise added to the pose and radii
       */
      ConstrainedDualQuadric addNoise(double sd);

      /// @}
      /// @name Manifold group traits
      /// @{
      enum { dimension = 9 };

      /**
       * The Retract at origin
       */
      static ConstrainedDualQuadric Retract(const Vector9& v);

      /**
       * The Local at origin
       */
      static Vector9 LocalCoordinates(const ConstrainedDualQuadric& q);


      /**
       * The retract function
       * @param v vector in tangent space (change in latent dims)
       * @return dual ellipsoid on the manifold
       */
      ConstrainedDualQuadric retract(const Vector9& v) const;

      /**
       * The local function
       * @param dE dual ellipsoid on the manifold
       * @return vector between ellipsoids in tangent space
       */
      Vector9 localCoordinates(const ConstrainedDualQuadric& other) const;

      /// @}
      /// @name Testable group traits
      /// @{
        
      /** Prints the dual quadric with optional string */
      void print(const std::string& s = "") const;

      /** Compares two ellipsoids */
      bool equals(const ConstrainedDualQuadric& other, double tol = 1e-9) const;
      /// @}
  };

  // add ConstrainedDualQuadric to Manifold group
  template <>
  struct traits<ConstrainedDualQuadric> : public internal::Manifold<ConstrainedDualQuadric> {};

  template <>
  struct traits<const ConstrainedDualQuadric> : public internal::Manifold<ConstrainedDualQuadric> {};
    
} // namespace gtsam
