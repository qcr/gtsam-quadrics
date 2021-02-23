/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file QuadricAngleFactor.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

#pragma once

#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <gtsam_quadrics/geometry/AlignedBox2.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

  /**
   * @class QuadricAngleFactor
   * AlignedBox3 factor between Pose3 and ConstrainedDualQuadric
   * Projects the quadric at the current pose estimates,
   * Calculates the bounds of the dual conic, 
   * and compares this to the measured bounding box.
   */
  class QuadricAngleFactor : public NoiseModelFactor1<ConstrainedDualQuadric> {
      
    protected:
      typedef NoiseModelFactor1<ConstrainedDualQuadric> Base; ///< base class has keys and noisemodel as private members

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /// @name Constructors and named constructors
      /// @{

      /** Constructor from measured box, calbration, dimensions and posekey, quadrickey, noisemodel */
      QuadricAngleFactor(const Key& quadricKey, const SharedNoiseModel& model): 
        Base(model, quadricKey) { };

      /// @}
      /// @name Class methods
      /// @{

      /**
       * Evaluate the error between a quadric and 3D pose
       * @param pose the 6DOF camera position
       * @param quadric the constrained dual quadric
       * @param H1 the derivative of the error wrt camera pose (4x6)
       * @param H2 the derivative of the error wrt quadric (4x9)
       */
      Vector evaluateError(const ConstrainedDualQuadric& quadric,
			  boost::optional<Matrix &> H = boost::none) const;      

      /// @}
      /// @name Testable group traits
      /// @{

      /** Prints the boundingbox factor with optional string */
      void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

      /** Returns true if equal keys, measurement, noisemodel and calibration */
      bool equals(const QuadricAngleFactor& other, double tol = 1e-9) const;
  };

  /** \cond PRIVATE */
  // Add to testable group 
  template <>
  struct traits<QuadricAngleFactor> : public Testable<QuadricAngleFactor> {};
  /** \endcond */

} // namespace gtsam
