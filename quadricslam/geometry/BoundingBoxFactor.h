/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BoundingBoxFactor.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

#pragma once

#include <quadricslam/geometry/ConstrainedDualQuadric.h>
#include <quadricslam/geometry/AlignedBox2.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

  /**
   * @class BoundingBoxFactor
   * AlignedBox3 factor between Pose3 and ConstrainedDualQuadric
   * Projects the quadric at the current pose estimates,
   * Calculates the bounds of the dual conic, 
   * and compares this to the measured bounding box.
   */
  class BoundingBoxFactor : public NoiseModelFactor2<Pose3, ConstrainedDualQuadric> {

    protected:
      AlignedBox2 measured_; ///< measured bounding box
      boost::shared_ptr<Cal3_S2> calibration_; ///< camera calibration
      typedef NoiseModelFactor2<Pose3, ConstrainedDualQuadric> Base; ///< base class has keys and noisemodel as private members

    public:

      /// @name Constructors and named constructors
      /// @{

      /** Default constructor */
      BoundingBoxFactor() :
        measured_(0.,0.,0.,0.) {};

      /** Constructor from measured box, calbration, dimensions and posekey, quadrickey, noisemodel */
      BoundingBoxFactor(const AlignedBox2& measured, const boost::shared_ptr<Cal3_S2>& calibration, 
      const Key& poseKey, const Key& quadricKey, const SharedNoiseModel& model) : 
          Base(model, poseKey, quadricKey), measured_(measured), calibration_(calibration) {};

      /// @}
      /// @name Class accessors
      /// @{

      /** Returns the measured bounding box */
      AlignedBox2 measurement() const { return AlignedBox2(measured_.vector());}

      /** Returns the pose key */
      Key poseKey() const { return key1();}

      /** Returns the object/landmark key */
      Key objectKey() const { return key2();}

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
      Vector evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric,
			  boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none) const;      

      /** Evaluates the derivative of the error wrt pose */
      Matrix evaluateH1(const Pose3& pose, const ConstrainedDualQuadric& quadric) const;

      /** Evaluates the derivative of the error wrt quadric */
      Matrix evaluateH2(const Pose3& pose, const ConstrainedDualQuadric& quadric) const;

      /** 
       * Add this factor to given graph 
       */
      void addToGraph(NonlinearFactorGraph& graph);

      /** 
       * Retrieve factor from graph at index
       */
      static BoundingBoxFactor getFromGraph(const NonlinearFactorGraph& graph, size_t idx);

      /// @}
      /// @name Testable group traits
      /// @{

      /** Prints the boundingbox factor with optional string */
      void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

      /** Returns true if equal keys, measurement, noisemodel and calibration */
      bool equals(const BoundingBoxFactor& other, double tol = 1e-9) const;
  };

  // Add to testable group 
  template <>
  struct traits<BoundingBoxFactor> : public Testable<BoundingBoxFactor> {};

} // namespace gtsam
