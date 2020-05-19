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
   * AlignedBox3 factor between pose3 and ConstrainedDualQuadric, constrains quadric by 
   * projecting quadric into image plane at each view
   */
  class BoundingBoxFactor : public NoiseModelFactor2<Pose3, ConstrainedDualQuadric> {

    protected:
      AlignedBox2 measured_; ///< measured bounding box
      boost::shared_ptr<Cal3_S2> calibration_; ///< camera calibration
      Vector2 imageDimensions_; ///< image dimensions (width, height)
      typedef NoiseModelFactor2<Pose3, ConstrainedDualQuadric> Base; ///< base class has keys and noisemodel as private members

    public:

      /** Default constructor */
      BoundingBoxFactor() {};

      /** Constructor from measured box, calbration, dimensions and posekey, quadrickey, noisemodel */
      BoundingBoxFactor(const AlignedBox2& measured, const boost::shared_ptr<Cal3_S2>& calibration, 
        const Vector2& imageDimensions, const Key& poseKey, const Key& quadricKey, 
        const SharedNoiseModel& model) : 
          Base(model, poseKey, quadricKey), measured_(measured), 
          calibration_(calibration), imageDimensions_(imageDimensions) {};

      /** Get key1 from base */
      AlignedBox2 measurement() const { return AlignedBox2(measured_.vector());}

      /** Get key1 from base */
      Key poseKey() const { return key1();}

      /** Get key2 from base */
      Key objectKey() const { return key2();}

      /**
       * Evaluate the error between a quadric and 3D pose
       * @param pose the 3D camera position
       * @param quadric the quadric
       * @param H1 the derivative of the error wrt pose (4x6)
       * @param H2 the derivative of the error wrt quadric (4x9)
       */
      Vector evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric,
			  boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none) const;      

      /** Add to graph */
      void addToGraph(NonlinearFactorGraph& graph);

      /** Get from graph */
      static BoundingBoxFactor getFromGraph(const NonlinearFactorGraph& graph, size_t idx);

      /** Prints the dual quadric with optional string */
      void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

      /** Compares two ellipsoids */
      bool equals(const BoundingBoxFactor& other, double tol = 1e-9) const;
  };

  // declare dimensions of calibration pointer for expressions
  template <>
  struct traits<boost::shared_ptr<Cal3_S2>> {
    enum { dimension = 5};
  };

} // namespace gtsam
