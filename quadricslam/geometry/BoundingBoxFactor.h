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
      boost::shared_ptr<Vector2> imageDimensions_; ///< camera calibration
      typedef NoiseModelFactor2<Pose3, ConstrainedDualQuadric> Base; ///< base class has keys and noisemodel as private members

    public:

      /// @name Constructors and named constructors
      /// @{  

      /** default constructor */
      BoundingBoxFactor() {};

      /** constructor from measured box, calbration, dimensions and posekey, quadrickey, noisemodel */
      BoundingBoxFactor(const AlignedBox2& measured, const boost::shared_ptr<Cal3_S2>& calibration, 
        const boost::shared_ptr<Vector2>& imageDimensions, const Key& poseKey, const Key& quadricKey, 
        const SharedNoiseModel& model) : 
          Base(model, poseKey, quadricKey), measured_(measured), 
          calibration_(calibration), imageDimensions_(imageDimensions) {};

      /// @}

      Vector evaluateError(const Pose3 &pose, const ConstrainedDualQuadric &quadric,
			  boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none) const;

  };

} // namespace gtsam
