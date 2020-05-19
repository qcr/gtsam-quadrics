/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file QuadricCamera.h
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief a class responsible for projecting quadrics
 */


#pragma once

#include <quadricslam/geometry/ConstrainedDualQuadric.h>
#include <quadricslam/geometry/DualConic.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/base/types.h>

namespace gtsam {

  /**
   * @class QuadricCamera
   * A camera that projects quadrics
   */
  class QuadricCamera {

    public:

      /// @}
      /// @name Class methods
      /// @{

      /** Static projection matrix */
      static Matrix34 transformToImage(const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration);

      /**
       * Project a quadric at the stored 3D pose and calibration
       * @param quadric the 3D quadric surface to be projected
       * @return the projected dual conic 
       */
      static DualConic project(const ConstrainedDualQuadric& quadric, const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration, 
        OptionalJacobian<9,9> dC_dq = boost::none, OptionalJacobian<9,6> dC_dx = boost::none);

      /** Project box to planes */
      static std::vector<Vector4> project(const AlignedBox2& box, const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration);
      
      /// @}
  };

} // namespace gtsam
