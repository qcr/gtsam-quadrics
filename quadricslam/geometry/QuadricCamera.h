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

  /**
   * @class QuadricCamera
   * A camera that projects quadrics
   */
  class GTSAM_EXPORT QuadricCamera : public PinholePose<Cal3_S2> {

    private:

      typedef PinholePose<Cal3_S2> Base; ///< base class has pose and calibration as private member

    public:

      /// @name Constructors and named constructors
      /// @{
        
      /** Default constructor */
      QuadricCamera() {};

      /** Constructor with pose and calibration */
      QuadricCamera(const Pose3& pose, const boost::shared_ptr<Cal3_S2>& K) : Base(pose, K) {};

      /** 
       * Named static constructor for Expressions 
       * as found in PinholeCamera.h
      */
      static QuadricCamera Create(const Pose3& pose, const boost::shared_ptr<Cal3_S2>& K, OptionalJacobian<6,6> dCamera_dPose, OptionalJacobian<6,5> dCamera_dCalibration);

      /// @}
      /// @name Class methods
      /// @{

      /** Calculate the 3x4 projection matrix */
      Matrix34 transformToImage(OptionalJacobian<12,6> dP_dCamera = boost::none) const;

      /** Static projection matrix */
      static Matrix34 transformToImage(const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration);

      /** Static projection function */
      static DualConic project(const ConstrainedDualQuadric& quadric, const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration, 
        OptionalJacobian<9,9> dc_dq = boost::none, OptionalJacobian<9,6> dc_dx = boost::none, OptionalJacobian<9,5> dc_dk = boost::none);

      /** Project box to planes */
      static std::vector<Vector4> project(const AlignedBox2& box, const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration);
      
      /**
       * Project a quadric at the stored 3D pose and calibration
       * @param quadric the 3D quadric surface to be projected
       * @return the projected dual conic 
       */
      DualConic project(const ConstrainedDualQuadric& quadric) const;

      /** Matrix version of project for numerical differentiation */
      static Matrix3 project_(const ConstrainedDualQuadric& quadric, const Pose3& pose, const boost::shared_ptr<Cal3_S2>& calibration);
      
      /// @}
  };

  // Add dimensions for expressions
  template<>
  struct traits<QuadricCamera> {
    enum { dimension = 6};
  };  

} // namespace gtsam
