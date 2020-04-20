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
   * @class QuadricProjectionException
   * Exception thrown when attemption to calculate quadric bounding box fails
   */
  class GTSAM_EXPORT QuadricProjectionException: public ThreadsafeException<QuadricProjectionException> {
    public:
      QuadricProjectionException()
        : QuadricProjectionException(std::numeric_limits<Key>::max()) {}

      QuadricProjectionException(Key j)
        : ThreadsafeException<QuadricProjectionException>("QuadricProjectionException"),
          j_(j) {}

      Key nearbyVariable() const {return j_;}

    private:
      Key j_;
  };


  /**
   * @class QuadricCamera
   * A camera that projects quadrics
   */
  class GTSAM_EXPORT QuadricCamera : public PinholePose<Cal3_S2> {

    private:

      typedef PinholePose<Cal3_S2> Base; ///< basde class has pose and calibration as private member

    public:
    
      /** default constructor */
      QuadricCamera() {};

      /** constructor with pose and calibration */
      QuadricCamera(const Pose3& pose, const boost::shared_ptr<Cal3_S2>& K) : Base(pose, K) {};

      /** named static constructor for Expressions 
       * as found in PinholeCamera.h
      */
      static QuadricCamera Create(const Pose3& pose, const boost::shared_ptr<Cal3_S2>& K, OptionalJacobian<6,6> dCamera_dPose, OptionalJacobian<6,5> dCamera_dCalibration);
      
      /**
       * Calculate the 3x4 projection matrix 
       */
      Matrix34 transformToImage(OptionalJacobian<12,6> dP_dCamera = boost::none) const;
      
      /**
       * Project a quadric at the stored 3D pose and calibration
       * @param quadric the 3D quadric surface to be projected
       * @return the projected dual conic 
       */
      DualConic project(const ConstrainedDualQuadric& quadric, OptionalJacobian<5,6> dC_dCamera = boost::none, OptionalJacobian<5,9> dC_dQ = boost::none) const;

  };

  // add traits<A>::dimension for Expressions
  template<>
  struct traits<QuadricCamera> {
    enum { dimension = 6};
  };  

} // namespace gtsam
