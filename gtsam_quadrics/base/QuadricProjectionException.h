/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file QuadricProjectionException.h
 * @date May 19, 2020
 * @author Lachlan Nicholson
 * @brief An exception to be thrown when projecting a quadric has failed
 */

#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/inference/Key.h>

#include <string>

namespace gtsam_quadrics {

/**
 * @class QuadricProjectionException
 * Exception thrown when attemption to calculate quadric bounding box fails
 */
class QuadricProjectionException
    : public gtsam::ThreadsafeException<QuadricProjectionException> {
 public:
  QuadricProjectionException()
      : QuadricProjectionException(std::numeric_limits<gtsam::Key>::max()) {}

  QuadricProjectionException(gtsam::Key j)
      : ThreadsafeException<QuadricProjectionException>(
            "QuadricProjectionException"),
        j_(j) {}

  QuadricProjectionException(const std::string& description)
      : ThreadsafeException<QuadricProjectionException>(description) {}

  gtsam::Key nearbyVariable() const { return j_; }

 private:
  gtsam::Key j_;
};

}  // namespace gtsam_quadrics
