/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file AlignedBox2.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief an axis aligned 2D bounding box
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace gtsam_quadrics {

/**
 * @class AlignedBox2
 * An axis aligned 2D bounding box (xmin,ymin,xmax,ymax)
 */
class AlignedBox2 {
 protected:
  gtsam::Vector4 tlbr_;  ///< xmin,ymin,xmax,ymax

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors and named constructors
  /// @{

  /** Default constructor */
  AlignedBox2() : tlbr_(0, 0, 0, 0){};

  /** Constructor from doubles */
  AlignedBox2(const double& xmin, const double& ymin, const double& xmax,
              const double& ymax)
      : tlbr_(xmin, ymin, xmax, ymax){};

  /**
   * Constructor from vector
   * @param tlbr vector of xmin,ymin,xmax,ymax (Vector4)
   */
  AlignedBox2(const gtsam::Vector4& tlbr) : tlbr_(tlbr){};

  /// @}
  /// @name Class accessors
  /// @{

  /** Get xmin */
  double xmin() const { return tlbr_[0]; }

  /** Get ymin */
  double ymin() const { return tlbr_[1]; }

  /** Get xmax */
  double xmax() const { return tlbr_[2]; }

  /** Get ymax */
  double ymax() const { return tlbr_[3]; }

  /** Returns box in xmin,ymin,xmax,ymax vector */
  gtsam::Vector4 vector() const { return tlbr_; };

  /** Returns Point2(xmin, ymin) */
  gtsam::Point2 minPoint() const { return gtsam::Vector2(xmin(), ymin()); }

  /** Returns Point2(xmax, ymax) */
  gtsam::Point2 maxPoint() const { return gtsam::Vector2(xmax(), ymax()); }

  /** Returns box centre */
  gtsam::Point2 center() const {
    return gtsam::Vector2((xmin() + xmax()) / 2, (ymin() + ymax()) / 2);
  }

  /** Returns box width */
  double width() const { return std::abs(xmax() - xmin()); }

  /** Returns box height */
  double height() const { return std::abs(ymax() - ymin()); }

  /// @}
  /// @name Class methods
  /// @{

  /** Returns equation of boxes lines */
  std::vector<gtsam::Vector3> lines() const;

  /**
   * Returns true if this contains the point
   * Points intersecting a line are considered containing
   */
  bool contains(const gtsam::Point2& point) const;

  /**
   * Returns true if this completely contains other box
   * Edges touching are considered contained
   */
  bool contains(const AlignedBox2& other) const;

  /**
   * Returns true if this intersects other box
   * Edges touching are considered not intersecting
   * NOTE: assumes xmin < xmax, ymin < ymax
   */
  bool intersects(const AlignedBox2& other) const;

  /**
   * Calculates the standard intersection over union
   * between two axis aligned bounding boxes.
   */
  double iou(const AlignedBox2& other) const;

  /// @}
  /// @name Testable group traits
  /// @{

  /** Prints the box vector with optional string */
  void print(const std::string& s = "") const;

  /** Compares two boxes */
  bool equals(const AlignedBox2& other, double tol = 1e-9) const;

  /// @}
};

// Add vector<> typedef for python wrapper
typedef std::vector<gtsam::Vector3> Vector3Vector;

}  // namespace gtsam_quadrics

/** \cond PRIVATE */
// Add to testable group
template <>
struct gtsam::traits<gtsam_quadrics::AlignedBox2>
    : public gtsam::Testable<gtsam_quadrics::AlignedBox2> {};
/** \endcond */
