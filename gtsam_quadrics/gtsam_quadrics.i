/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file gtsam_quadrics.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief this file provides the interface for gtsams python wrapping tool
 */

/* -------------------------------------
  NOTES:
    - cannot have comments after gtsam namespace at eof (i.e, // namespace
  gtsam)
    - remove Matrix/Vector dimensions from input/output
    - remove (void) from input arguments
    - forward declare return type classes for gtsam objects
    - show inheritence with virtual + forward declare virtual
    - for default arguments expose the method twice
    - wrap std::vector<class> with typedef and python class, ensure gtsam::class
    - for std::string remove std::
    - const boost::shared_ptr<X>& -> const X*
    - const SharedNoiseModel& -> const gtsam::noiseModel::Base*
    - Key -> size_t

  RESTRICTIONS:
    - cant used boost::shared_ptr<eigenobject>

  QUESTIONS:
    - how to wrap optional jacobians?
    - how to wrap operator[] ?
    - how to wrap methods to belong inside a namespace?
 */

namespace gtsam_quadrics {

#include <gtsam_quadrics/base/Utilities.h>
namespace utils {
gtsam::Pose3 interpolate(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
                         const double& percent);
}  // namespace utils

#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
class ConstrainedDualQuadric {
  ConstrainedDualQuadric();
  ConstrainedDualQuadric(const gtsam::Matrix& dQ);
  ConstrainedDualQuadric(const gtsam::Pose3& pose, const gtsam::Vector& radii);
  ConstrainedDualQuadric(const gtsam::Rot3& R, const gtsam::Point3& t,
                         const gtsam::Vector& r);
  gtsam::Pose3 pose() const;
  gtsam::Vector radii() const;
  gtsam::Point3 centroid() const;
  gtsam::Matrix matrix() const;
  static ConstrainedDualQuadric constrain(const gtsam::Matrix& dual_quadric);
  AlignedBox3 bounds() const;
  bool isBehind(const gtsam::Pose3& cameraPose) const;
  bool contains(const gtsam::Pose3& cameraPose) const;
  static ConstrainedDualQuadric Retract(const gtsam::Vector& v);
  static gtsam::Vector LocalCoordinates(
      const gtsam_quadrics::ConstrainedDualQuadric& q);
  gtsam::ConstrainedDualQuadric retract(const gtsam::Vector& v) const;
  Vector localCoordinates(
      const gtsam_quadrics::ConstrainedDualQuadric& other) const;
  void addToValues(gtsam::Values& v, const size_t& k);
  static ConstrainedDualQuadric getFromValues(const gtsam::Values& v,
                                              const size_t& k);
  void print(const string& s) const;
  void print() const;
  bool equals(const gtsam_quadrics::ConstrainedDualQuadric& other,
              double tol) const;
  bool equals(const gtsam_quadrics::ConstrainedDualQuadric& other) const;
};

#include <gtsam_quadrics/geometry/BoundingBoxFactor.h>
virtual class BoundingBoxFactor : gtsam::NonlinearFactor {
  BoundingBoxFactor();
  BoundingBoxFactor(const gtsam_quadrics::AlignedBox2& measured,
                    const gtsam::Cal3_S2* calibration, const size_t& poseKey,
                    const size_t& quadricKey,
                    const gtsam::noiseModel::Base* model);
  BoundingBoxFactor(const gtsam_quadrics::AlignedBox2& measured,
                    const gtsam::Cal3_S2* calibration, const size_t& poseKey,
                    const size_t& quadricKey,
                    const gtsam::noiseModel::Base* model,
                    const string& errorString);
  AlignedBox2 measurement() const;
  size_t poseKey() const;
  size_t objectKey() const;
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      const gtsam_quadrics::ConstrainedDualQuadric& quadric) const;
  gtsam::Matrix evaluateH1(
      const gtsam::Pose3& pose,
      const gtsam_quadrics::ConstrainedDualQuadric& quadric) const;
  gtsam::Matrix evaluateH2(
      const gtsam::Pose3& pose,
      const gtsam_quadrics::ConstrainedDualQuadric& quadric) const;
  gtsam::Matrix evaluateH1(const gtsam::Values& x) const;
  gtsam::Matrix evaluateH2(const gtsam::Values& x) const;
};

#include <gtsam_quadrics/geometry/QuadricAngleFactor.h>
virtual class QuadricAngleFactor {
  QuadricAngleFactor(const size_t& quadricKey, const gtsam::Rot3& measured,
                     const gtsam::noiseModel::Base* model);
  Vector evaluateError(
      const gtsam_quadrics::ConstrainedDualQuadric& quadric) const;
};

// TODO need to figure out how to get this to import...
// template <T = {gtsam_quadrics::ConstrainedDualQuadric}>
// virtual class gtsam::PriorFactor : gtsam::NoiseModelFactor {
//   PriorFactor(size_t key, const T& prior,
//               const gtsam::noiseModel::Base* noiseModel);
//   T prior() const;

//   // enabling serialization functionality
//   void serialize() const;
// };

#include <gtsam_quadrics/geometry/AlignedBox2.h>
class Vector3Vector {
  Vector3Vector();
  size_t size() const;
  bool empty() const;
  gtsam::Vector at(size_t n) const;
  void push_back(const gtsam::Vector& pose);
};

class AlignedBox2 {
  AlignedBox2();
  AlignedBox2(const double& xmin, const double& ymin, const double& xmax,
              const double& ymax);
  AlignedBox2(const gtsam::Vector& tlbr);
  AlignedBox2(const gtsam_quadrics::AlignedBox2& other);
  double xmin() const;
  double ymin() const;
  double xmax() const;
  double ymax() const;
  Vector vector() const;
  Point2 center() const;
  double width() const;
  double height() const;
  bool contains(const gtsam::Point2& point) const;
  bool contains(const gtsam_quadrics::AlignedBox2& other) const;
  bool intersects(const gtsam_quadrics::AlignedBox2& other) const;
  double iou(const gtsam_quadrics::AlignedBox2& other) const;
  Vector3Vector lines() const;
  void print(const string& s) const;
  void print() const;
  bool equals(const gtsam_quadrics::AlignedBox2& other, double tol) const;
  bool equals(const gtsam_quadrics::AlignedBox2& other) const;
};

#include <gtsam_quadrics/geometry/AlignedBox3.h>
class AlignedBox3 {
  AlignedBox3();
  AlignedBox3(const double& xmin, const double& xmax, const double& ymin,
              const double& ymax, const double& zmin, const double& zmax);
  AlignedBox3(const gtsam::Vector& xxyyzz);
  AlignedBox3(const gtsam_quadrics::AlignedBox3& other);
  double xmin() const;
  double xmax() const;
  double ymin() const;
  double ymax() const;
  double zmin() const;
  double zmax() const;
  gtsam::Vector vector() const;
  gtsam::Vector dimensions() const;
  gtsam::Vector centroid() const;
  double volume() const;
  double iou(const gtsam_quadrics::AlignedBox3& other) const;
  void print(const string& s) const;
  void print() const;
  bool equals(const gtsam_quadrics::AlignedBox3& other, double tol) const;
  bool equals(const gtsam_quadrics::AlignedBox3& other) const;
};

#include <gtsam_quadrics/geometry/DualConic.h>
class DualConic {
  DualConic();
  DualConic(const gtsam::Matrix& dC);
  DualConic(const gtsam::Pose2& pose, const gtsam::Vector& radii);
  gtsam::Matrix matrix() const;
  gtsam::AlignedBox2 bounds() const;
  bool isDegenerate() const;
  bool isEllipse() const;
};

#include <gtsam_quadrics/geometry/QuadricCamera.h>
class QuadricCamera {
  static Matrix transformToImage(const gtsam::Pose3& pose,
                                 const gtsam::Cal3_S2* calibration);
  static gtsam_quadrics::DualConic project(
      const gtsam_quadrics::ConstrainedDualQuadric& quadric,
      const gtsam::Pose3& pose, const gtsam::Cal3_S2* calibration);
};

}  // namespace gtsam_quadrics
