/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file quadricslam.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief this file provides the interface for gtsams python wrapping tool
 */

/* -------------------------------------
  NOTES: 
    - cannot have comments after gtsam namespace at eof (i.e, // namespace gtsam)
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



virtual class NoiseModelFactor;
class NonlinearFactorGraph;
class Cal3_S2;
class Point2;
class Point3;
class Values;
class Pose3;

class ConstrainedDualQuadric;
class AlignedBox2;
class AlignedBox3;
class DualConic;


namespace gtsam {

#include <quadricslam/base/Utilities.h>
namespace utils {
  Pose3 interpolate(const Pose3& p1, const Pose3& p2, const double& percent);  
}

#include <quadricslam/geometry/ConstrainedDualQuadric.h>
class ConstrainedDualQuadric {
  ConstrainedDualQuadric();
  ConstrainedDualQuadric(const Matrix& dQ);
  ConstrainedDualQuadric(const Pose3& pose, const Vector& radii);
  ConstrainedDualQuadric(const Rot3& R, const Point3& t, const Vector& r);
  Pose3 pose() const;
  Vector radii() const;
  Point3 centroid() const;
  Matrix matrix() const;
  static ConstrainedDualQuadric constrain(const Matrix& dual_quadric);
  gtsam::AlignedBox3 bounds() const;
  bool isBehind(const Pose3& cameraPose) const;
  bool contains(const Pose3& cameraPose) const;
  static ConstrainedDualQuadric Retract(const Vector& v);
  static Vector LocalCoordinates(const ConstrainedDualQuadric& q);
  gtsam::ConstrainedDualQuadric retract(const Vector& v) const;
  Vector localCoordinates(const ConstrainedDualQuadric& other) const;
  void addToValues(Values &v, const size_t& k);
  static ConstrainedDualQuadric getFromValues(const Values &v, const size_t& k);
  void print(const string& s) const;
  void print() const;
  bool equals(const ConstrainedDualQuadric& other, double tol) const;
  bool equals(const ConstrainedDualQuadric& other) const;
};

#include <quadricslam/geometry/BoundingBoxFactor.h>
virtual class BoundingBoxFactor : NoiseModelFactor {
  BoundingBoxFactor();
  BoundingBoxFactor(const AlignedBox2& measured, const Cal3_S2* calibration,
    const size_t& poseKey, const size_t& quadricKey, const gtsam::noiseModel::Base* model);
  gtsam::AlignedBox2 measurement() const;
  size_t poseKey() const;
  size_t objectKey() const;
  Vector evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric) const;
  Matrix evaluateH1(const Pose3& pose, const ConstrainedDualQuadric& quadric) const;
  Matrix evaluateH2(const Pose3& pose, const ConstrainedDualQuadric& quadric) const;
  void addToGraph(NonlinearFactorGraph& graph);
  static gtsam::BoundingBoxFactor getFromGraph(const NonlinearFactorGraph& graph, size_t idx);
};

#include <quadricslam/geometry/AlignedBox2.h>
class Vector3Vector 
{
  Vector3Vector();
  size_t size() const;
  bool empty() const;
  Vector at(size_t n) const;
  void push_back(const Vector& pose);
};

class AlignedBox2 {
  AlignedBox2();
  AlignedBox2(const double& xmin, const double& ymin, const double& xmax, const double& ymax);
  AlignedBox2(const Vector& tlbr);
  AlignedBox2(const AlignedBox2& other);
  double xmin() const;
  double ymin() const;
  double xmax() const;
  double ymax() const;
  Vector vector() const;
  double width() const;
  double height() const;
  bool contains(const Point2& point) const;
  bool contains(const AlignedBox2& other) const;
  bool intersects(const AlignedBox2& other) const;
  gtsam::Vector3Vector lines() const;
  void print(const string& s) const;
  void print() const;
  bool equals(const AlignedBox2& other, double tol) const;
  bool equals(const AlignedBox2& other) const;
};

#include <quadricslam/geometry/AlignedBox3.h>
class AlignedBox3 {
  AlignedBox3();
  AlignedBox3(const double& xmin, const double& xmax, const double& ymin, const double& ymax, const double& zmin, const double& zmax);
  AlignedBox3(const Vector& xxyyzz);
  AlignedBox3(const AlignedBox3& other);
  double xmin() const;
  double xmax() const;
  double ymin() const;
  double ymax() const;
  double zmin() const;
  double zmax() const;
  Vector vector() const;
  Vector dimensions() const;
  Vector centroid() const;
  void print(const string& s) const;
  void print() const;
  bool equals(const AlignedBox3& other, double tol) const;
  bool equals(const AlignedBox3& other) const;
};

#include <quadricslam/geometry/DualConic.h>
class DualConic {
  DualConic();
  DualConic(const Matrix& dC);
  DualConic(const Pose2& pose, const Vector& radii);
  Matrix matrix() const;
  gtsam::AlignedBox2 bounds() const;
  bool isDegenerate() const;
  bool isEllipse() const;
};

#include <quadricslam/geometry/QuadricCamera.h>
class QuadricCamera {
  static Matrix transformToImage(const Pose3& pose, const Cal3_S2* calibration);
  static gtsam::DualConic project(const ConstrainedDualQuadric& quadric, const Pose3& pose, const Cal3_S2* calibration);
};


}
