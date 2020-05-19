/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
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
 */



class Point2;
class Pose3;
class Point3;
class Values;
class Cal3_S2;
class NonlinearFactorGraph;
virtual class NoiseModelFactor;

class ConstrainedDualQuadric;
class AlignedBox2;
class AlignedBox3;
class DualConic;


namespace gtsam {

#include <quadricslam/base/Noise.h>
class Noise {
  static Pose3 interpolate(const Pose3& p1, const Pose3& p2, const double& percent);  
};

#include <quadricslam/base/TestClass.h>
class TestClass {
  TestClass();
  TestClass(double x);
  double getx();
  double doWork(double y);
};

#include <quadricslam/geometry/ConstrainedDualQuadric.h>
class ConstrainedDualQuadric {
  ConstrainedDualQuadric();
  ConstrainedDualQuadric(const Matrix& dQ);
  ConstrainedDualQuadric(const Pose3& pose, const Vector& radii);
  ConstrainedDualQuadric(const Rot3& R, const Point3& t, const Vector& r);
  Pose3 getPose() const;
  Vector getRadii() const;
  Point3 centroid() const;
  Matrix matrix() const;
  static ConstrainedDualQuadric constrain(const Matrix& dual_quadric);
  Vector bounds() const;
  gtsam::ConstrainedDualQuadric addNoise(const Vector& noiseVector);
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
    const Vector& imageDimensions, const size_t& poseKey, const size_t& quadricKey, 
    const gtsam::noiseModel::Base* model);
  gtsam::AlignedBox2 measurement() const;
  size_t poseKey() const;
  size_t objectKey() const;
  Vector evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric) const;
  Vector evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric,
    Matrix& H1, Matrix& H2) const;
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
  double xmin() const;
  double ymin() const;
  double xmax() const;
  double ymax() const;
  Vector vector() const;
  bool contains(const Point2& point) const;
  bool completelyContains(const AlignedBox2& other) const;
  bool contains(const AlignedBox2& other) const;
  bool intersects(const AlignedBox2& other) const;
  gtsam::Vector3Vector lines() const;
  void print(const string& s) const;
  void print() const;
  bool equals(const AlignedBox2& other, double tol) const;
  bool equals(const AlignedBox2& other) const;
};

#include <quadricslam/geometry/DualConic.h>
class DualConic {
  DualConic();
  DualConic(const Matrix& dC);
  DualConic(const Pose2& pose, const Vector& radii);
  Matrix matrix() const;
  gtsam::AlignedBox2 bounds() const;
  gtsam::AlignedBox2 smartBounds(const Cal3_S2* calibration) const;
  bool isDegenerate() const;
  bool isEllipse() const;
};

#include <quadricslam/geometry/QuadricCamera.h>
class QuadricCamera {
  static Matrix transformToImage(const Pose3& pose, const Cal3_S2* calibration);
  static gtsam::DualConic project(const ConstrainedDualQuadric& quadric, const Pose3& pose, const Cal3_S2* calibration);
};



#include <quadricslam/geometry/AlignedBox3.h>
class AlignedBox3 {
  AlignedBox3();
  AlignedBox3(const double& xmin, const double& xmax, const double& ymin, const double& ymax, const double& zmin, const double& zmax);
  AlignedBox3(const Vector& xxyyzz);
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

}
