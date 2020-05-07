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
    - size_t -> size_t

  RESTRICTIONS:
    - cant used boost::shared_ptr<eigenobject>

  QUESTIONS:
    - how to wrap optional jacobians?
    - how to wrap operator[] ?
 */



class Pose3;
class Point3;
virtual class NoiseModelFactor;
class ConstrainedDualQuadric;
class AlignedBox2;


namespace gtsam {

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
  Vector bounds() const;
  ConstrainedDualQuadric addNoise(const Vector& noiseVector);
  static ConstrainedDualQuadric Retract(const Vector& v);
  static Vector LocalCoordinates(const ConstrainedDualQuadric& q);
  ConstrainedDualQuadric retract(const Vector& v) const;
  Vector localCoordinates(const ConstrainedDualQuadric& other) const;
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
  Vector evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric) const;
  Vector evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric,
    Matrix& H1, Matrix& H2) const;
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
  Vector vector() const;
  gtsam::Vector3Vector lines() const;
  AlignedBox2 addNoise(const Vector& noiseVector);
  void print(const string& s) const;
  void print() const;
  bool equals(const AlignedBox2& other, double tol) const;
  bool equals(const AlignedBox2& other) const;
};

}
