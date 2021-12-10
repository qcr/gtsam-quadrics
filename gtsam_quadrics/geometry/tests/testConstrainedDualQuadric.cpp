/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testConstrainedDualQuadric.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief test cases for ConstrainedDualQuadric
 */


#include <CppUnitLite/TestHarness.h>

#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <gtsam_quadrics/geometry/DualConic.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>

using namespace std;
using namespace gtsam_quadrics;

TEST(ConstrainedDualQuadric, Constructors) {
  ConstrainedDualQuadric q1;
  ConstrainedDualQuadric q2(Pose3(), Vector3(0.9,1.0,1.1));
  ConstrainedDualQuadric q3(Rot3(), Point3(), Vector3(0.9,1.0,1.1));
  ConstrainedDualQuadric q4(q3);

  Matrix44 expected = (Matrix44() <<  0.9*0.9, 0.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0, 0.0,
                                      0.0, 0.0, 1.1*1.1, 0.0,
                                      0.0, 0.0, 0.0, -1.0).finished();
  Matrix44 expectedDefault = (Matrix44() <<  1.0, 0.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0,
                                            0.0, 0.0, 0.0, -1.0).finished();

  EXPECT(assert_equal(expectedDefault, q1.matrix()));
  EXPECT(assert_equal(expected, q2.matrix()));
  EXPECT(assert_equal(expected, q3.matrix()));
  EXPECT(assert_equal(expected, q4.matrix()));
}

/// TODO: test generic dual quadric
TEST(ConstrainedDualQuadric, Constrain) {
  Matrix44 defaultQuadric = (Matrix44() <<  1.0, 0.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0,
                                            0.0, 0.0, 0.0, -1.0).finished();
  Matrix44 constrainedQuadric = (Matrix44() <<  0.9*0.9, 0.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0, 0.0,
                                      0.0, 0.0, 1.1*1.1, 0.0,
                                      0.0, 0.0, 0.0, -1.0).finished();

  Matrix44 actual1 = ConstrainedDualQuadric::constrain(defaultQuadric).matrix();
  Matrix44 actual2 = ConstrainedDualQuadric::constrain(constrainedQuadric).matrix();
  EXPECT(assert_equal(defaultQuadric, actual1));
  EXPECT(assert_equal(constrainedQuadric, actual2));
}

TEST(ConstrainedDualQuadric, Accessors) {
  Point3 expectedPoint(0.2,0.3,0.4);
  Rot3 expectedRot = Rot3::Rodrigues(0.1,0.2,0.3);
  Pose3 expectedPose(expectedRot, expectedPoint);
  Vector3 expectedRadii(0.3,0.4,0.5);
  ConstrainedDualQuadric q1(expectedPose, expectedRadii);

  EXPECT(assert_equal(expectedPose, q1.pose()));
  EXPECT(assert_equal(expectedRadii, q1.radii()));
  EXPECT(assert_equal(expectedRot, q1.pose().rotation()));
  EXPECT(assert_equal(expectedPoint, q1.centroid()));
}

TEST(ConstrainedDualQuadric, Matrix) {
  Rot3 R = Rot3::Rodrigues(1.,2.,3.);
  Point3 t(3.,4.,5.);
  Vector3 s(0.1,0.2,0.3);
  ConstrainedDualQuadric q(R, t, s);

  Matrix44 expected = Pose3(R,t).matrix() * (Vector4() << (s).array().pow(2), -1.0).finished().asDiagonal() * Pose3(R,t).matrix().transpose();
  EXPECT(assert_equal(expected, q.matrix()));
};

TEST(ConstrainedDualQuadric, NormalizedMatrix) {
  ConstrainedDualQuadric q1 = ConstrainedDualQuadric::Retract((Vector9()<<1.,2.,3.,4.,5.,6.,7.,8.,9.).finished());

  Matrix44 expected = q1.matrix() / q1.matrix()(3,3);
  EXPECT(assert_equal(expected, q1.normalizedMatrix()));
};

TEST(ConstrainedDualQuadric, Bounds) {
  ConstrainedDualQuadric Q(Pose3(Rot3(), Point3(2,3,-4.1)), Vector3(0.9,1.0,1.1));

  AlignedBox3 expected = AlignedBox3(1.1, 2.9, 2.0, 4.0, -5.2, -3.0);
  AlignedBox3 actual = Q.bounds();

  EXPECT(assert_equal(expected, actual));
}

TEST(ConstrainedDualQuadric, IsBehind) {
  ConstrainedDualQuadric Q(Rot3(), Point3(0.,0.,20.), Vector3(1.,2.,3.));
  Pose3 qInfront(Rot3(), Point3(0.,0.,3.));
  Pose3 qBehind(Rot3(), Point3(0.,0.,30.));

  EXPECT(Q.isBehind(qInfront) == false);
  EXPECT(Q.isBehind(qBehind) == true);
}

TEST(ConstrainedDualQuadric, Contains) {
  ConstrainedDualQuadric Q;
  Pose3 inside(Rot3(), Point3(0.1,0.1,0.1));
  Pose3 outside(Rot3(), Point3(3,3,3));
  Pose3 edge(Rot3(), Point3(0.,0.,1.0));

  EXPECT(Q.contains(inside) == true)
  EXPECT(Q.contains(outside) == false)
  EXPECT(Q.contains(edge) == true)
}

TEST(ConstrainedDualQuadric, RetractAndLocal) {
  ConstrainedDualQuadric q1 = ConstrainedDualQuadric::Retract((Vector9()<<1.,2.,3.,4.,5.,6.,7.,8.,9.).finished());
  ConstrainedDualQuadric q2 = ConstrainedDualQuadric::Retract((Vector9()<<3.,4.,5.,6.,7.,8.,9.,10.,11.).finished());
  Pose3 expectedPose = Pose3::Retract((Vector6()<<1.,2.,3.,4.,5.,6.).finished());
  Vector3 expectedRadii(7.,8.,9.);

  Vector6 expectedLocalPose = Pose3::Retract((Vector6()<<1.,2.,3.,4.,5.,6.).finished()).localCoordinates(Pose3::Retract((Vector6()<<3.,4.,5.,6.,7.,8.).finished()));
  Vector3 expectedLocalRadii = Vector3(2.,2.,2.);
  Vector9 expectedLocal; expectedLocal << expectedLocalPose, expectedLocalRadii;

  EXPECT(assert_equal(expectedPose, q1.pose()));
  EXPECT(assert_equal(expectedRadii, q1.radii()));
  EXPECT(assert_equal(expectedLocal, q1.localCoordinates(q2)));
  EXPECT(assert_equal(q2, q1.retract(expectedLocal)));
}

TEST(ConstrainedDualQuadric, AddGetFromValues) {
  Values values;
  ConstrainedDualQuadric Q(Pose3(Rot3(), Point3(2,3,-4.1)), Vector3(0.9,1.0,1.1));
  values.insert(1, Q);
  ConstrainedDualQuadric Q2 = values.at<ConstrainedDualQuadric>(1);
  EXPECT(assert_equal(Q, Q2));
}

TEST(ConstrainedDualQuadric, ExplicitAddGetFromValues) {
  Values values;
  ConstrainedDualQuadric Q(Pose3(Rot3(), Point3(2,3,-4.1)), Vector3(0.9,1.0,1.1));
  Q.addToValues(values, 1);
  ConstrainedDualQuadric Q2 = ConstrainedDualQuadric::getFromValues(values, 1);
  EXPECT(assert_equal(Q, Q2));
}

TEST(ConstrainedDualQuadric, Equals) {
  ConstrainedDualQuadric q1 = ConstrainedDualQuadric::Retract((Vector9()<<1.,2.,3.,4.,5.,6.,7.,8.,9.).finished());
  ConstrainedDualQuadric q2 = ConstrainedDualQuadric::Retract((Vector9()<<2.,3.,4.,5.,6.,7.,8.,9.,10.).finished());

  EXPECT(q1.equals(q1));
  EXPECT(!q1.equals(q2));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
