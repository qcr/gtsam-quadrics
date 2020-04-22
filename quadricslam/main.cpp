/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file main.cpp
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief a simple main function to facilitate debugging
 */

#include <quadricslam/geometry/ConstrainedDualQuadric.h>
#include <quadricslam/geometry/BoundingBoxFactor.h>
#include <quadricslam/geometry/QuadricCamera.h>
#include <quadricslam/geometry/AlignedBox2.h>
#include <quadricslam/geometry/DualConic.h>
#include <quadricslam/base/TestClass.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;

int main() {

  ConstrainedDualQuadric x; 
  AlignedBox2 box(1.2, 3.4, 5.6, 7.8);
  TestClass t(5);

  // // power of a matrix
  // Matrix22 z = (Matrix22() << 1.0,2.0,3.0,4.0).finished();
  // print((Matrix)z, "z:");
  // print((Matrix)z.array().pow(2).matrix(), "z.array():");

  // create measurement, calibration, dimensions, keys, model, pose, quadric
  AlignedBox2 measured(15.2, 18.5, 120.5, 230.2);
  boost::shared_ptr<Cal3_S2> calibration(new Cal3_S2(525.0, 525.0, 0.0, 320.0, 240.0));
  boost::shared_ptr<Vector2> imageDimensions(new Vector2(320.0, 240.0));
  Key poseKey(Symbol('x', 1));
  Key quadricKey(Symbol('q', 1));
  boost::shared_ptr<noiseModel::Diagonal> model = noiseModel::Diagonal::Sigmas(Vector4(0.2,0.2,0.2,0.2));

  // set camera pose and quadric
  Pose3 cameraPose(Rot3(), Point3(0,0,-3));
  ConstrainedDualQuadric quadric;

  // create and use bbf
  BoundingBoxFactor bbf(measured, calibration, imageDimensions, poseKey, quadricKey, model);
  // Vector4 error = bbf.evaluateError(cameraPose, quadric);

  // calculate prediction by hand
  QuadricCamera camera(cameraPose, calibration);
  Matrix P = camera.transformToImage();
  Matrix dC1 = P * quadric.matrix() * P.transpose();
  DualConic dC = camera.project(quadric);
  AlignedBox2 predictedBounds = dC.bounds();

  // print by hand
  cout << "quadric matrix\n" << quadric.matrix() << endl;
  cout << "pose matrix\n" << cameraPose.matrix() << endl;
  cout << "P matrix\n" << P << endl;
  cout << "C1 matrix\n" << dC1 << endl;
  cout << "conic matrix\n" << dC.matrix() << endl;
  predictedBounds.print("By hand");

  // check expression has same result
  Expression<Pose3> cameraPose_('x',1);
  Expression<ConstrainedDualQuadric> quadric_('q',1);
  Values values;
  values.insert(symbol('x',1), cameraPose);
  values.insert(symbol('q',1), quadric);
  Expression<AlignedBox2> bbfExpression = bbf.expression(cameraPose_, quadric_);
  AlignedBox2 result = bbfExpression.value(values);
  result.print("expression");


  cout << "done" << endl;
  return 1;
}

