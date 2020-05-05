/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file System.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief code to operate the quadric slam library on a dataset
 */

#include <quadricslam/base/System.h>

#include <quadricslam/geometry/ConstrainedDualQuadric.h>
#include <quadricslam/geometry/AlignedBox2.h>
#include <quadricslam/geometry/QuadricCamera.h>
#include <quadricslam/geometry/BoundingBoxFactor.h>
#include <quadricslam/base/Dataset.h>
#include <quadricslam/base/Noise.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/slam/BetweenFactor.h>

#include <random>
#include <string>
#include <iostream>

#define ODOM_SD 0.01
#define QUAD_SD 0.1
#define BOX_SD 3

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
Values BackEnd::offline(NonlinearFactorGraph graph, Values initialEstimate, bool testSensitivity) {

  // define parameters
  LevenbergMarquardtParams parameters;
  parameters.setRelativeErrorTol(1e-10); ///< stop iterating when change in error between steps is less than this
  parameters.setAbsoluteErrorTol(1e-8); ///< stop when cost-costchange < tol
  parameters.setMaxIterations(20); 
  // parameters.setlambdaInitial(1e-5);
  // parameters.setlambdaUpperBound(1e5); ///< defaults to 1e5
  // parameters.setlambdaLowerBound(1e-8); ///< defaults to 0.0
  parameters.setVerbosityLM("SUMMARY"); // SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA

  // build optimiser
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parameters);

  // optimise the graph
  Values result = optimizer.optimize();
  // int n_iterations = optimizer.iterations();

  // test sensitivity
  /// TODO: quantify how much results differ
  if (testSensitivity) {
    Values initialEstimatePerturbed = Noise::perturbValues(initialEstimate, 1e-8);
    Values sensResult = BackEnd::offline(graph, initialEstimatePerturbed);
    if (!result.equals(sensResult, 1e-6)) {
      cout << "WARNING: system is sensitive to input" << endl;
    }
  }

  // return result and info
  return result;
}

/* ************************************************************************* */
void FrontEnd::begin(SimulatedDataset dataset) {

  // create empty graph / estimate
  NonlinearFactorGraph graph;
  Values initialEstimate;

  // define sigmas 
  auto odomNoiseModel = noiseModel::Diagonal::Sigmas( Vector6::Constant(ODOM_SD) );
  auto boxNoiseModel = noiseModel::Diagonal::Sigmas( Vector4::Constant(BOX_SD) );

  // create initial estimates if not supplied
  vector<ConstrainedDualQuadric> noisyQuadrics;
  vector<Pose3> noisyTrajectory;
  bool estimatesSupplied = true;
  if (!estimatesSupplied) {

    /// Initialize trajectory from odometry
    /// NOTE: if no reference, will be in local coordinates
    // noisyTrajectory = SimulatedDataset::asTrajectory(dataset.noisyOdometry_);

    /// Initialize quadrics from boxes / trajectory
    // noisyQuadrics = FrontEnd::intializeQuadrics(noisyTrajectory, dataset.noisyBoxes_);
    
  } else {
    noisyTrajectory = dataset.noisyTrajectory_;
    noisyQuadrics = dataset.noisyQuadrics_;
  }


  // add trajectory estimate
  for (unsigned i = 0; i < noisyTrajectory.size(); i++) {
    Key poseKey(Symbol('x', i));
    initialEstimate.insert(poseKey, noisyTrajectory[i]);
  }

  // add or create quadric estimates
  for (unsigned j = 0; j < noisyQuadrics.size(); j++) {
    Key quadricKey(Symbol('q', j));
    initialEstimate.insert(quadricKey, noisyQuadrics[j]);
  }

  // create and add box factors
  for (unsigned j = 0; j < noisyQuadrics.size(); j++) {
    Key quadricKey(Symbol('q', j));

    for (unsigned i = 0; i < noisyTrajectory.size(); i++) {
      Key poseKey(Symbol('x', i));
      BoundingBoxFactor bbf(dataset.noisyBoxes_[j][i], dataset.calibration_, dataset.imageDimensions_, poseKey, quadricKey, boxNoiseModel);
      graph.add(bbf);
    }
  }

  // create odometry factors
  for (unsigned i = 0; i < dataset.noisyOdometry_.size(); i++) {
    Key startKey(Symbol('x', i));
    Key endKey(Symbol('x', i+1));
    BetweenFactor<Pose3> bf(startKey, endKey, dataset.noisyOdometry_[i], odomNoiseModel);
    graph.add(bf);
  }

  // send to back end
  Values optValues = BackEnd::offline(graph, initialEstimate, false);
}

// /* ************************************************************************* */
// vector<ConstrainedDualQuadric> FrontEnd::initializeQuadrics(const vector<Pose3>& trajectory, const vector<vector<AlignedBox2>>& boxes, const QuadricCamera& camera) {

//   vector<ConstrainedDualQuadric> quadrics;
  
//   // for each quadric 
//   // extract relevent poses
//   // extract aligning box at each pose
//   // calculate SVD
//   // constrain quadric
//   // check quadric is ok
//   // push back

//   return quadrics;
// }

// ConstrainedDualQuadric FrontEnd::initializeQuadric(const vector<Pose3>& poses, const vector<AlignedBox2>& boxes, const QuadricCamera& camera) {
//   ConstrainedDualQuadric quadric;
//   return quadric;
// }


/* ************************************************************************* */
int main(void) {

  // load test datasets
  SimulatedDataset dataset(QUAD_SD, ODOM_SD, BOX_SD);

  // run system on datasets
  FrontEnd::begin(dataset);

  // evaluate results against ground truth
  // metrics = Evaluation::evaluate(result);

  cout << "Main finished\n";
  return 1;
}
/* ************************************************************************* */
