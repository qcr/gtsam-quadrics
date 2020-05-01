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

class BackEnd {
    public:
        static Values offline(NonlinearFactorGraph graph, Values initialEstimate) {

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

            // return result and info
            return result;
        }
};

class FrontEnd {
    public:
        static void begin(SimulatedDataset dataset) {

            // create empty graph / estimate
            NonlinearFactorGraph graph;
            Values initialEstimate;

            // define sigmas 
            boost::shared_ptr<noiseModel::Diagonal> odomNoiseModel = noiseModel::Diagonal::Sigmas( (Vector6() << ODOM_SD,ODOM_SD,ODOM_SD,ODOM_SD,ODOM_SD,ODOM_SD).finished() );
            boost::shared_ptr<noiseModel::Diagonal> boxNoiseModel = noiseModel::Diagonal::Sigmas(Vector4(BOX_SD,BOX_SD,BOX_SD,BOX_SD));

            // add or create trajectory estimate 
            for (unsigned i = 0; i < dataset.noisyTrajectory_.size(); i++) {
                Key poseKey(Symbol('x', i));
                // cout << "pose " << i << endl << dataset.noisyTrajectory_[i].matrix() << endl;
                initialEstimate.insert(poseKey, dataset.noisyTrajectory_[i]);
            }

            // add quadric estimates
            for (unsigned j = 0; j < dataset.noisyQuadrics_.size(); j++) {
                Key quadricKey(Symbol('q', j));
                // cout << "quadric " << j << endl << dataset.noisyQuadrics_[j].matrix() << endl;
                initialEstimate.insert(quadricKey, dataset.noisyQuadrics_[j]);
            }

            // create and add box factors
            for (unsigned j = 0; j < dataset.noisyQuadrics_.size(); j++) {
                Key quadricKey(Symbol('q', j));
                // cout << "Quadric " << j << endl;

                for (unsigned i = 0; i < dataset.noisyTrajectory_.size(); i++) {
                    Key poseKey(Symbol('x', i));
                    BoundingBoxFactor bbf(dataset.noisyBoxes_[j][i], dataset.calibration_, dataset.imageDimensions_, poseKey, quadricKey, boxNoiseModel);
                    // cout << "box " << i << ": " << dataset.noisyBoxes_[j][i].vector().transpose() << endl;
                    // cout << "error(" << j << "," << i << "): " << bbf.unwhitenedError(initialEstimate).transpose() << endl;
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
            Values optValues = BackEnd::offline(graph, initialEstimate);

            Pose3 r1 = optValues.at<Pose3>(Symbol('x',2));
            r1.print("\nx2\n");

            // perturb values

            // for each value
            // add 1e-8 to each value
            // update value in values

            // auto value = optValues.at(1);
            // value.dim();

            // auto vv = optValues.zeroVectors();
            // vv.print("\nvectorvalues\n");

            optValues = Noise::perturbValues(optValues, 1e-1);

            r1 = optValues.at<Pose3>(Symbol('x',2));
            r1.print("\nx2\n");

        }
};






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

// run on a dataset
// DATASET_PATH = x
// FrontEnd.begin(DATASET_PATH)