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

/**
 * A simple example with multiple camera positions and quadric landmarks
 *  - The robot moves in a circle around two objects
 *  - We have the noisy object detections from each pose 
 *  - We have the noisy odometry from the IMU
 *  - We can calculate an initial trajectory estimate
 *  - We have some way of gaining an initial estimate for each quadric
 */

#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <gtsam_quadrics/geometry/AlignedBox2.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>
#include <gtsam_quadrics/geometry/BoundingBoxFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <random>
#include <string>
#include <iostream>

#define ODOM_SD 0.01
#define QUAD_SD 0.1
#define BOX_SD 3

using namespace gtsam;
using namespace std;

int main(void) {

    // define key poses
    vector<Pose3> poses;
    poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(10,0,0), Point3(), Point3(0,0,1)));
    poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(0,-10,0), Point3(), Point3(0,0,1)));
    poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(-10,0,0), Point3(), Point3(0,0,1)));
    poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(0,10,0), Point3(), Point3(0,0,1)));
    poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(10,0,0), Point3(), Point3(0,0,1)));

    // define quadrics
    vector<ConstrainedDualQuadric> quadrics;
    quadrics.push_back(ConstrainedDualQuadric(Pose3(), Vector3(1.,2.,3.)));
    quadrics.push_back(ConstrainedDualQuadric(Pose3(Rot3(), Point3(0.1,0.1,0.1)), Vector3(1.,2.,3.)));

    // define calibration
    boost::shared_ptr<Cal3_S2> calibration(new Cal3_S2(525.0, 525.0, 0.0, 320.0, 240.0));

    // create random number engine and normal distribution 
    std::default_random_engine generator;
    std::normal_distribution<double> odom_noise_distribution(0.0, ODOM_SD);    
    std::normal_distribution<double> box_noise_distribution(0.0, BOX_SD);    
    std::normal_distribution<double> quad_noise_distribution(0.0, QUAD_SD);    

    // interpolate poses into a refined trajectory
    int nBetween = 10;
    vector<Pose3> trajectory;
    for(unsigned i = 0; i < poses.size()-1; i++) {
        trajectory.push_back(poses[i]);

        for(int j = 0; j < nBetween; j++) {
            double perc = (j+1) / double(nBetween+1);
            Pose3 newPose = gtsam::interpolate<Pose3>(poses[i], poses[i+1], perc);
            trajectory.push_back(newPose);
        }
    }
    trajectory.push_back(poses.back());
    
    // convert global trajectory to relative poses
    vector<Pose3> odometry;
    for (unsigned i = 0; i < trajectory.size()-1; i++) {
        odometry.push_back(trajectory[i].between(trajectory[i+1]));
    }


    // add noise to relative poses
    vector<Pose3> noisyOdometry;
    for (auto pose : odometry) {
        vector<double> noiseVector(6); 
        std::generate(noiseVector.begin(), noiseVector.end(), [&]{return odom_noise_distribution(generator);});
        Pose3 delta = Pose3::Retract(Vector6(noiseVector.data()));
        noisyOdometry.push_back(pose.compose(delta));
    }

    // recompose trajectory from relative poses
    vector<Pose3> noisyTrajectory;
    noisyTrajectory.push_back(Pose3());
    for (unsigned i = 0; i < noisyOdometry.size(); i++) {
        noisyTrajectory.push_back(noisyTrajectory.back().compose(noisyOdometry[i]));
    }

    // add noise to quadrics 
    vector<ConstrainedDualQuadric> noisyQuadrics;
    for (auto quadric : quadrics) {
        vector<double> noiseVector(9); 
        std::generate(noiseVector.begin(), noiseVector.end(), [&]{return quad_noise_distribution(generator);});

        Vector9 eigenNoise(noiseVector.data());
        Pose3 delta = Pose3::Retract(eigenNoise.head<6>());
        Pose3 noisyPose = quadric.pose().compose(delta);
        Vector3 noisyRadii = quadric.radii() + eigenNoise.tail<3>();

        noisyQuadrics.push_back(ConstrainedDualQuadric(noisyPose, noisyRadii));
    }

    // reproject true quadrics into each true pose
    vector<vector<AlignedBox2>> measurements; 
    for (auto quadric : quadrics) {
        vector<AlignedBox2> objectMeasurements;
        for (auto pose : trajectory) {
            DualConic conic = QuadricCamera::project(quadric, pose, calibration);
            AlignedBox2 bounds = conic.bounds();
            objectMeasurements.push_back(bounds);
        }
        measurements.push_back(objectMeasurements);
    }

    // add noise to bounding box measurements
    vector<vector<AlignedBox2>> noisyMeasurements;
    for (auto objectMeasurements : measurements) {
        vector<AlignedBox2> noisyObjectMeasurements;
        for (auto box : objectMeasurements) {
            vector<double> noiseVector(4); 
            std::generate(noiseVector.begin(), noiseVector.end(), [&]{return box_noise_distribution(generator);});
            AlignedBox2 noisyBox = AlignedBox2(box.vector() + Vector4(noiseVector.data()));
            noisyObjectMeasurements.push_back(noisyBox);
        }
        noisyMeasurements.push_back(noisyObjectMeasurements);
    }
    
    // because noisyTrajectory is in a local reference frame
    // we need to align it with our initial quadric estimate
    // either by putting the trajectory in the global reference frame 
    // or by transforming the quadric to the local frame
    vector<Pose3> transformed;
    for (auto pose : noisyTrajectory) {
        transformed.push_back(trajectory[0].transformPoseFrom(pose));
    }
    noisyTrajectory = transformed;

    // define noise models 
    boost::shared_ptr<noiseModel::Diagonal> priorNoiseModel = noiseModel::Diagonal::Sigmas(Vector6::Ones()*1e-1);
    boost::shared_ptr<noiseModel::Diagonal> odomNoiseModel = noiseModel::Diagonal::Sigmas(Vector6::Ones()*ODOM_SD);
    boost::shared_ptr<noiseModel::Diagonal> boxNoiseModel = noiseModel::Diagonal::Sigmas(Vector4::Ones()*BOX_SD);

    // create empty graph / estimate
    NonlinearFactorGraph graph;
    Values initialEstimate;

    // add prior to first pose
    PriorFactor<Pose3> priorFactor(Symbol('x',0), noisyTrajectory[0], priorNoiseModel);
    graph.add(priorFactor);

    // add trajectory estimate
    for (unsigned i = 0; i < noisyTrajectory.size(); i++) {
        Key poseKey(Symbol('x', i));
        initialEstimate.insert(poseKey, noisyTrajectory[i]);
    }

    // add quadric estimate
    for (unsigned i = 0; i < noisyQuadrics.size(); i++) {
        Key quadricKey(Symbol('q', i));
        initialEstimate.insert(quadricKey, noisyQuadrics[i]);
    }

    // create and add box factors
    for (unsigned i = 0; i < noisyTrajectory.size(); i++) {
        Key poseKey(Symbol('x', i));
            
        for (unsigned j = 0; j < noisyQuadrics.size(); j++) {
            Key quadricKey(Symbol('q', j));
            BoundingBoxFactor bbf(noisyMeasurements[j][i], calibration, poseKey, quadricKey, boxNoiseModel);
            graph.add(bbf);
        }
    }

    // create odometry factors
    for (unsigned i = 0; i < noisyOdometry.size(); i++) {
        Key startKey(Symbol('x', i));
        Key endKey(Symbol('x', i+1));
        BetweenFactor<Pose3> bf(startKey, endKey, noisyOdometry[i], odomNoiseModel);
        graph.add(bf);
    }

    LevenbergMarquardtParams parameters;
    parameters.setVerbosityLM("SUMMARY"); // SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA
    parameters.setMaxIterations(100); 
    parameters.setlambdaUpperBound(1e10); ///< defaults to 1e5

    // build optimiser
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parameters);

    // optimise the graph
    Values result = optimizer.optimize();

    return 1;
}
