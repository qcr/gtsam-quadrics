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

class System {
    public:
        static Values offline(NonlinearFactorGraph graph, Values initialEstimate) {

            // define parameters
            LevenbergMarquardtParams parameters;
            parameters.setRelativeErrorTol(1e-10); ///< stop iterating when change in error between steps is less than this
            parameters.setAbsoluteErrorTol(1e-8); ///< stop when cost-costchange < tol
            parameters.setMaxIterations(20); 
            // parameters.setlambdaInitial(1e-5);
            // parameters.setlambdaUpperBound(1e10) ///< defaults to 1e5
            // parameters.setlambdaLowerBound(0) ///< defaults to 0.0
            parameters.setVerbosityLM("SUMMARY"); // SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA

            // build optimiser
            LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parameters);

            // optimise the graph
            Values result = optimizer.optimize();
            int n_iterations = optimizer.iterations();

            // return result and info
            return result;
        }
};

class FrontEnd {
    public:
        static void begin(std::string dataset_path) {

            // load dataset

            // calculate intermediates

            // assemble factor graph and estimate from data
            // graph = buildGraph(odometry, boxes, noisemodels);
            // initialEstimate = buildEstimate(odometry, boxes);

            // run back-end to digest graph
            // result = System.offline(graph, initialEstimate)

        }
};


void printTrajectory(const vector<Pose3>& poses) {
    for (auto p : poses) {
        p.print("\n");
    }
}

vector<Pose3> interpolatePoses(const vector<Pose3>& poses, int nBetween) {

    vector<Pose3> trajectory;
    for(int i = 0; i < poses.size()-1; i++) {
        trajectory.push_back(poses[i]);

        double p = 1.0/(nBetween+1);
        for(int j = 0; j < nBetween; j++) {
            double perc = p*(j+1);
            Pose3 newPose = gtsam::interpolate<Pose3>(poses[i], poses[i+1], perc);
            trajectory.push_back(newPose);
        }
    }
    trajectory.push_back(poses.back());
    return trajectory;
}

vector<Pose3> asOdometry(const vector<Pose3>& trajectory) {
    vector<Pose3> odometry;
    for (int i = 0; i < trajectory.size()-1; i++) {
        odometry.push_back(trajectory[i].between(trajectory[i+1]));
    }
    return odometry;
}

vector<Pose3> asTrajectory(const vector<Pose3>& odometry, const Pose3& reference = Pose3()) {
    vector<Pose3> trajectory;
    trajectory.push_back(reference);
    for (int i = 0; i < odometry.size(); i++) {
        trajectory.push_back(trajectory.back().compose(odometry[i]));
    }
    return trajectory;
}

vector<Pose3> addNoise(const vector<Pose3>& odometry, double sd) {
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, sd);
    vector<Pose3> noisyOdometry;

    for (auto pose : odometry) {
        vector<double> noiseVector(6); 
        std::generate(noiseVector.begin(), noiseVector.end(), [&]{return distribution(generator);});
        Pose3 delta = Pose3::Retract(Vector6(noiseVector.data()));
        noisyOdometry.push_back(pose.compose(delta));
    }
    return noisyOdometry;
}

vector<AlignedBox2> addNoise(const vector<AlignedBox2>& boxes, double sd) {
    vector<AlignedBox2> noisyBoxes;
    for (auto box : boxes) {
        noisyBoxes.push_back(box.addNoise(sd));
    }
    return noisyBoxes;
}


/** 
 * Recieves trajectory in local reference frame, transforms to global frame 
 * i.e, reference pose is where the first pose of the trajectory should sit in global frame
*/
vector<Pose3> transformTrajectoryFrom(const Pose3& reference, const vector<Pose3>& trajectory) {
    vector<Pose3> transformed;
    for (auto pose : trajectory) {
        transformed.push_back(reference.transformPoseFrom(pose));
    }
    return transformed;
}

vector<AlignedBox2> reprojectQuadric(const ConstrainedDualQuadric& quadric, const vector<Pose3>& trajectory, const boost::shared_ptr<Cal3_S2>& calibration) {
    vector<AlignedBox2> projections;
    for (auto pose : trajectory) {
        DualConic conic = QuadricCamera::project(quadric, pose, calibration);
        AlignedBox2 bounds = conic.bounds();
        projections.push_back(bounds);
    }
    return projections;
}

class TestDataset {
public:
void test(void) {
    std::cout << "Starting QuadricSLAM on test Dataset" << std::endl;

    // define trajectory and odometry
    vector<Pose3> poses;
    poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(10,0,0), Point3(), Point3(0,0,1)));
    poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(0,-10,0), Point3(), Point3(0,0,1)));
    poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(-10,0,0), Point3(), Point3(0,0,1)));
    poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(0,10,0), Point3(), Point3(0,0,1)));
    poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(10,0,0), Point3(), Point3(0,0,1)));
    vector<Pose3> trajectory = interpolatePoses(poses, 3);
    vector<Pose3> odometry = asOdometry(trajectory);
    vector<Pose3> noisyOdometry = addNoise(odometry, ODOM_SD);
    vector<Pose3> noisyTrajectory = asTrajectory(noisyOdometry);

    // define quadrics
    ConstrainedDualQuadric quadric(Pose3(), Vector3(1.,2.,3.));
    ConstrainedDualQuadric noisyQuadric = quadric.addNoise(QUAD_SD);

    // define calibration
    boost::shared_ptr<Cal3_S2> calibration(new Cal3_S2(525.0, 525.0, 0.0, 320.0, 240.0));
    boost::shared_ptr<Vector2> imageDimensions(new Vector2(320.0, 240.0));

    // define bbox measurements
    vector<AlignedBox2> measurements = reprojectQuadric(quadric, trajectory, calibration); 
    vector<AlignedBox2> noisyMeasurements = addNoise(measurements, BOX_SD);

    // define sigmas 
    boost::shared_ptr<noiseModel::Diagonal> odomNoiseModel = noiseModel::Diagonal::Sigmas( (Vector6() << ODOM_SD,ODOM_SD,ODOM_SD,ODOM_SD,ODOM_SD,ODOM_SD).finished() );
    boost::shared_ptr<noiseModel::Diagonal> boxNoiseModel = noiseModel::Diagonal::Sigmas(Vector4(BOX_SD,BOX_SD,BOX_SD,BOX_SD));


    // create empty graph / estimate
    NonlinearFactorGraph graph;
    Values initialEstimate;

    // create and add box factors
    Key quadricKey(Symbol('q', 0));
    for (int i = 0; i < trajectory.size(); i++) {
        Key poseKey(Symbol('x', i));
        BoundingBoxFactor bbf(noisyMeasurements[i], calibration, imageDimensions, poseKey, quadricKey, boxNoiseModel);
        graph.add(bbf);
    }

    // create odometry factors
    for (int i = 0; i < odometry.size(); i++) {
        Key startKey(Symbol('x', i));
        Key endKey(Symbol('x', i+1));
        BetweenFactor<Pose3> bf(startKey, endKey, noisyOdometry[i]);
        graph.add(bf);
    }

    // add trajectory estimate
    for (int i = 0; i < trajectory.size(); i++) {
        Key poseKey(Symbol('x', i));
        initialEstimate.insert(poseKey, noisyTrajectory[i]);
    }

    // add quadric estimate
    initialEstimate.insert(quadricKey, noisyQuadric);

    // send to back end
    Values optValues = System::offline(graph, initialEstimate);
    cout << "optimisation complete!" << endl;
} 
};




int main(void) {

    TestDataset td;
    td.test();
    cout << "done" << endl;
    return 1;
}

// run on a dataset
// DATASET_PATH = x
// FrontEnd.begin(DATASET_PATH)