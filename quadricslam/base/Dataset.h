/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Dataset.h
 * @date Apr 30, 2020
 * @author Lachlan Nicholson
 * @brief abstract base dataset, interface to run system
 */

#pragma once

#include <quadricslam/geometry/AlignedBox2.h>
#include <quadricslam/geometry/ConstrainedDualQuadric.h>
#include <quadricslam/geometry/QuadricCamera.h>
#include <quadricslam/geometry/BoundingBoxFactor.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/CalibratedCamera.h>

#include <vector>
#include <random>
#include <string>
#include <iostream>

using std::vector;

namespace gtsam {

/**
 * @class Dataset
 * An interface class for the front end
 * Derived classes must provide 
 * - odometry
 * - boxes
 * - calibration
 * And optional initial estimates:
 * - quadrics
 * - trajectory
 */
class Dataset {
  protected:
    vector<Pose3> trueOdometry;
    vector<Pose3> noisyOdometry;
    vector<AlignedBox2> trueBoxes;
    vector<AlignedBox2> noisyBoxes;

  public:
    Dataset() {};
};


class SimulatedDataset {
  public:
    std::default_random_engine generator_;
    boost::shared_ptr<Cal3_S2> calibration_;
    boost::shared_ptr<Vector2> imageDimensions_;
    vector<ConstrainedDualQuadric> trueQuadrics_;
    vector<ConstrainedDualQuadric> noisyQuadrics_;
    vector<Pose3> trueTrajectory_;
    vector<Pose3> noisyTrajectory_;
    vector<Pose3> trueOdometry_;
    vector<Pose3> noisyOdometry_;
    vector<vector<AlignedBox2> > trueBoxes_;
    vector<vector<AlignedBox2> > noisyBoxes_;
  
  public:
    SimulatedDataset(double quad_sd, double odom_sd, double box_sd);
    // static SimulatedDataset simulate(double quad_sd, double odom_sd, double box_sd);

    Vector generateNoise(int n, double sd);
    vector<Pose3> addNoise(const vector<Pose3>& odometry, double sd);
    vector<AlignedBox2> addNoise(const vector<AlignedBox2>& boxes, double sd);
    vector<vector<AlignedBox2>> addNoise(const vector<vector<AlignedBox2>>& boxes, double sd);
    vector<ConstrainedDualQuadric> addNoise(const vector<ConstrainedDualQuadric>& quadrics, double sd);

    static void printTrajectory(const vector<Pose3>& poses);
    static vector<Pose3> interpolatePoses(const vector<Pose3>& poses, int nBetween);
    static vector<Pose3> asOdometry(const vector<Pose3>& trajectory);
    static vector<Pose3> asTrajectory(const vector<Pose3>& odometry, const Pose3& reference = Pose3());
    static vector<Pose3> transformTrajectoryFrom(const vector<Pose3>& trajectory, const Pose3& reference);
    static vector<AlignedBox2> reprojectQuadric(const ConstrainedDualQuadric& quadric, const vector<Pose3>& trajectory, const boost::shared_ptr<Cal3_S2>& calibration);
    static vector<vector<AlignedBox2>> reprojectQuadrics(const vector<ConstrainedDualQuadric>& quadrics, const vector<Pose3>& trajectory, const boost::shared_ptr<Cal3_S2>& calibration);
};




} // namespace gtsam