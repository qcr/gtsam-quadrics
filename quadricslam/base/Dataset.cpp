/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Dataset.cpp
 * @date Apr 30, 2020
 * @author Lachlan Nicholson
 * @brief abstract base dataset, interface to run system
 */

#include <quadricslam/base/Dataset.h>

#include <gtsam/base/Matrix.h>

#include <iostream>

using namespace std;

namespace gtsam {
  
/* ************************************************************************* */
SimulatedDataset::SimulatedDataset(double quad_sd, double odom_sd, double box_sd) {

  // seed random
  generator_ = std::default_random_engine(19);

  // generate random quadrics
  trueQuadrics_.push_back(ConstrainedDualQuadric(Pose3(), Vector3(1.,2.,3.)));
  trueQuadrics_.push_back(ConstrainedDualQuadric(Pose3(), Vector3(1.,1.,1.)));
  trueQuadrics_.push_back(ConstrainedDualQuadric(Pose3(), Vector3(1.,1.,1.)));
  trueQuadrics_.push_back(ConstrainedDualQuadric(Pose3(), Vector3(1.,1.,1.)));
  noisyQuadrics_ = this->addNoise(trueQuadrics_, quad_sd);

  // generate n random camera positions looking at quadric(s)
  vector<Pose3> poses;
  poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(10,0,0), Point3(), Point3(0,0,1)));
  poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(0,-10,0), Point3(), Point3(0,0,1)));
  poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(-10,0,0), Point3(), Point3(0,0,1)));
  poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(0,10,0), Point3(), Point3(0,0,1)));
  poses.push_back(gtsam::CalibratedCamera::LookatPose(Point3(10,0,0), Point3(), Point3(0,0,1)));

  // interpolate poses and add noise
  trueTrajectory_ = interpolatePoses(poses, 3);
  trueOdometry_ = asOdometry(trueTrajectory_);
  noisyOdometry_ = addNoise(trueOdometry_, odom_sd);
  noisyTrajectory_ = asTrajectory(noisyOdometry_);

  // align initial quadric estimate and initial trajectory
  // either by putting the trajectory in the global reference frame (set ref=trajectory[0])
  // or by transforming the quadric to the local frame (transformTo(ref=trajector[0]))
  noisyTrajectory_ = transformTrajectoryFrom(noisyTrajectory_, poses[0]);

  // define calibration
  calibration_ = boost::make_shared<Cal3_S2>(525.0, 525.0, 0.0, 320.0, 240.0);
  imageDimensions_ = boost::make_shared<Vector2>(320.0, 240.0);

  // define bbox measurements
  trueBoxes_ = reprojectQuadrics(trueQuadrics_, trueTrajectory_, calibration_); 
  noisyBoxes_ = addNoise(trueBoxes_, box_sd);
}

/* ************************************************************************* */
Vector SimulatedDataset::generateNoise(int n, double sd) {
  std::normal_distribution<double> distribution(0.0, sd);
  vector<double> noiseVector(n);
  std::generate(noiseVector.begin(), noiseVector.end(), [&]{return distribution(generator_);});
  auto eigenVector = Eigen::Map<Eigen::VectorXd>(noiseVector.data(), noiseVector.size());
  return eigenVector;
}


/* ************************************************************************* */
void SimulatedDataset::printTrajectory(const vector<Pose3>& poses) {
  for (auto p : poses) {
      p.print("\n");
  }
}

/* ************************************************************************* */
vector<Pose3> SimulatedDataset::interpolatePoses(const vector<Pose3>& poses, int nBetween) {

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

/* ************************************************************************* */
vector<Pose3> SimulatedDataset::asOdometry(const vector<Pose3>& trajectory) {
  vector<Pose3> odometry;
  for (int i = 0; i < trajectory.size()-1; i++) {
      odometry.push_back(trajectory[i].between(trajectory[i+1]));
  }
  return odometry;
}

/* ************************************************************************* */
vector<Pose3> SimulatedDataset::asTrajectory(const vector<Pose3>& odometry, const Pose3& reference) {
  vector<Pose3> trajectory;
  trajectory.push_back(reference);
  for (int i = 0; i < odometry.size(); i++) {
      trajectory.push_back(trajectory.back().compose(odometry[i]));
  }
  return trajectory;
}

/* ************************************************************************* */
vector<Pose3> SimulatedDataset::addNoise(const vector<Pose3>& odometry, double sd) {
  vector<Pose3> noisyOdometry;
  for (auto pose : odometry) {
    Vector6 noiseVector = this->generateNoise(6, sd);
    Pose3 delta = Pose3::Retract(noiseVector);
    noisyOdometry.push_back(pose.compose(delta));
  }
  return noisyOdometry;
}

/* ************************************************************************* */
vector<AlignedBox2> SimulatedDataset::addNoise(const vector<AlignedBox2>& boxes, double sd) {
  vector<AlignedBox2> noisyBoxes;
  for (auto box : boxes) {
    Vector4 noiseVector = this->generateNoise(4, sd);
    noisyBoxes.push_back(box.addNoise(noiseVector));
  }
  return noisyBoxes;
}

/* ************************************************************************* */
vector<vector<AlignedBox2>> SimulatedDataset::addNoise(const vector<vector<AlignedBox2>>& boxes, double sd) {
  vector<vector<AlignedBox2>> noisyBoxes;
  for (auto boxes_ : boxes) {
    noisyBoxes.push_back(addNoise(boxes_, sd));
  }
  return noisyBoxes;
}

/* ************************************************************************* */
vector<ConstrainedDualQuadric> SimulatedDataset::addNoise(const vector<ConstrainedDualQuadric>& quadrics, double sd) {
  vector<ConstrainedDualQuadric> noiseyQuadrics;
  for (auto quadric : quadrics) {
    Vector9 noiseVector = this->generateNoise(9, sd);
    noiseyQuadrics.push_back(quadric.addNoise(noiseVector));
  }
  return noiseyQuadrics;
}


/** 
 * Recieves trajectory in local reference frame, transforms to global frame 
 * i.e, reference pose is where the first pose of the trajectory should sit in global frame
*/
/* ************************************************************************* */
vector<Pose3> SimulatedDataset::transformTrajectoryFrom(const vector<Pose3>& trajectory, const Pose3& reference) {
  vector<Pose3> transformed;
  for (auto pose : trajectory) {
      transformed.push_back(reference.transformPoseFrom(pose));
  }
  return transformed;
}

/* ************************************************************************* */
vector<AlignedBox2> SimulatedDataset::reprojectQuadric(const ConstrainedDualQuadric& quadric, const vector<Pose3>& trajectory, const boost::shared_ptr<Cal3_S2>& calibration) {
  vector<AlignedBox2> projections;
  for (auto pose : trajectory) {
      DualConic conic = QuadricCamera::project(quadric, pose, calibration);
      AlignedBox2 bounds = conic.bounds();
      projections.push_back(bounds);
  }
  return projections;
}

/* ************************************************************************* */
vector<vector<AlignedBox2>> SimulatedDataset::reprojectQuadrics(const vector<ConstrainedDualQuadric>& quadrics, const vector<Pose3>& trajectory, const boost::shared_ptr<Cal3_S2>& calibration) {
  vector<vector<AlignedBox2>> projections;
  for (auto quadric : quadrics) {
      projections.push_back(reprojectQuadric(quadric, trajectory, calibration));
  }
  return projections;
}





} // namespace gtsam