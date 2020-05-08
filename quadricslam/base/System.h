/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file System.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief containing the systems front and back end
 */

#pragma once

#include <quadricslam/base/Dataset.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

namespace gtsam {

/**
 * @class BackEnd
 * A simple test class (double)
 */
class BackEnd {
  public:
  
    /** 
     * Static function to optimise the graph
     * optionally test sensitivity
    */
    static Values offline(const NonlinearFactorGraph& graph, const Values& initialEstimate);
};


/**
 * @class FrontEnd
 * A simple test class (double)
 */
class FrontEnd {
  public:

    /** 
     * Static function to build factor graph / estimate
    */
    static void begin(SimulatedDataset dataset);

    /**
     * Initialize quadrics from trajectory, boxes, camera
     * Checking each quadric is OK and constrained
     */
    static vector<ConstrainedDualQuadric> initializeQuadrics(const vector<Pose3>& trajectory, const vector<vector<AlignedBox2>>& boxes, const QuadricCamera& camera);

    /**
     * Initialize a single quadric using SVD
     */ 
    static ConstrainedDualQuadric initializeQuadric(const vector<Pose3>& poses, const vector<AlignedBox2>& boxes, const QuadricCamera& camera);

};





} // namespace gtsam