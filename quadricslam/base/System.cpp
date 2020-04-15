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

#include <string>
#include <iostream>

class System {
    public:
        static void offline(/*graph, initialEstimate*/) {

            // define parameters

            // build optimiser

            // optimise the graph

            // return result and info

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


int main(void) {

    std::cout << "Starting QuadricSLAM on Dataset" << std::endl;

    // run on a dataset
    // DATASET_PATH = x
    // FrontEnd.begin(DATASET_PATH)

}