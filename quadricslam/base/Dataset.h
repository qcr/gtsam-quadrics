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

namespace gtsam {

class Dataset {
  private:
    double x_;

  public:
    Dataset() {};
    Dataset(double x) : x_(x) {};
    double getx(void) {return x_;}
    double doWork(double y);
};

}