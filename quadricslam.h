/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file AlignedBox2.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief an axis aligned 2D bounding box
 */

/* -------------------------------------
  NOTES: 
    - cannot have comments after gtsam namespace at eof (i.e, // namespace gtsam)
 */


namespace gtsam {

#include <quadricslam/base/TestClass.h>
class TestClass {
  TestClass();
  TestClass(double x);
  double getx();
  double doWork(double y);
};

}
