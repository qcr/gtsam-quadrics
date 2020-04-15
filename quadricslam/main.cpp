
#include <quadricslam/geometry/ConstrainedDualQuadric.h>
#include <quadricslam/geometry/AlignedBox2.h>
#include <quadricslam/base/TestClass.h>

using namespace std;
using namespace gtsam;

int main() {

  // run simple test to ensure compiling correctly
  ConstrainedDualQuadric x; 
  AlignedBox2 box(1.2, 3.4, 5.6, 7.8);
  box.print("myboxis");

  TestClass t(5);
  double r = t.doWork(2);
  // cout << r << endl;

  // Vector4 y(1.0,2.0,3.0,4.0);
  // cout << y.pow(2) << endl;

  // power of a matrix
  Matrix22 z = (Matrix22() << 1.0,2.0,3.0,4.0).finished();
  print((Matrix)z, "z:");
  print((Matrix)z.array().pow(2).matrix(), "z.array():");

  return 1;
}

