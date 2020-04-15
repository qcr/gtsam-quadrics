
#include <quadricslam/geometry/DualEllipsoid.h>
#include <quadricslam/geometry/AlignedBox2.h>
#include <quadricslam/base/TestClass.h>

using namespace std;
using namespace gtsam;

int main() {

  // run simple test to ensure compiling correctly
  DualEllipsoid x; 
  AlignedBox2 box(1.2, 3.4, 5.6, 7.8);
  box.print("myboxis");

  TestClass t(5);
  double r = t.doWork(2);
  cout << r << endl;

  return 1;
}

