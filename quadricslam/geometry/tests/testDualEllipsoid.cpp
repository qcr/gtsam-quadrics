

#include <CppUnitLite/TestHarness.h>

#include <quadricslam/geometry/DualEllipsoid.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

TEST(DualEllipsoid, basictest) {
  const double x = 5.0;
  const double y = 5.0;
  EXPECT(assert_equal(x, y));
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
