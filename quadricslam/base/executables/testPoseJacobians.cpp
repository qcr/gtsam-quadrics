/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testExpressions.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief an example of how to use gtsam expressions
 */

// The two new headers that allow using our Automatic Differentiation Expression framework
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

// Header order is close to far
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Testable.h>

#include <vector>
#include <quadricslam/geometry/ConstrainedDualQuadric.h>
#include <gtsam/base/numericalDerivative.h>

using namespace std;
using namespace gtsam;
using namespace noiseModel;


// GOAL1: test pose retract/matrix
// GOAL2: confirm or deny numerical jacobian
// GOAL3: derive jacobian for and use arbitrary parametrization 


// namespace gtsam {
// class Foo {
//     private:
//         Vector3 x_;
//         Vector3 y_;

//     public:
//         Foo();
//         Foo(Vector3 x, Vector3 y) : x_(x), y_(y) {};
//         static Foo Create(Vector3 x, Vector3 y, OptionalJacobian<6,3> H1 = boost::none, OptionalJacobian<6,3> H2 = boost::none) {
//             return Foo(x,y);
//         }
//         Vector3 doWork(const Vector3& z, OptionalJacobian<3,6> H1 = boost::none, OptionalJacobian<3,3> H2 = boost::none) const {
//             return x_+y_+z;
//         }

//         // Foo retract(const Vector6& v) const {};
//         // Vector6 localCoordinates(const Foo& other) const {};
//         void print(const std::string& s = "") const {};
//         bool equals(const Foo& other, double tol = 1e-9) const {};

// };
// template <>
// struct traits<Foo> {
//     enum { dimension = 6};
// };
// }



Matrix31 h(const Rot3& r, const Vector3& p) {
    return r.matrix() * p;
}



Matrix33 retract_matrix(const Vector3& v) {
    return Rot3::Retract(v).matrix();
}



// for any function that acts on a pose matrix X, GTSAM needs us to express the jacobian wrt to the pose's 6 degrees of freedom 
// QUESTION: how can we define the .matrix() function analytically wrt a pose's 6 degrees of freedom 
// QUESTION: does retract do this?
// QUESTION: why does the numerical jacobian suggest that x1 doesn't impact X11,X21,X31? 
// Question: even if ^^ it shouldn't affect numerical diff because the point that the derivative is calculated isn't with yz = 0
void testPose(void) {
    
    // test retract to see if following cayley math
    Vector6 poseVector = (Vector6() << 1.0,2.,3.,  0.,0.,0.).finished();
    Matrix44 poseMatrixRetract = Pose3::Retract(poseVector).matrix();
    Matrix44 poseMatrixChartAtOriginRetract = Pose3::ChartAtOrigin::Retract(poseVector).matrix();
    Matrix44 poseMatrixTraitsRetract = traits<Pose3>::Retract(Pose3(), poseVector).matrix();

    cout << "poseVector\n" << poseVector.transpose() << endl << endl; 
    cout << "poseMatrixRetract\n" << poseMatrixRetract << endl << endl; 
    cout << "poseMatrixChartAtOriginRetract\n" << poseMatrixChartAtOriginRetract << endl << endl; 
    cout << "poseMatrixTraitsRetract\n" << poseMatrixTraitsRetract << endl << endl; 

    // create matricies from different retractions
    Vector3 rotVector = (Vector3() << 1.2,12.3,3.5).finished();
    Rot3 rot = Rot3::Retract(rotVector);
    Matrix33 rotMatrix = rot.matrix();
    Matrix33 rotMatrixCayley = Rot3::CayleyChart::Retract(rotVector).matrix();
    Matrix33 rotMatrixExpmap = Rot3::Expmap(rotVector).matrix();

    cout << "rotVector\n" << rotVector.transpose() << endl << endl; 
    cout << "rotMatrixCayley\n" << rotMatrixCayley << endl << endl; 
    cout << "rotMatrixExpmap\n" << rotMatrixExpmap << endl << endl; 



    // check numerical derivative 
    boost::function<Matrix3(const Rot3&)> matrix_(boost::bind(&Rot3::matrix, _1));
    Eigen::Matrix<double, 9,3> dMatrix = numericalDerivative11(matrix_, rot, 1e-6);
    boost::function<Matrix3(const Vector3&)> retract_(boost::bind(&retract_matrix, _1));
    Eigen::Matrix<double, 9,3> dRetract = numericalDerivative11(retract_, rotVector, 1e-6);

    cout << "dMatrix\n" << dMatrix << endl << endl; // this is what the numerical diff gives us 
    cout << "dRetract\n" << dRetract << endl << endl; // this is what matlab gives us 

    // check what happens to rot matrix as we move r1
    Vector3 rotVectorDiff = (Vector3() << 1.0,0.0,0.0).finished();
    Matrix33 chartJacobian = Rot3::ExpmapDerivative(rotVector);
    Matrix33 rotRetractDiffMatrix = rot.retract(rotVectorDiff).matrix(); 
    Matrix33 rotRetractAddVectors = Rot3::Retract(Rot3::LocalCoordinates(rot)+rotVectorDiff).matrix();

    cout << "chartJacobian\n" << chartJacobian << endl << endl;

    cout << "rotMatrix\n" << rotMatrix << endl << endl;
    cout << "rotRetractDiffMatrix\n" << rotRetractDiffMatrix << endl << endl;
    cout << "rotRetractAddVectors\n" << rotRetractAddVectors << endl << endl;

    Vector3 p = (Vector3() << 3.,4.,5.).finished();
    // Matrix31 y = h(rot, p);
    boost::function<Matrix31(const Rot3&, const Vector3&)> h_(boost::bind(&h, _1, _2));
    Eigen::Matrix<double, 3,3> dY_dr = numericalDerivative21(h_, rot, p, 1e-6);
    Eigen::Matrix<double, 3,3> dY_dp = numericalDerivative22(h_, rot, p, 1e-6);
    cout << "dY_dr\n" << dY_dr << endl << endl;
    cout << "dY_dp\n" << dY_dp << endl << endl;

  

}
// pose3::retracts are definitely using cayley for rotation
// rot3.retract(1,0,0) shows that x1 doesn't change X11,X21,X31
// per cayley chart, if x2,x3 are zero, then X11,X21,X31 = [1,0,0] regardless of x1, which is what the numerical jacobian does
// it is clear that d(rot3.matrix()) and d(Rot3::retract(v).matrix()) are quite different 
// rot.retract(dx) != Rot3::Retract(Rot3::Local(rot)+dx)
// ^ because rot.retract(dx) = rot.matrix() * Rot3::Retract(dx).matrix()
// if y = h(R,p) = R*p where p = [3,4,5] and r = [1,2,3]
// then dy = R*[[-p]x I3] where first 3x3 is dy_dr and second is dy_dp


int main(void) {

    testPose();

    return 1;
}
