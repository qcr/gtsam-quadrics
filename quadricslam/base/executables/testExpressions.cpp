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

using namespace std;
using namespace gtsam;
using namespace noiseModel;

Vector3 f(Vector3 x, OptionalJacobian<3,3> H = boost::none) {
    if (H) {
        *H = (Matrix33() << 2*x(0), 0.0, 0.0,
                            0.0, 2*x(1), 0.0,
                            0.0, 0.0, 2*x(2)).finished();
    }
    return x.cwiseProduct(x);
}

Vector3 g(Vector3 y, OptionalJacobian<3,3> H) {
    if (H) {
        *H = (Matrix33() << 2*y(0), 0.0, 0.0,
                            0.0, 2*y(1), 0.0,
                            0.0, 0.0, 2*y(2)).finished();
    }
    return y.cwiseProduct(y);
}


Vector3 h(Vector3 y, OptionalJacobian<3,3> H) {
    *H = Matrix33::Zero();
    return y.cwiseProduct(y);
}

namespace gtsam {
class Foo {
    private:
        Vector3 x_;
        Vector3 y_;

    public:
        Foo();
        Foo(Vector3 x, Vector3 y) : x_(x), y_(y) {};
        static Foo Create(Vector3 x, Vector3 y, OptionalJacobian<6,3> H1 = boost::none, OptionalJacobian<6,3> H2 = boost::none) {
            *H1 = Matrix63::Zero();
            *H2 = Matrix63::Zero();
            return Foo(x,y);
        }
        Vector3 doWork(const Vector3& z, OptionalJacobian<3,6> H1 = boost::none, OptionalJacobian<3,3> H2 = boost::none) const {
            *H1 = Matrix36::Zero();
            *H2 = Matrix33::Zero();
            return x_+y_+z;
        }

        // Foo retract(const Vector6& v) const {};
        // Vector6 localCoordinates(const Foo& other) const {};
        // void print(const std::string& s = "") const {};
        // bool equals(const Foo& other, double tol = 1e-9) const {};

};

template <>
struct traits<Foo> {
    enum { dimension = 6};
};

// template <>
// struct traits<Foo> : public Testable<Foo> {};

// template <>
// struct traits<const Foo> : public Testable<Foo> {};
}


ConstrainedDualQuadric constructQuadric(Pose3 pose, Vector3 radii, OptionalJacobian<9,6> H1, OptionalJacobian<9,3> H2) {
    *H1 = Matrix96::Zero();
    *H2 = Matrix93::Zero();
    return ConstrainedDualQuadric(pose, radii);
} 

Foo constructFoo(Vector3 x, Vector3 y, OptionalJacobian<6,3> H1, OptionalJacobian<6,3> H2) {
    *H1 = Matrix63::Zero();
    *H2 = Matrix63::Zero();
    return Foo(x,y);
}



// check this is correct 
Matrix kronecker(const Matrix m1, const Matrix m2) {
    Matrix m3(m1.rows()*m2.rows(), m1.cols()*m2.cols());
    for (int j = 0; j < m1.cols(); j++) {
        for (int i = 0; i < m1.rows(); i++) {
            m3.block(i*m2.rows(), j*m2.cols(), m2.rows(), m2.cols()) =  m1(i,j)*m2;
        }
    }
    return m3;
}

// if I can do this, I can use this for all matrix math in code
// https://atmos.washington.edu/~dennis/MatrixCalculus.pdf
// https://en.wikipedia.org/wiki/Kronecker_product
// https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
// https://people.maths.ox.ac.uk/gilesm/files/NA-08-01.pdf
// Some Theorems on Matrix Differentiation with Special Reference to Kronecker Matrix Products (H. Neudecker, 1969)
// A tutorial on SE(3) transformation parameterizations and on-manifold optimization Jose-Luis Blanco (p.35)
// - https://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf
// http://www.ee.ic.ac.uk/hp/staff/dmb/matrix/special.html#VecTranspose
Matrix34 somefunc(const Matrix34 A, const Matrix44 B, OptionalJacobian<12,12> H1, OptionalJacobian<12,16> H2) {
    if (H1) {
        *H1 = kronecker(B.transpose(), Matrix::Identity(4,4));
    } if (H2) {
        *H2 = kronecker(Matrix::Identity(4,4), A);
    }
    return A*B;
}



class D {
    public:
        double n_;

        D() {};

        D(double n) {
            n_ = n;
        }

        D operator*(const D& a) const {
            return D(n_*a.n_);
        }
};





void wrap_dot(void) {
    Matrix34 A = (Matrix34() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0).finished();
    Matrix44 B = (Matrix44::Ones() * 2.0);
    Expression<Matrix34> A_('A',1);
    Expression<Matrix44> B_('B',1);
    Values values;
    values.insert(symbol('A',1), A);
    values.insert(symbol('B',1), B);

    // method 2: use dot func
    // Note: this would require me to calc H1,H2 inside dot
    Expression<Matrix34> C_(&somefunc, A_, B_);
    Matrix34 C1 = C_.value(values);
    cout << "C1: \n" << C1;

    // Matrix34 (Matrix34::*operator*)(Matrix44) = &Matrix34::operator*;
    // Expression<Matrix34> C_2(A_, (&Matrix34::operator*)(Matrix44), B_);
    // Matrix34 C2 = C_2.value(values);
    // cout << "C2: \n" << C2;

    Matrix34 Ca = A.operator*(B);
    cout << Ca << endl;
    // Matrix34 Cb = A.applyOnTheRight(B);


    // D x(5);
    // D y(2.0);
    // D z = x*y;
    // D w = x.operator*(y);
    // cout << z.n_ << " " << w.n_ << endl;


    // method 1: just do A*B with the expressions
    // auto C = A*B;

    // method 3: hope they have optional jacobians in matrix multiplaction 
    // as this is what tf / pytorch will do automatically

    // Eigen::Matrix<double, 12, 12> dC_dA;
    // Eigen::Matrix<double, 12, 16> dC_dB;
    // Matrix34 C = dot(A,B, dC_dA, dC_dB);
    // cout << "A: \n" << A << endl;
    // cout << "B: \n" << B << endl;
    // cout << "C: \n" << C << endl;
    // cout << "dC_dA: \n" << dC_dA << endl;
    // cout << "dC_dB: \n" << dC_dB << endl;

    
}

void wrap_class(void) {
    // Expression<Pose3> p('p', 1);
    // Expression<Vector3> r('r', 1);
    // Expression<ConstrainedDualQuadric> Q1(&constructQuadric, p, r);
    // Expression<ConstrainedDualQuadric> Q2(&ConstrainedDualQuadric::Create, p, r);

    Expression<Vector3> x('x', 1);
    Expression<Vector3> y('y', 1);
    Expression<Vector3> z('z', 1);
    Values values;
    values.insert(symbol('x', 1), Vector3(1.,2.,3.));
    values.insert(symbol('y', 1), Vector3(2.,2.,2.));
    values.insert(symbol('z', 1), Vector3(3.,4.,5.));

    // Expression<Foo> foo1(&constructFoo, x, y); // need to provide dFoo_dx (6x3) and dFoo_dy (6x3)
    Expression<Foo> foo1(&Foo::Create, x, y);
    Expression<Vector3> result(foo1, &Foo::doWork, z); // need to provide dres_dFoo (3,6) and dres_dz (3,3)

    Vector3 res = result.value(values);
    cout << "res: " << res.transpose() << endl;
}

void without_expressions(void) {
    cout << "Test WITHOUT expressions: \n";
    
    // define x
    Vector3 x(1.,2.,3.);

    // calculate y and dy_dx
    Matrix33 dy_dx;
    Vector3 y = f(x, dy_dx);
    // Vector3 t = f(x);

    // calculate g and dz_dy
    Matrix33 dz_dy;
    Vector3 z = g(y, dz_dy);

    // calculate dz_dx
    Matrix33 dz_dx = dz_dy * dy_dx;

    // print result and jacobian
    cout << "z: " << z.transpose() << endl;
    cout << "dz_dx:\n" << dz_dx << endl;
    
}

void with_expressions(void) {
    // define expression chain to calculate z from x
    // ensuring each function exposes jacobians along the way
    Expression<Vector3> x('x',1); // x = (1,2,3)
    Expression<Vector3> y(&f, x); // y = f(x)
    Expression<Vector3> z(&g, y); // z = g(y)

    // insert variables into values
    Values values;
    values.insert(symbol('x',1), Vector3(1.,2.,3.));

    // get result and jacobian
    std::vector<Matrix> gradients;
    Matrix3 dz_dx; gradients.push_back(dz_dx);
    Vector3 result = z.value(values, gradients);
    dz_dx = gradients[0];

    // print result and jacobian
    cout << "z: " << result.transpose() << endl;
    cout << "dz_dx:\n" << dz_dx << endl;
}

void print_expression(Expression<Vector3> e) {
    // print expression
    e.print("expression: ");

    // print keys
    std::set<Key> keys = e.keys();
    cout << "keys ";
    for (auto k : keys) {
        cout << symbolChr(k) << symbolIndex(k) << ' ';
    }
    cout << endl;

    // print dims
    std::map<Key, int> map;
    e.dims(map);
    cout << "dims " << endl;
    for (auto t : map) {
        cout << "map[" << symbolChr(t.first) << symbolIndex(t.first) << "] = " << t.second << endl;
    }
} 

void without_jacobians(void) {
    // define expression chain to calculate z from x
    // ensuring each function exposes jacobians along the way
    Expression<Vector3> x('x',1); // x = (1,2,3)
    Expression<Vector3> y(&h, x); // y = f(x)

    // insert variables into values
    Values values;
    values.insert(symbol('x',1), Vector3(1.,2.,3.));

    // get result and jacobian
    Vector3 result = y.value(values);

    // print result and jacobian
    cout << "z: " << result.transpose() << endl;
}


int main(void) {

    Matrix33 x = (Matrix33() << 1.0,2.0,3.0).finished();
    cout << "x\n" << x << endl;


    // Pose3 pose(Rot3::rodriguez(3.14,2.0,1.0), Point3(1,2,3));
    // Matrix H;
    // Vector6 x = Pose3::ChartAtOrigin::Local(pose, H);
    // cout << "x " << x.transpose() << endl;
    // cout << "H\n" << H << endl;


    // Pose3 p1(Rot3(), Point3(1,2,3));
    // Pose3 p2(Rot3(), Point3(4,5,6));

    // Expression<Pose3> p1_(p1);
    // Expression<Pose3> p2_(p2);

    // Expression<Pose3> p3_(p1_*p2_);
    // Pose3 p3 = p1*p2;

    // Matrix3 m1 = (Matrix3() << 1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0).finished();
    // Matrix3 m2 = (Matrix3() << 5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0).finished();
    // Matrix3 m3 = m1*m2;
    // Expression<Matrix3> m1_(m1);
    // Expression<Matrix3> m2_(m2);
    // Expression<Matrix3> m3_(m1_*m2_);


    cout << "\nTEST: warp_dot\n";
    wrap_dot();

    cout << "\nTEST: wrap_class\n";
    wrap_class();

    cout << "\nTEST: with_expressions\n";
    with_expressions();

    cout << "\nTEST: without_expressions\n";
    without_expressions();

    cout << "\nTEST: without_jacobians\n";
    without_jacobians();



    return 1;
}
