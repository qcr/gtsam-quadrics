// The two new headers that allow using our Automatic Differentiation Expression framework
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

// Header order is close to far
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <vector>

using namespace std;
using namespace gtsam;
using namespace noiseModel;

Vector3 f(Vector3 x, OptionalJacobian<3,3> H) {
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



void without_expressions(void) {
    cout << "Test WITHOUT expressions: \n";
    
    // define x
    Vector3 x(1.,2.,3.);

    // calculate y and dy_dx
    Matrix33 dy_dx;
    Vector3 y = f(x, dy_dx);

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
    cout << "Test with expressions: \n";
    
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


int main(void) {


    with_expressions();
    without_expressions();



    return 1;
}
