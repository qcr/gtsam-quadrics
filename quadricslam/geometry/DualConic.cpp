/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DualConic.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a dual conic
 */

#include <quadricslam/geometry/DualConic.h>
#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
DualConic::DualConic() {
  dC_ = (Matrix33() << 1.0, 0.0, 0.0, 
                        0.0, 1.0, 0.0,
                        0.0, 0.0, -1.0).finished();
}

/* ************************************************************************* */
DualConic::DualConic(const Matrix33& dC) {
  dC_ = dC;
}

/* ************************************************************************* */
Matrix33 DualConic::matrix() const {
  return dC_;
}

/* ************************************************************************* */
void DualConic::normalize() {
  dC_ = dC_/dC_(2,2);
}

/* ************************************************************************* */
// TODO: assert conic is closed (eccentricity)
// assert bounds are real-valued
// normalize conic
AlignedBox2 DualConic::bounds(OptionalJacobian<4,9> H) const {
  double xmin = (dC_(0,2) + std::sqrt(dC_(0,2)*dC_(0,2)-dC_(2,2)*dC_(0,0))) / dC_(2,2);
  double xmax = (dC_(0,2) - std::sqrt(dC_(0,2)*dC_(0,2)-dC_(2,2)*dC_(0,0))) / dC_(2,2);
  double ymin = (dC_(1,2) + std::sqrt(dC_(1,2)*dC_(1,2)-dC_(2,2)*dC_(1,1))) / dC_(2,2);
  double ymax = (dC_(1,2) - std::sqrt(dC_(1,2)*dC_(1,2)-dC_(2,2)*dC_(1,1))) / dC_(2,2);

  if (H) {
    // Eigen::Matrix<double, 9,5> dC_dc = this->dC_dc(); // used to parametrizing conic as 5vec

    // // calculate db_dC:
    // double x = 1.0 / std::sqrt( dC_(0,2)*dC_(0,2) - dC_(0,0)*dC_(2,2) );
    // double y = 1.0 / std::sqrt( dC_(1,2)*dC_(1,2) - dC_(1,1)*dC_(2,2) );

    // double db1_dC11 = -0.5*x;
    // double db1_dC31 = 1./dC_(2,2) * (dC_(0,2)*x + 1);
    // double db2_dC22 = -0.5*y;
    // double db2_dC32 = 1./dC_(2,2) * (dC_(1,2)*y + 1);
    
    // double db1_dC33 = -1/(dC_(2,2) * dC_(2,2)) * (dC_(0,2) + x) - dC_(0,0) / (2.0*dC_(2,2)*x);
    // double db2_dC33 = -1/(dC_(2,2) * dC_(2,2)) * (dC_(1,2) + x) - dC_(1,1) / (2.0*dC_(2,2)*x);
    // double db3_dC33 = -1/(dC_(2,2) * dC_(2,2)) * (dC_(0,2) + x) + dC_(0,0) / (2.0*dC_(2,2)*x);
    // double db4_dC33 = -1/(dC_(2,2) * dC_(2,2)) * (dC_(1,2) + x) + dC_(1,1) / (2.0*dC_(2,2)*x);


    // OPTION1: db_dC31 == db_dC13
    //            c11, c21,  c31,    c12, c22,         c32,        c13,      c23,      c33
    // db_dC << db1_dC11,  0, db1_dC31,  0,     0,         0,     db1_dC31,      0,     db1_dC33,
    //              0,     0,     0,     0, db2_dC22,  db2_dC32,      0,     db2_dC32,  db2_dC33,
    //         -db1_dC11,  0, -db1_dC31, 0,     0,         0,     -db1_dC31,     0,     db3_dC33,
    //               0,    0,     0,     0, -db2_dC22, -db2_dC32,     0,     -db2_dC32, db4_dC33;

    // OPTION2: db_dC31 != db_dC13
    //           c11, c21,c31,c12, c22,    c32,    c13,      c23,      c33
    // db_dC << db1_dC11,  0, 0, 0,     0,     0,  db1_dC31,      0,     db1_dC33,
    //              0,     0, 0, 0, db2_dC22,  0,      0,     db2_dC32,  db2_dC33,
    //         -db1_dC11,  0, 0, 0,     0,     0,  -db1_dC31,     0,     db3_dC33,
    //               0,    0, 0, 0, -db2_dC22, 0,      0,     -db2_dC32, db4_dC33;

    Eigen::Matrix<double, 4,9> db_dC = Matrix::Zero(4,9);
    double f = std::sqrt(dC_(0,2)*dC_(0,2)-dC_(0,0)*dC_(2,2));
    double g = std::sqrt(dC_(1,2)*dC_(1,2)-dC_(1,1)*dC_(2,2));
    db_dC(0,0) = 1.0/f*(-1.0/2.0);
    db_dC(0,6) = (dC_(0,2)*1.0/f+1.0)/dC_(2,2);
    db_dC(0,8) = -1.0/(dC_(2,2)*dC_(2,2))*(dC_(0,2)+f)-(dC_(0,0)*1.0/f*(1.0/2.0))/dC_(2,2);
    db_dC(1,4) = 1.0/g*(-1.0/2.0);
    db_dC(1,7) = (dC_(1,2)*1.0/g+1.0)/dC_(2,2);
    db_dC(1,8) = -1.0/(dC_(2,2)*dC_(2,2))*(dC_(1,2)+g)-(dC_(1,1)*1.0/g*(1.0/2.0))/dC_(2,2);
    db_dC(2,0) = 1.0/f*(1.0/2.0);
    db_dC(2,6) = -(dC_(0,2)*1.0/f-1.0)/dC_(2,2);
    db_dC(2,8) = -1.0/(dC_(2,2)*dC_(2,2))*(dC_(0,2)-f)+(dC_(0,0)*1.0/f*(1.0/2.0))/dC_(2,2);
    db_dC(3,4) = 1.0/g*(1.0/2.0);
    db_dC(3,7) = -(dC_(1,2)*1.0/g-1.0)/dC_(2,2);
    db_dC(3,8) = -1.0/(dC_(2,2)*dC_(2,2))*(dC_(1,2)-g)+(dC_(1,1)*1.0/g*(1.0/2.0))/dC_(2,2);

    // cout << "DEBUG db_dC\n" << db_dC << endl << endl;
    // cout << "DEBUG db_dc\n" << db_dC * dC_dc << endl << endl;
    // *H = db_dC * dC_dc;
    *H = db_dC;
  }

  return AlignedBox2(xmin, ymin, xmax, ymax);
}

Eigen::Matrix<double, 5,9> DualConic::dc_dC(void) {
  static Eigen::Matrix<double, 5,9> dc_dC;
  dc_dC << 1,0,0,0,0,0,0,0,0,
           0,0,0,1,0,0,0,0,0,
           0,0,0,0,1,0,0,0,0,
           0,0,0,0,0,0,1,0,0,
           0,0,0,0,0,0,0,1,0;
  return dc_dC;
}

Eigen::Matrix<double, 9,5> DualConic::dC_dc(void) {
  static Eigen::Matrix<double, 9,5> dC_dc;
  dC_dc << 1,0,0,0,0,
           0,1,0,0,0,
           0,0,0,1,0,
           0,1,0,0,0,
           0,0,1,0,0,
           0,0,0,0,1,
           0,0,0,1,0,
           0,0,0,0,1,
           0,0,0,0,0;
  return dC_dc;
}


/* ************************************************************************* */
void DualConic::print(const std::string& s) const {
  cout << s << " : \n" << dC_ << endl;
}

/* ************************************************************************* */
bool DualConic::equals(const DualConic& other, double tol) const {
  return dC_.isApprox(other.dC_, tol);
}
    
} // namespace gtsam
