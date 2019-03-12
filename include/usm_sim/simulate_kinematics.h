#ifndef SIM_KINEMATICS_H
#define SIM_KINEMATICS_H

#include <usm_utils/geometry.h>

#include <iostream>


/**
 * Class for simulating kinematics of an usm
 */

class SimKinematics {
  private:
    int nq;
    int nxi;
    double h;
     
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    Eigen::VectorXd q;

  public:
    SimKinematics();
    SimKinematics(const double h, const int joints);
   
    void setStepSize(const double h);

    Eigen::VectorXd sim(const Eigen::VectorXd &zeta, const int steps);
};


/**
 * Default constructor
 */

SimKinematics::SimKinematics()
: h(0.01)
, p(Eigen::Matrix<double, 3, 1>::Zero())
, R(Eigen::Matrix<double, 3, 3>::Zero())
{}


/**
 * Constructor with specified step size 
 */

SimKinematics::SimKinematics(const double h, const int joints)
: h(h)
, p(Eigen::Vector3d::Zero())
, R(Eigen::Matrix3d::Zero())
, nq(joints)
, nxi(6+joints)
{
  this->q.resize(joints);
  this->q = Eigen::VectorXd::Zero(joints);
}


/**
 * Utility function to set step size
 */

void
SimKinematics::setStepSize(const double h)
{
  this->h = h;
}


/**
 * Simulate kinematics for a fixed number of steps and return state vector
 */

Eigen::VectorXd
SimKinematics::sim(const Eigen::VectorXd &zeta, const int steps) 
{
  std::cout << "In Sim :: " << std::endl;
  
  // Dimensioning check zeta
  if (zeta.size() != this->nxi) {
    std::cerr << "Conflicting dimensions; zeta:: " << zeta.rows() << std::endl;
    return Eigen::VectorXd::Zero(this->nxi); 
  }

  // Euler integration 
  Eigen::Matrix3d Su = Geometry::skew(zeta.block(3, 0, 3, 1));
  for (int i = 0; i < steps; i++) {
    
    this->p = this->p + this->h*(zeta.block(0, 0, 3, 1));

    this->R = this->R + this->h*(Su*this->R);

    this->q = this->q + this->h*(zeta.block(3+3, 0, this->nq, 1));
  }

  // Return
  Eigen::VectorXd xi(this->nxi);

  xi.block(0, 0, 3, 1) = this->p;
  xi.block(3, 0, 3, 1) = Geometry::R2euler(this->R); 
  xi.block(6, 0, this->nq, 1) = this->q;
  return xi;
}

#endif
