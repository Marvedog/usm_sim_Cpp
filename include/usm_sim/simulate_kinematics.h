#ifndef SIM_KINEMATICS_H
#define SIM_KINEMATICS_H

#include <usm_utils/geometry.h>
#include <usm_sim/common.h>

#include <iostream>

/**
 * Class for simulating kinematics of an usm
 * @template Np is the position dimension
 * @template Nrot is rotation dimension
 * @template Nq is the number of joints
 */

template<int Np, int Nrot, int Nq, int Nxi>
class SimKinematics {
  private:
    double h;
    
    Eigen::Matrix<double, Np, 1> p;
    Eigen::Matrix<double, Nrot, Nrot> R;
    Eigen::Matrix<double, Nq, 1> q;

  public:
    SimKinematics();
    SimKinematics(const double h);
   
    void setStepSize(const double h);

    Eigen::Matrix<double, Nxi, 1> sim(const Eigen::Matrix<double, Nxi, 1> &zeta, const int steps);
};


/**
 * Default constructor
 */

template<int Np, int Nrot, int Nq, int Nxi>
SimKinematics<Np, Nrot, Nq, Nxi>::SimKinematics()
: h(0.01)
, p(Eigen::Matrix<double, Np, 1>::Zero())
, R(Eigen::Matrix<double, Nrot, Nrot>::Zero())
, q(Eigen::Matrix<double, Nq, 1>::Zero())
{}


/**
 * Constructor with specified step size 
 */

template<int Np, int Nrot, int Nq, int Nxi>
SimKinematics<Np, Nrot, Nq, Nxi>::SimKinematics(const double h)
: h(h)
, p(Eigen::Matrix<double, Np, 1>::Zero())
, R(Eigen::Matrix<double, Nrot, Nrot>::Zero())
, q(Eigen::Matrix<double, Nq, 1>::Zero())
{}


/**
 * Utility function to set step size
 */

template<int Np, int Nrot, int Nq, int Nxi>
void
SimKinematics<Np, Nrot, Nq, Nxi>::setStepSize(const double h)
{
  this->h = h;
}


/**
 * Simulate kinematics for a fixed number of steps and return state vector
 */

template<int Np, int Nrot, int Nq, int Nxi>
Eigen::Matrix<double, Nxi, 1>
SimKinematics<Np, Nrot, Nq, Nxi>::sim(const Eigen::Matrix<double, Nxi, 1> &zeta, const int steps) 
{
  std::cout << "In Sim :: " << std::endl;

  // Euler integration 
  Eigen::Matrix<double, Nrot, Nrot> Su = Geometry::skew(zeta.block(Np, 0, Nrot, 1));
  for (int i = 0; i < steps; i++) {
    
    this->p = this->p + this->h*(zeta.block(0, 0, Np, 1));

    this->R = this->R + this->h*(Su*this->R);

    this->q = this->q + this->h*(zeta.block(Np+Nrot, 0, Nq, 1));
  }

  // Return
  Eigen::Matrix<double, Nxi, 1> xi;

  xi.block(0, 0, Np, 1) = this->p;
  xi.block(3, 0, Nrot, 1) = Geometry::R2euler(this->R); 
  xi.block(6, 0, Nq, 1) = this->q;
  return xi;
}

#endif
