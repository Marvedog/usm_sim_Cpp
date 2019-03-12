#ifndef SIMULATION_NODE_H
#define SIMULATION_NODE_H

#include <ros/ros.h>

#include <usm_msgs/UsmState.h>
#include <usm_msgs/Control.h>

#include <Eigen/Dense>

#include <usm_sim/simulate_kinematics.h>

/**
 * Class SimulationNode wrapping kinematic simulator
 */

class SimulationNode {
  private:
    void controlCb(const usm_msgs::Control &msg);
    void timerCb(const ros::TimerEvent &e);
    void publishUsmState();

    ros::NodeHandle nh;
    ros::NodeHandle nhp;

    ros::Subscriber controlSub;
    ros::Publisher usmStatePub;

    ros::Timer timer;
    
    int steps;
    double ts;
    bool kinematicsOn;
    bool firstControl;
    std::string baseFrame;

    SimKinematics<Np, Nrot, Nq, Nxi> simKinematics;

    Eigen::Matrix<double, Nxi, 1> xi;
    Eigen::Matrix<double, Nzeta, 1> zeta;


  public:
    SimulationNode(const ros::NodeHandle &nh, const ros::NodeHandle &nhp);
};

#endif
