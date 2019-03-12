#ifndef SIMULATION_NODE_H
#define SIMULATION_NODE_H

#include <ros/ros.h>

#include <usm_msgs/Control.h>

#include <usm_utils/snake.h>
#include <usm_sim/simulate_kinematics.h>

#include <Eigen/Dense>
#include <mutex>

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
    int nxi; 
    double ts;
    bool kinematicsOn;
    bool firstControl;

    Utils::Snake usm;
    SimKinematics simKinematics;

    Eigen::VectorXd xi;
    Eigen::VectorXd zeta;

    std::mutex mutex;
  public:
    SimulationNode(const ros::NodeHandle &nh, const ros::NodeHandle &nhp);
};

#endif
