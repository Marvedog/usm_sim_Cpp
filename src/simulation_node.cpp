#include <usm_sim/simulation_node.h>

#include <tf/transform_datatypes.h>


/**
 * Publish odometry transform
 * @param msg is the current pose
 * @param frame is the pose frame

void
SimulationNode::publishOdometry(const usm_msgs::UsmState &msg, const std::string &frame)
{
  tf::Vector3 trans = tf::Vector3(msg.x.p.position.x, msg.x.p.position.y, msg.x.p.position.z);
  tf::Quaternion rot;
  tf::quaternionMsgToTF(msg.x.p.orientation, rot);
  
  tf::Transform tfOdom(rot, trans);
  this->bc.sendTransform(tf::StampedTransform(tfOdom, ros::Time::now(), this->baseFrame, frame));
}
*/


//TODO: Add link transforms

/**
 * Default constructor initializing kinematic simulator
 */

SimulationNode::SimulationNode(const ros::NodeHandle &nh, const ros::NodeHandle &nhp)
: nh(nh)
, nhp(nhp)
, xi(Eigen::Matrix<double, Nxi, 1>::Zero())
, zeta(Eigen::Matrix<double, Nzeta, 1>::Zero())
, firstControl(false)
{
  // Simulation data
  this->nhp.getParam("/simulation/sim_steps", this->steps);
  this->nhp.getParam("/simulation/h", this->ts);
  this->nhp.getParam("/simulation/sim_kinematics", this->kinematicsOn);
  this->nhp.getParam("/simulation/base_frame", this->baseFrame);

  // Set up kinematic simulator
  this->simKinematics = SimKinematics<Np, Nrot, Nq, Nxi>(this->ts);

  std::string controlTopic, usmStateTopic;
  this->nh.getParam("/control/plant_input", controlTopic);
  this->nh.getParam("/state_estimation/usm_state", usmStateTopic);

  this->controlSub = this->nh.subscribe("/control/plant_input", 1, &SimulationNode::controlCb, this);
  this->usmStatePub = this->nh.advertise<usm_msgs::UsmState>("/state_estimation/usm_state", 1);
  
  // Set up timer cb
  while (!this->firstControl)
    ros::spinOnce();

  this->timer = this->nh.createTimer(ros::Duration(this->steps*this->ts), &SimulationNode::timerCb, this);

  std::cout << "Simulation set up" << std::endl;
}


/**
 * Timer callback running kinematic simulations and publishing \
 * simulation output
 */

void
SimulationNode::timerCb(const ros::TimerEvent &e)
{
  // Add condition
  if (true) {
    //std::cout << "Simulation::timerCb:: " << std::endl;
    //std::cout << "xi size:: pre sim" << this->xi.size() << std::endl;
    //std::cout << "zeta size:: " << this->zeta.size() << std::endl;
    this->xi = this->simKinematics.sim(this->zeta, this->steps);
  }

  // Check for nan
  if (this->xi[0] != this->xi[0]) {
    std::cerr << "Simulation:: Nan alarm!!" << std::endl;
    ros::shutdown();
  }

  this->publishUsmState();
}


/**
 * Velocity callback receiving desired input velocity from \
 * guidance and inverse kinematics algorithms
 */

void
SimulationNode::controlCb(const usm_msgs::Control &msg)
{
  // Linear velocity
  Eigen::Matrix<double, Np, 1> vTr;
  vTr[0] = msg.guidance.v.linear.x;
  vTr[1] = msg.guidance.v.linear.y;
  vTr[2] = msg.guidance.v.linear.z;

  // Angular velocity (se(3))
  //std::cout << "Angular velocities!! " << std::endl;
  Eigen::Matrix<double, Nrot, 1> vRot;
  vRot[0] = msg.guidance.v.angular.x;
  vRot[1] = msg.guidance.v.angular.y;
  vRot[2] = msg.guidance.v.angular.z;

  // Joint velocities
  if (Nq != msg.guidance.dq.theta.size()) {
    std::cerr << "Erroneous number of joint angles! Shutting down simulation." << std::endl;
    ros::shutdown();
  }

  //std::cout << "Joint velocities!! " << std::endl;
  
  Eigen::Matrix<double, Nq, 1> dq;
  for (int i = 0; i < Nq; i++)
    dq[i] = msg.guidance.dq.theta[i].angle;

  this->zeta.block<Np,1>(0,0) = vTr;
  this->zeta.block<Nrot,1>(Np,0) = vRot;
  this->zeta.block<Nq,1>(Np+Nrot,0) = dq;

  //std::cout << "Published state!! " << std::endl;

  if (!this->firstControl)
    this->firstControl = true;
}


/**
 * Publish usm state message with pose and velocities
 */

void
SimulationNode::publishUsmState()
{
  usm_msgs::UsmState usmState;
  
  // Pose
  usmState.x.p.position.x = xi[0];
  usmState.x.p.position.y = xi[1];
  usmState.x.p.position.z = xi[2];
   
  Eigen::Matrix<double, Nrot, 1> eul = xi.block<Nrot, 1>(Np,0);
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(eul[0], eul[1], eul[2]);
  usmState.x.p.orientation = quat;

  usmState.x.q.theta.resize(Nq);
  for (int i = Np + Nrot; i < Nxi; i++)
    usmState.x.q.theta[i].angle = xi[i];

  // Velocities
  usmState.dx.v.linear.x = zeta[0];
  usmState.dx.v.linear.y = zeta[1];
  usmState.dx.v.linear.z = zeta[2];
   
  usmState.dx.v.angular.x = zeta[0];
  usmState.dx.v.angular.y = zeta[1];
  usmState.dx.v.angular.z = zeta[2];

  usmState.dx.dq.theta.resize(Nq);
  for (int i = Np + Nrot; i < Nxi; i++)
    usmState.dx.dq.theta[i].angle = zeta[i];

  this->usmStatePub.publish(usmState);
}

  
int
main(int argc, char** argv)
{
  ros::init(argc, argv, "simulation_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nhp("~");
  SimulationNode simulationNode(nh, nhp);
  ros::spin();
  return 0;
}
