#include <usm_sim/simulation_node.h>
#include <usm_utils/containers.h>
#include <usm_utils/conversions.h>

#include <tf/tf.h>
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
, firstPose(false)
, firstControl(false)
{
  // Snake configuration setup
  Utils::Config config;
  this->nh.getParam("/snake_config", config.snake);
  this->nh.getParam("/d", config.d);
  this->nh.getParam("/a", config.a);
  this->nh.getParam("/alpha", config.alpha);
  this->nh.getParam("/theta", config.theta);
  this->nh.getParam("/base_frame", config.base);
  
  this->usm = Utils::Snake(config);

  // Simulation data
  this->nhp.getParam("/simulation/sim_steps", this->steps);
  this->nhp.getParam("/simulation/h", this->ts);
  this->nhp.getParam("/simulation/sim_kinematics", this->kinematicsOn);

  // Set up kinematic simulator
  this->nxi = 6 + this->usm.getJoints();

  this->xi.resize(this->nxi);
  this->zeta.resize(this->nxi);

  mutex.lock();  
  this->xi = Eigen::VectorXd::Zero(this->nxi);
  this->zeta = Eigen::VectorXd::Zero(this->nxi);
  mutex.unlock();

  //std::string controlTopic, usmStateTopic;
  //this->nh.getParam("/control/plant_input", controlTopic);
  //this->nh.getParam("/state_estimation/usm_state", usmStateTopic);


  // Wait for start pose
  this->initSub = this->nh.subscribe("/simulation/pose_init", 1, &SimulationNode::initCb, this);
  
  ros::Rate d(1);
  while (!this->firstPose) {
    d.sleep();
    ros::spinOnce();
  }

  this->simKinematics = SimKinematics(this->ts, this->usm.getJoints(), this->xi.block(0, 0, 3, 1));


  this->controlSub = this->nh.subscribe("/control/plant_input", 1, &SimulationNode::controlCb, this);
  this->usmStatePub = this->nh.advertise<usm_msgs::UsmState>("/state_estimation/usm_state", 1);
  
  // Set up timer cb
  while (!this->firstControl) {
    d.sleep();
    ros::spinOnce();
    this->publishUsmState();
  }
  
  this->timer = this->nh.createTimer(ros::Duration(this->steps*this->ts), &SimulationNode::timerCb, this);
}


/**
 * Timer callback running kinematic simulations and publishing \
 * simulation output
 */

void
SimulationNode::timerCb(const ros::TimerEvent &e)
{
  // Add condition
  if (true)
    this->xi = this->simKinematics.sim(this->zeta, this->steps);

  // Check for nan
  mutex.lock();
  if (this->xi[0] != this->xi[0])
    std::cout << "Simulation:: Nan alarm!!" << std::endl;
  mutex.unlock();

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
  mutex.lock();
  Eigen::Matrix3d R_Ib = Geometry::Rzyx(this->xi[3], this->xi[4], this->xi[5]); 
  Eigen::Vector3d V_b_d;
  V_b_d <<  msg.guidance.v.linear.x, msg.guidance.v.linear.y, msg.guidance.v.linear.z;

  this->zeta.block(0, 0, 3, 1) = R_Ib*V_b_d;

  // Angular velocity (se(3))
  this->zeta[3] = msg.guidance.v.angular.x;
  this->zeta[4] = msg.guidance.v.angular.y;
  this->zeta[5] = msg.guidance.v.angular.z;

  // Joint velocities
  if ((this->zeta.size() - 6) != msg.guidance.dq.theta.size())
    std::cout << "Erroneous number of joint angles! Shutting down simulation." << std::endl;
  
  for (int i = 0; i < this->usm.getJoints(); i++)
    this->zeta[6+i] = msg.guidance.dq.theta[i].angle;

  if (!this->firstControl)
    this->firstControl = true;
  mutex.unlock();
}


/**
 *
 */

void
SimulationNode::initCb(const usm_msgs::UsmState &msg)
{
  Utils::EigenTf T_Ib;
  Utils::geometryMsgPoseToEigenTf(msg.x.p, T_Ib);

  T_Ib.tf.print();

  mutex.lock();
  this->xi.block(0, 0, 3, 1) = T_Ib.tf.getOrigin();
  mutex.unlock();
  
  this->firstPose = true;
}


/**
 * Publish usm state message with pose and velocities
 */

void
SimulationNode::publishUsmState()
{
  usm_msgs::UsmState usmState;
  
  mutex.lock();
  // Pose
  usmState.x.p.position.x = this->xi[0];
  usmState.x.p.position.y = this->xi[1];
  usmState.x.p.position.z = this->xi[2];
   
  Eigen::Vector3d eul = this->xi.block(3, 0, 3, 1);
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(eul[0], eul[1], eul[2]);
  usmState.x.p.orientation = quat;

  usmState.x.q.theta.resize(this->usm.getJoints());
  for (int i = 3 + 3; i < this->nxi; i++)
    usmState.x.q.theta[i-6].angle = this->xi[i];

  // Velocities
  usmState.dx.v.linear.x = this->zeta[0];
  usmState.dx.v.linear.y = this->zeta[1];
  usmState.dx.v.linear.z = this->zeta[2];
   
  usmState.dx.v.angular.x = this->zeta[3];
  usmState.dx.v.angular.y = this->zeta[4];
  usmState.dx.v.angular.z = this->zeta[5];

  usmState.dx.dq.theta.resize(this->usm.getJoints());
  for (int i = 3 + 3; i < this->nxi; i++)
    usmState.dx.dq.theta[i-6].angle = this->zeta[i];

  mutex.unlock();
  this->usmStatePub.publish(usmState);
}


int
main(int argc, char** argv)
{
  ros::init(argc, argv, "simulation_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nhp("~");
  SimulationNode simulationNode(nh, nhp);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  ros::waitForShutdown();
}
