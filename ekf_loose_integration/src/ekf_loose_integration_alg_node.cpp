#include "ekf_loose_integration_alg_node.h"

EkfLooseIntegrationAlgNode::EkfLooseIntegrationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<EkfLooseIntegrationAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->pose_publisher_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped
      > ("/estimated_pose", 1);
  
  // [init subscribers]
  this->estimated_ackermann_subscriber_ = this->public_node_handle_.subscribe(
      "/estimated_ackermann_state", 1, &EkfLooseIntegrationAlgNode::cb_ackermannState, this);
  
  this->imu_subscriber_ = this->public_node_handle_.subscribe(
      "/imu_data", 1, &EkfLooseIntegrationAlgNode::cb_imuData, this);

  this->AMCL_pose_sub_ = this->public_node_handle_.subscribe(
      "/amcl_pose", 1, &EkfLooseIntegrationAlgNode::cb_AMCLPose, this);

  this->odom_GNSS_sub_ = this->public_node_handle_.subscribe(
      "/odometry_gps", 1, &EkfLooseIntegrationAlgNode::cb_GNSSOdom, this);

  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

EkfLooseIntegrationAlgNode::~EkfLooseIntegrationAlgNode(void)
{
  // [free dynamic memory]
}

void EkfLooseIntegrationAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void EkfLooseIntegrationAlgNode::cb_ackermannState(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr& estimated_ackermann_state_msg)
{
  this->alg_.lock();

  this->estimated_ackermann_state_.drive.speed = estimated_ackermann_state_msg->drive.speed;
  this->estimated_ackermann_state_.drive.steering_angle = estimated_ackermann_state_msg->drive.steering_angle;

  this->alg_.unlock();
}

void EkfLooseIntegrationAlgNode::cb_imuData(const sensor_msgs::Imu::ConstPtr& IMU_msg)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void EkfLooseIntegrationAlgNode::cb_AMCLPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void EkfLooseIntegrationAlgNode::cb_GNSSOdom(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  this->alg_.lock();

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void EkfLooseIntegrationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void EkfLooseIntegrationAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<EkfLooseIntegrationAlgNode>(argc, argv, "ekf_loose_integration_alg_node");
}
