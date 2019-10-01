#include "ekf_loose_integration_alg_node.h"

EkfLooseIntegrationAlgNode::EkfLooseIntegrationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<EkfLooseIntegrationAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  acc_reading_       = Eigen::Vector3d::Zero();
  acc_corrected_     = Eigen::Vector3d::Zero();

  acc_bias_          = Eigen::Vector3d::Zero();
  acc_scale_factor_  = Eigen::Matrix3d::Zero();
  acc_misaligment_   = Eigen::Matrix3d::Zero();

  gyro_reading_      = Eigen::Vector3d::Zero();
  gyro_corrected_    = Eigen::Vector3d::Zero();

  gyro_bias_         = Eigen::Vector3d::Zero();
  gyro_scale_factor_ = Eigen::Matrix3d::Zero();
  gyro_misaligment_  = Eigen::Matrix3d::Zero();

  // Using calibration results obtained with imu_tk
  acc_bias_(0) = 1.00228;
  acc_bias_(1) = -1.44683;
  acc_bias_(2) = -2.11036;

  acc_scale_factor_(0,0) = 1.00871;
  acc_scale_factor_(1,1) = 0.999859;
  acc_scale_factor_(2,2) = 0.976556;

  acc_misaligment_(0,0) = 1.0;
  acc_misaligment_(0,1) = 0.0317041;
  acc_misaligment_(0,2) = -0.0146073;

  acc_misaligment_(1,1) = 1.0;
  acc_misaligment_(1,2) = 0.00196899;

  acc_misaligment_(2,2) = 1.0;


  gyro_bias_(0) = 0.000473265;
  gyro_bias_(1) = -0.000742748;
  gyro_bias_(2) = -4.33131 * 1e-6;

  gyro_scale_factor_(0,0) = 0.96651;
  gyro_scale_factor_(1,1) = 0.993987;
  gyro_scale_factor_(2,2) = 1.00491;

  gyro_misaligment_(0,0) = 1.0;
  gyro_misaligment_(0,1) = -0.0181747;
  gyro_misaligment_(0,2) = -0.00295471;

  gyro_misaligment_(1,0) = 0.0187744;
  gyro_misaligment_(1,1) = 1.0;
  gyro_misaligment_(1,2) = -0.0184467;

  gyro_misaligment_(2,0) = -0.00744203;
  gyro_misaligment_(2,1) = -0.00940402;
  gyro_misaligment_(2,2) = 1.0;



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

  acc_reading_(0) = IMU_msg->linear_acceleration.x;
  acc_reading_(1) = IMU_msg->linear_acceleration.y;
  acc_reading_(2) = IMU_msg->linear_acceleration.z;

  acc_corrected_ = acc_misaligment_ * acc_scale_factor_ * (acc_reading_ + acc_bias_);

  gyro_reading_(0) = IMU_msg->angular_velocity.x;
  gyro_reading_(1) = IMU_msg->angular_velocity.y;
  gyro_reading_(2) = IMU_msg->angular_velocity.z;

  gyro_corrected_ = gyro_misaligment_ * gyro_scale_factor_ * (gyro_reading_ + gyro_bias_);

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
