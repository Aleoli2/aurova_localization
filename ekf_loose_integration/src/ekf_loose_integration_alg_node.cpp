#include "ekf_loose_integration_alg_node.h"

EkfLooseIntegrationAlgNode::EkfLooseIntegrationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<EkfLooseIntegrationAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  rover_state_ = Eigen::Matrix<double, 9, 1>::Zero();

  flag_first_imu_msg_received_ = false;
  previous_timestamp_ = 0.0;
  current_timestamp_ = 0.0;

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
  
  this->odom_publisher_ = this->public_node_handle_.advertise < nav_msgs::Odometry > ("/estimated_odometry", 1);

  // [init subscribers]
  this->estimated_ackermann_subscriber_ = this->public_node_handle_.subscribe(
      "/estimated_ackermann_state", 1, &EkfLooseIntegrationAlgNode::cb_ackermannState, this);
  
  this->imu_subscriber_ = this->public_node_handle_.subscribe(
      "/imu/data", 1, &EkfLooseIntegrationAlgNode::cb_imuData, this);

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
  this->estimated_odom_.pose.pose.position.x = rover_state_(0); //acc_corrected_(0);
  this->estimated_odom_.pose.pose.position.y = rover_state_(1); //acc_corrected_(1);
  this->estimated_odom_.pose.pose.position.z = rover_state_(2); //acc_corrected_(2);
  
  // Converting to quarternion to fill the ROS message
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(rover_state_(6),
                                                          rover_state_(7),
                                                          rover_state_(8));

  this->estimated_odom_.pose.pose.orientation.x = quaternion[0];
  this->estimated_odom_.pose.pose.orientation.y = quaternion[1];
  this->estimated_odom_.pose.pose.orientation.z = quaternion[2];
  this->estimated_odom_.pose.pose.orientation.w = quaternion[3];

  this->estimated_odom_.twist.twist.linear.x = rover_state_(3);
  this->estimated_odom_.twist.twist.linear.y = rover_state_(4);
  this->estimated_odom_.twist.twist.linear.z = rover_state_(5);


  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->odom_publisher_.publish(this->estimated_odom_);
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

  this->estimated_odom_.header = IMU_msg->header;
  //this->estimated_odom_.child_frame_id = IMU_msg->child_frame_id;

  //std::cout << "IMU message received!" << std::endl;

  current_timestamp_ = IMU_msg->header.stamp.sec + (IMU_msg->header.stamp.nsec * 1e-9);

  if (flag_first_imu_msg_received_)
  {
    double previous_x = rover_state_(0);
    double previous_y = rover_state_(1);
    double previous_z = rover_state_(2);

    double previous_vx = rover_state_(3);
    double previous_vy = rover_state_(4);
    double previous_vz = rover_state_(5);

    double previous_acc_x = acc_corrected_(0);
    double previous_acc_y = acc_corrected_(1);
    double previous_acc_z = acc_corrected_(2);

    double dt = current_timestamp_ - previous_timestamp_;
    //std::cout << "Delta t = " << dt << std::endl;

    double delta_vx = previous_acc_x * dt;
    double delta_vy = previous_acc_y * dt;
    double delta_vz = previous_acc_z * dt;

    double current_vx = previous_vx + delta_vx;
    double current_vy = previous_vy + delta_vy;
    double current_vz = previous_vz + delta_vz;

    double dt_squared = dt * dt;
    double current_x = previous_x + previous_vx * dt + 0.5 * previous_acc_x * dt_squared;
    double current_y = previous_y + previous_vy * dt + 0.5 * previous_acc_y * dt_squared;
    double current_z = previous_z + previous_vz * dt + 0.5 * previous_acc_z * dt_squared;

    double previous_roll  = rover_state_(6);
    double previous_pitch = rover_state_(7);
    double previous_yaw   = rover_state_(8);

    double previous_roll_rate  = gyro_corrected_(0);
    double previous_pitch_rate = gyro_corrected_(1);
    double previous_yaw_rate   = gyro_corrected_(2);

    double delta_roll  = previous_roll_rate  * dt;
    double delta_pitch = previous_pitch_rate * dt;
    double delta_yaw   = previous_yaw_rate   * dt;

    double current_roll  = previous_roll  + delta_roll;
    double current_pitch = previous_pitch + delta_pitch;
    double current_yaw   = previous_yaw   + delta_yaw;

    // Update state
    rover_state_(0) = current_x;
    rover_state_(1) = current_y;
    rover_state_(2) = current_z;

    rover_state_(3) = current_vx;
    rover_state_(4) = current_vy;
    rover_state_(5) = current_vz;

    rover_state_(6) = current_roll;
    rover_state_(7) = current_pitch;
    rover_state_(8) = current_yaw;

  }

  acc_reading_(0) = IMU_msg->linear_acceleration.x;
  acc_reading_(1) = IMU_msg->linear_acceleration.y;
  acc_reading_(2) = IMU_msg->linear_acceleration.z;

  acc_corrected_ = acc_misaligment_ * acc_scale_factor_ * (acc_reading_ - acc_bias_);

  acc_corrected_(2) += 9.8; // to cancel out the gravity vector

  gyro_reading_(0) = IMU_msg->angular_velocity.x;
  gyro_reading_(1) = IMU_msg->angular_velocity.y;
  gyro_reading_(2) = IMU_msg->angular_velocity.z;

  gyro_corrected_ = gyro_misaligment_ * gyro_scale_factor_ * (gyro_reading_ - gyro_bias_);

  previous_timestamp_ = current_timestamp_;

  if(!flag_first_imu_msg_received_) flag_first_imu_msg_received_ = true;

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
