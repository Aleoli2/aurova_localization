#include "ekf_fusion_alg_node.h"

EkfFusionAlgNode::EkfFusionAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<EkfFusionAlgorithm>()
{

  //init class attributes if necessary
  this->kalman_config_.x_ini = 0.1; //set first amcl variances
  this->kalman_config_.y_ini = 0.1;
  this->kalman_config_.theta_ini = 0.01;
  this->kalman_config_.x_model = 0.05; //0.016 / 9
  this->kalman_config_.y_model = 0.05;
  this->kalman_config_.theta_model = 0.01; //0.00037
  this->kalman_config_.outlier_mahalanobis_threshold = 1000000;
  this->ekf_ = new CEkf(this->kalman_config_);
  this->loop_rate_ = 10; //in [Hz]
  this->flag_send_pose = false;
  this->flag_plot_pose = false;

  // [init publishers]
  this->pose_publisher_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped
      > ("/initialpose", 1);
  this->pose_plot_pub_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped
      > ("/initialpose_plot", 1);

  // [init subscribers]
  this->amcl_pose_sub_ = this->public_node_handle_.subscribe("/amcl_pose", 1, &EkfFusionAlgNode::cb_getPoseMsg, this);
  this->odom_gps_sub_ = this->public_node_handle_.subscribe("/odometry_gps", 1, &EkfFusionAlgNode::cb_getGpsOdomMsg,
                                                            this);
  this->odom_raw_sub_ = this->public_node_handle_.subscribe("/odometry", 1, &EkfFusionAlgNode::cb_getRawOdomMsg, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

EkfFusionAlgNode::~EkfFusionAlgNode(void)
{
  // [free dynamic memory]
}

void EkfFusionAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  if (this->flag_plot_pose)
  {
    this->pose_plot_pub_.publish(this->pose_plot_);
    this->flag_plot_pose = false;
  }
  if (this->flag_send_pose)
  {
    this->pose_publisher_.publish(this->pose_filtered_);
    this->flag_send_pose = false;
  }
}

/*  [subscriber callbacks] */
void EkfFusionAlgNode::cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  this->alg_.lock();

  ekf::SlamObservation obs;

  double min_std_x = 0.1, min_std_y = 0.1, min_std_theta = 0.017 * 2; /// GET FROM PARAMETER !!!!
  double var_max = 4 * 4, var_max_theta = 0.017 * 10 * 0.017 * 10; /// GET FROM PARAMETER !!!!

  //get yaw information
  tf::Quaternion q(pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y,
                   pose_msg->pose.pose.orientation.z, pose_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //set observation
  obs.x = pose_msg->pose.pose.position.x;
  obs.y = pose_msg->pose.pose.position.y;
  obs.theta = yaw;
  obs.sigma_x = pose_msg->pose.covariance[0];
  obs.sigma_y = pose_msg->pose.covariance[7];
  obs.sigma_theta = pose_msg->pose.covariance[35];

  assert(
      !isnan(obs.x) && !isnan(obs.y) && !isnan(obs.theta) && !isnan(obs.sigma_x) && !isnan(obs.sigma_y)
          && !isnan(obs.sigma_theta) && "Error in EkfFusionAlgNode::cb_getPoseMsg: nan value");

  //saturation of amcl variances
  if (obs.sigma_x < min_std_x * min_std_x)
  {
    obs.sigma_x = min_std_x * min_std_x;
  }
  if (obs.sigma_y < min_std_y * min_std_y)
  {
    obs.sigma_y = min_std_y * min_std_y;
  }
  if (obs.sigma_theta < min_std_theta * min_std_theta)
  {
    obs.sigma_theta = min_std_theta * min_std_theta;
  }

  if (obs.sigma_x > var_max || obs.sigma_x > var_max || obs.sigma_theta > var_max_theta)
  {
    Eigen::Matrix<double, 3, 1> state;
    Eigen::Matrix<double, 3, 3> covariance;

    this->ekf_->getStateAndCovariance(state, covariance);

    //update ros message structure
    this->pose_filtered_.header.frame_id = "/map"; //load from parameter
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, state(2));
    this->pose_filtered_.pose.pose.position.x = state(0);
    this->pose_filtered_.pose.pose.position.y = state(1);
    this->pose_filtered_.pose.pose.orientation.x = quaternion[0];
    this->pose_filtered_.pose.pose.orientation.y = quaternion[1];
    this->pose_filtered_.pose.pose.orientation.z = quaternion[2];
    this->pose_filtered_.pose.pose.orientation.w = quaternion[3];
    this->pose_filtered_.pose.covariance[0] = covariance(0, 0);
    this->pose_filtered_.pose.covariance[1] = covariance(0, 1);
    this->pose_filtered_.pose.covariance[5] = covariance(0, 2);
    this->pose_filtered_.pose.covariance[6] = covariance(1, 0);
    this->pose_filtered_.pose.covariance[7] = covariance(1, 1);
    this->pose_filtered_.pose.covariance[11] = covariance(1, 2);
    this->pose_filtered_.pose.covariance[30] = covariance(2, 0);
    this->pose_filtered_.pose.covariance[31] = covariance(2, 1);
    this->pose_filtered_.pose.covariance[35] = covariance(2, 2);
    this->flag_send_pose = true;
  }
  else
  {
    this->ekf_->update(obs);
  }

  this->alg_.unlock();
}

void EkfFusionAlgNode::cb_getGpsOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  this->alg_.lock();

  ekf::GnssObservation obs;
  double min_std_x = 0.9, min_std_y = 0.9; /// GET FROM PARAMETER !!!!

  //get yaw information
  tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                   odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //set observation
  obs.x = odom_msg->pose.pose.position.x;
  obs.y = odom_msg->pose.pose.position.y;
  obs.theta = yaw;
  obs.sigma_x = odom_msg->pose.covariance[0];
  obs.sigma_y = odom_msg->pose.covariance[7];
  obs.sigma_theta = odom_msg->pose.covariance[35];

  //saturation of gps variances
  if (obs.sigma_x < min_std_x * min_std_x)
  {
    obs.sigma_x = min_std_x * min_std_x;
  }
  if (obs.sigma_y < min_std_y * min_std_y)
  {
    obs.sigma_y = min_std_y * min_std_y;
  }

  this->ekf_->update(obs);

  this->alg_.unlock();
}

void EkfFusionAlgNode::cb_getRawOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  static double x_prev = 0.0;
  static double y_prev = 0.0;
  static double theta_prev = 0.0;

  this->alg_.lock();

  ekf::OdomAction act;

  //get yaw information
  tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                   odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //set observation
  act.delta_x = odom_msg->pose.pose.position.x - x_prev;
  act.delta_y = odom_msg->pose.pose.position.y - y_prev;
  act.delta_theta = yaw - theta_prev;

  this->ekf_->predict(act);

  //for next step
  x_prev = odom_msg->pose.pose.position.x;
  y_prev = odom_msg->pose.pose.position.y;
  theta_prev = yaw;

  Eigen::Matrix<double, 3, 1> state;
  Eigen::Matrix<double, 3, 3> covariance;

  this->ekf_->getStateAndCovariance(state, covariance);

  ////////////////////////////////////////////////////////////////////////////////
  //update ros message structure
  this->pose_plot_.header.frame_id = "/map"; //load from parameter
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, state(2));
  this->pose_plot_.pose.pose.position.x = state(0);
  this->pose_plot_.pose.pose.position.y = state(1);
  this->pose_plot_.pose.pose.orientation.x = quaternion[0];
  this->pose_plot_.pose.pose.orientation.y = quaternion[1];
  this->pose_plot_.pose.pose.orientation.z = quaternion[2];
  this->pose_plot_.pose.pose.orientation.w = quaternion[3];
  this->pose_plot_.pose.covariance[0] = covariance(0, 0);
  this->pose_plot_.pose.covariance[1] = covariance(0, 1);
  this->pose_plot_.pose.covariance[5] = covariance(0, 2);
  this->pose_plot_.pose.covariance[6] = covariance(1, 0);
  this->pose_plot_.pose.covariance[7] = covariance(1, 1);
  this->pose_plot_.pose.covariance[11] = covariance(1, 2);
  this->pose_plot_.pose.covariance[30] = covariance(2, 0);
  this->pose_plot_.pose.covariance[31] = covariance(2, 1);
  this->pose_plot_.pose.covariance[35] = covariance(2, 2);
  this->flag_plot_pose = true;
  ////////////////////////////////////////////////////////////////////////////////

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void EkfFusionAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void EkfFusionAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < EkfFusionAlgNode > (argc, argv, "ekf_fusion_alg_node");
}
