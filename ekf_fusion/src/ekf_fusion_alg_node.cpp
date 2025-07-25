#include "ekf_fusion_alg_node.h"

using std::placeholders::_1;

EkfFusionAlgNode::EkfFusionAlgNode(void) :
    Node("ekf_fusion")
{
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  //init class attributes if necessary
  this->flag_plot_pose_ = false;
  this->get_parameter("frame_id", this->frame_id_);
  this->get_parameter("child_id", this->child_id_);
  this->get_parameter("x_model", this->kalman_config_.x_model);
  this->get_parameter("y_model", this->kalman_config_.y_model);
  this->get_parameter("theta_model", this->kalman_config_.theta_model);
  this->get_parameter("outlier_mahalanobis",
                                     this->kalman_config_.outlier_mahalanobis_threshold);
  this->get_parameter("min_speed", this->min_speed_);
  this->get_parameter("is_simulation", this->is_simulation_);

  auto parameter_callback = this->add_on_set_parameters_callback(std::bind(&EkfFusionAlgNode::node_config_update, this, _1));

  this->ekf_ = new CEkf(this->kalman_config_);

  // [init publishers]
  this->plot_pose_pub_ = this->create_publisher< geometry_msgs::msg::PoseWithCovarianceStamped> ("/pose_plot", 1);

  // [init subscribers]
  this->init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, std::bind(&EkfFusionAlgNode::cb_getInitPoseMsg,this, _1));
  this->odom_gps_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry_gps", 1, std::bind(&EkfFusionAlgNode::cb_getGpsOdomMsg, this, _1));
  this->odom_raw_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&EkfFusionAlgNode::cb_getRawOdomMsg, this, _1));

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
  if (this->flag_plot_pose_)
  {
    this->plot_pose_pub_->publish(this->plot_pose_);
    this->flag_plot_pose_ = false;
  }
}

/*  [subscriber callbacks] */
void EkfFusionAlgNode::cb_getInitPoseMsg(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr init_msg)
{
  if (this->ekf_->flag_ekf_initialised_)
  {
    //get yaw information
    tf2::Quaternion q(init_msg->pose.pose.orientation.x, init_msg->pose.pose.orientation.y,
                     init_msg->pose.pose.orientation.z, init_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // set manual state
    Eigen::Matrix<double, 3, 1> state;
    Eigen::Matrix<double, 3, 3> covariance;
    this->ekf_->getStateAndCovariance(state, covariance);
    state(0) = init_msg->pose.pose.position.x;
    state(1) = init_msg->pose.pose.position.y;
    state(2) = yaw;
    this->ekf_->setStateAndCovariance(state, covariance);

    ////////////////////////////////////////////////////////////////////////////////
    ///// generate frame -> child transform
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, state(2));
    geometry_msgs::msg::TransformStamped tf_odom2base;
    try
    {
      tf_odom2base = this->tf_buffer_->lookupTransform(this->child_id_, "base_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(),"[draw_frames] TF exception cb_getInitPoseMsg:", ex.what());
    }

    // generate 4x4 transform matrix odom2base
    Eigen::Affine3d af_odom2base;
    af_odom2base = tf2::transformToEigen(tf_odom2base);
    Eigen::Matrix4d tr_odom2base;
    tr_odom2base.setIdentity();
    tr_odom2base.block<3, 3>(0, 0) = af_odom2base.linear();
    tr_odom2base.block<3, 1>(0, 3) = af_odom2base.translation();

    // generate 4x4 transform matrix map2base
    Eigen::Matrix4d tr_map2base;
    tr_map2base.setIdentity();
    Eigen::Quaterniond rot(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    tr_map2base.block<3, 3>(0, 0) = rot.toRotationMatrix();
    tr_map2base.block<3, 1>(0, 3) = Eigen::Vector3d(state(0), state(1), 0.0);

    // given: odom2base * map2odom = map2base
    // thenn: map2odom = map2base * odom2base^(-1)
    Eigen::Matrix4d tr_map2odom;
    tr_map2odom = tr_map2base * tr_odom2base.inverse();

    Eigen::Quaterniond quat_final(tr_map2odom.block<3, 3>(0, 0));

    this->odom_to_map_.header.frame_id = this->frame_id_;
    this->odom_to_map_.child_frame_id = this->child_id_;
    this->odom_to_map_.header.stamp = this->now();
    this->odom_to_map_.transform.translation.x = tr_map2odom(0, 3);
    this->odom_to_map_.transform.translation.y = tr_map2odom(1, 3);
    this->odom_to_map_.transform.translation.z = tr_map2odom(2, 3);
    this->odom_to_map_.transform.rotation.x = quat_final.x();
    this->odom_to_map_.transform.rotation.y = quat_final.y();
    this->odom_to_map_.transform.rotation.z = quat_final.z();
    this->odom_to_map_.transform.rotation.w = quat_final.w();

    this->broadcaster_->sendTransform(this->odom_to_map_);
    ////////////////////////////////////////////////////////////////////////////////
  }

}

void EkfFusionAlgNode::cb_getGpsOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{

  static int first_exec = true;

  double vx = odom_msg->twist.twist.linear.x;
  double vy = odom_msg->twist.twist.linear.y;
  double speed = sqrt(vx * vx + vy * vy);

  if (speed > this->min_speed_ || (this->is_simulation_ && first_exec))
  {

    ekf::GnssObservation obs;

    //get yaw information
    tf2::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // set observation
    obs.x = odom_msg->pose.pose.position.x;
    obs.y = odom_msg->pose.pose.position.y;
    obs.theta = yaw;
    obs.sigma_x = odom_msg->pose.covariance[0];
    obs.sigma_y = odom_msg->pose.covariance[7];
    obs.sigma_theta = odom_msg->pose.covariance[35];

    // state lecture
    Eigen::Matrix<double, 3, 1> state;
    Eigen::Matrix<double, 3, 3> covariance;
    this->ekf_->getStateAndCovariance(state, covariance);

    // rear direction protection
    float diff = state(2) - obs.theta;
    if (abs(diff) > PI)
    {
      if (state(2) <= 0.0)
        diff = (state(2) + 2 * PI) - obs.theta;
      else if (obs.theta <= 0.0)
        diff = state(2) - (obs.theta + 2 * PI);
    }
    if (abs(diff) > PI / 2)
    {
      obs.theta = state(2);
    }

    this->ekf_->update(obs);

    ////////////////////////////////////////////////////////////////////////////////
    ///// generate frame -> child transform
    this->ekf_->getStateAndCovariance(state, covariance);
    tf2::Quaternion quaternion; 
    quaternion.setRPY(0, 0, state(2));
    geometry_msgs::msg::TransformStamped tf_odom2base;
    try
    {
      tf_odom2base = this->tf_buffer_->lookupTransform(this->child_id_, "base_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(),"[draw_frames] TF exception cb_getInitPoseMsg:", ex.what());
    }

    // generate 4x4 transform matrix odom2base
    Eigen::Affine3d af_odom2base;
    af_odom2base = tf2::transformToEigen(tf_odom2base);
    Eigen::Matrix4d tr_odom2base;
    tr_odom2base.setIdentity();
    tr_odom2base.block<3, 3>(0, 0) = af_odom2base.linear();
    tr_odom2base.block<3, 1>(0, 3) = af_odom2base.translation();

    // generate 4x4 transform matrix map2base
    Eigen::Matrix4d tr_map2base;
    tr_map2base.setIdentity();
    Eigen::Quaterniond rot(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    tr_map2base.block<3, 3>(0, 0) = rot.toRotationMatrix();
    tr_map2base.block<3, 1>(0, 3) = Eigen::Vector3d(state(0), state(1), 0.0);

    // given: odom2base * map2odom = map2base
    // thenn: map2odom = map2base * odom2base^(-1)
    Eigen::Matrix4d tr_map2odom;
    tr_map2odom = tr_map2base * tr_odom2base.inverse();

    Eigen::Quaterniond quat_final(tr_map2odom.block<3, 3>(0, 0));

    this->odom_to_map_.header.frame_id = this->frame_id_;
    this->odom_to_map_.child_frame_id = this->child_id_;
    this->odom_to_map_.header.stamp = this->now();
    this->odom_to_map_.transform.translation.x = tr_map2odom(0, 3);
    this->odom_to_map_.transform.translation.y = tr_map2odom(1, 3);
    this->odom_to_map_.transform.translation.z = tr_map2odom(2, 3);
    this->odom_to_map_.transform.rotation.x = quat_final.x();
    this->odom_to_map_.transform.rotation.y = quat_final.y();
    this->odom_to_map_.transform.rotation.z = quat_final.z();
    this->odom_to_map_.transform.rotation.w = quat_final.w();

    this->broadcaster_->sendTransform(this->odom_to_map_);
    ////////////////////////////////////////////////////////////////////////////////

    this->ekf_->flag_ekf_initialised_ = true;
    first_exec = false;

  }

}

void EkfFusionAlgNode::cb_getRawOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{

  if (this->ekf_->flag_ekf_initialised_)
  {

    this->odom_to_map_.header.stamp = this->now();
    this->broadcaster_->sendTransform(this->odom_to_map_);

    double yaw_frame, x_frame, y_frame;
    double yaw_frame_prev, x_frame_prev, y_frame_prev;
    double roll, pitch, yaw, yaw_use;
    //get yaw information
    tf2::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    yaw_use = yaw;

    //initialization of static variables
    static double x_prev = odom_msg->pose.pose.position.x;
    static double y_prev = odom_msg->pose.pose.position.y;
    static double theta_prev = yaw_use;

    //for differential problems
    float diff = yaw_use - theta_prev;
    if (abs(diff) > PI)
    {
      if (yaw_use <= 0.0)
        yaw_use = yaw_use + 2 * PI;
      else if (theta_prev <= 0.0)
        theta_prev = theta_prev + 2 * PI;
    }

    ///////////////////////////////////////////////////////////
    ///// TRANSFORM TO TF FARME
    geometry_msgs::msg::PointStamped point_frame;
    geometry_msgs::msg::PointStamped point_child;
    point_child.header.frame_id = this->child_id_;
    point_child.header.stamp = this->now();
    point_child.point.x = odom_msg->pose.pose.position.x;
    point_child.point.y = odom_msg->pose.pose.position.y;
    point_child.point.z = 0.0;
    try
    {
      this->tf_buffer_->transform(point_child, point_frame, this->frame_id_);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "[draw_frames] TF exception cb_getInitPoseMsg:", ex.what());
      return;
    }
    x_frame = point_frame.point.x;
    y_frame = point_frame.point.y;

    geometry_msgs::msg::PointStamped point_frame2;
    geometry_msgs::msg::PointStamped point_child2;
    point_child2.header.frame_id = this->child_id_;
    point_child2.header.stamp = this->now();
    point_child2.point.x = x_prev;
    point_child2.point.y = y_prev;
    point_child2.point.z = 0.0;
    try
    {
      this->tf_buffer_->transform(point_child2, point_frame2, this->frame_id_);

    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "[draw_frames] TF exception cb_getInitPoseMsg: %s", ex.what());
      return;
    }
    x_frame_prev = point_frame2.point.x;
    y_frame_prev = point_frame2.point.y;
    ///////////////////////////////////////////////////////////

    ekf::OdomAction act;

    act.delta_x = x_frame - x_frame_prev;
    act.delta_y = y_frame - y_frame_prev;
    act.delta_theta = yaw_use - theta_prev;
    this->ekf_->predict(act);

    //for next step
    x_prev = odom_msg->pose.pose.position.x;
    y_prev = odom_msg->pose.pose.position.y;
    theta_prev = yaw;

    ////////////////////////////////////////////////////////////////////////////////
    ///// update rclcpp message structure for plot
    Eigen::Matrix<double, 3, 1> state;
    Eigen::Matrix<double, 3, 3> covariance;
    this->ekf_->getStateAndCovariance(state, covariance);
    this->plot_pose_.header.frame_id = this->frame_id_;
    tf2::Quaternion quaternion; 
    quaternion.setRPY(0, 0, state(2));
    this->plot_pose_.pose.pose.position.x = state(0);
    this->plot_pose_.pose.pose.position.y = state(1);
    this->plot_pose_.pose.pose.orientation.x = quaternion[0];
    this->plot_pose_.pose.pose.orientation.y = quaternion[1];
    this->plot_pose_.pose.pose.orientation.z = quaternion[2];
    this->plot_pose_.pose.pose.orientation.w = quaternion[3];
    this->plot_pose_.pose.covariance[0] = covariance(0, 0);
    this->plot_pose_.pose.covariance[7] = covariance(1, 1);
    this->plot_pose_.pose.covariance[14] = 1000000000000.0;
    this->plot_pose_.pose.covariance[35] = covariance(2, 2);
    this->flag_plot_pose_ = true;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    ///// generate frame -> child transform
    geometry_msgs::msg::TransformStamped tf_odom2base;
    try
    {
      tf_odom2base = this->tf_buffer_->lookupTransform(this->child_id_, "base_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(),"[draw_frames] TF exception cb_getRawOdomMsg3:\n%s", ex.what());
    }

    // generate 4x4 transform matrix odom2base
    Eigen::Affine3d af_odom2base;
    af_odom2base = tf2::transformToEigen(tf_odom2base);
    Eigen::Matrix4d tr_odom2base;
    tr_odom2base.setIdentity();
    tr_odom2base.block<3, 3>(0, 0) = af_odom2base.linear();
    tr_odom2base.block<3, 1>(0, 3) = af_odom2base.translation();

    // generate 4x4 transform matrix map2base
    Eigen::Matrix4d tr_map2base;
    tr_map2base.setIdentity();
    Eigen::Quaterniond rot(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    tr_map2base.block<3, 3>(0, 0) = rot.toRotationMatrix();
    tr_map2base.block<3, 1>(0, 3) = Eigen::Vector3d(state(0), state(1), 0.0);

    // given: odom2base * map2odom = map2base
    // thenn: map2odom = map2base * odom2base^(-1)
    Eigen::Matrix4d tr_map2odom;
    tr_map2odom = tr_map2base * tr_odom2base.inverse();

    Eigen::Quaterniond quat_final(tr_map2odom.block<3, 3>(0, 0));

    this->odom_to_map_.header.frame_id = this->frame_id_;
    this->odom_to_map_.child_frame_id = this->child_id_;
    this->odom_to_map_.header.stamp = this->now();
    this->odom_to_map_.transform.translation.x = tr_map2odom(0, 3);
    this->odom_to_map_.transform.translation.y = tr_map2odom(1, 3);
    this->odom_to_map_.transform.translation.z = tr_map2odom(2, 3);
    this->odom_to_map_.transform.rotation.x = quat_final.x();
    this->odom_to_map_.transform.rotation.y = quat_final.y();
    this->odom_to_map_.transform.rotation.z = quat_final.z();
    this->odom_to_map_.transform.rotation.w = quat_final.w();

    this->broadcaster_->sendTransform(this->odom_to_map_);
    ////////////////////////////////////////////////////////////////////////////////

  }

}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

rcl_interfaces::msg::SetParametersResult EkfFusionAlgNode::node_config_update(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  this->get_parameter("x_model", this->kalman_config_.x_model);
  this->get_parameter("y_model", this->kalman_config_.y_model);
  this->get_parameter("theta_model", this->kalman_config_.theta_model);
  this->get_parameter("outlier_mahalanobis",
                                     this->kalman_config_.outlier_mahalanobis_threshold);
  return result;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EkfFusionAlgNode>();
  double rate = node->declare_parameter<double>("rate", 10.0);
  rclcpp::Rate loop_rate(rate);  // 10 Hz

  while (rclcpp::ok()) {
    node->mainNodeThread();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
