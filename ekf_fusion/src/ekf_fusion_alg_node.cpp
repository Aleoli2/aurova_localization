#include "ekf_fusion_alg_node.h"

EkfFusionAlgNode::EkfFusionAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<EkfFusionAlgorithm>()
{

  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]
  this->flag_plot_pose_ = false;
  this->public_node_handle_.getParam("/ekf_fusion/frame_id", this->frame_id_);
  this->public_node_handle_.getParam("/ekf_fusion/child_id", this->child_id_);
  this->public_node_handle_.getParam("/ekf_fusion/x_model", this->kalman_config_.x_model);
  this->public_node_handle_.getParam("/ekf_fusion/y_model", this->kalman_config_.y_model);
  this->public_node_handle_.getParam("/ekf_fusion/theta_model", this->kalman_config_.theta_model);
  this->public_node_handle_.getParam("/ekf_fusion/outlier_mahalanobis",
                                     this->kalman_config_.outlier_mahalanobis_threshold);
  this->public_node_handle_.getParam("/ekf_fusion/min_speed", this->min_speed_);

  this->ekf_ = new CEkf(this->kalman_config_);

  // [init publishers]
  this->plot_pose_pub_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped
      > ("/pose_plot", 1);

  // [init subscribers]
  this->init_pose_sub_ = this->public_node_handle_.subscribe("/initialpose", 1, &EkfFusionAlgNode::cb_getInitPoseMsg,
                                                             this);
  this->odom_gps_sub_ = this->public_node_handle_.subscribe("/odometry_gps", 1, &EkfFusionAlgNode::cb_getGpsOdomMsg,
                                                            this);
  this->odom_raw_sub_ = this->public_node_handle_.subscribe("/odom", 1, &EkfFusionAlgNode::cb_getRawOdomMsg, this);

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
    this->plot_pose_pub_.publish(this->plot_pose_);
    this->flag_plot_pose_ = false;
  }
}

/*  [subscriber callbacks] */
void EkfFusionAlgNode::cb_getInitPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_msg)
{
  this->alg_.lock();

  if (this->ekf_->flag_ekf_initialised_)
  {
    //get yaw information
    tf::Quaternion q(init_msg->pose.pose.orientation.x, init_msg->pose.pose.orientation.y,
                     init_msg->pose.pose.orientation.z, init_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
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
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, state(2));
    tf::StampedTransform tf_odom2base;
    try
    {
      this->listener_.lookupTransform(this->child_id_, "base_link", ros::Time(0), tf_odom2base);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("[draw_frames] TF exception cb_getInitPoseMsg:\n%s", ex.what());
    }

    // generate 4x4 transform matrix odom2base
    Eigen::Affine3d af_odom2base;
    tf::transformTFToEigen(tf_odom2base, af_odom2base);
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
    this->odom_to_map_.header.stamp = ros::Time::now();
    this->odom_to_map_.transform.translation.x = tr_map2odom(0, 3);
    this->odom_to_map_.transform.translation.y = tr_map2odom(1, 3);
    this->odom_to_map_.transform.translation.z = tr_map2odom(2, 3);
    this->odom_to_map_.transform.rotation.x = quat_final.x();
    this->odom_to_map_.transform.rotation.y = quat_final.y();
    this->odom_to_map_.transform.rotation.z = quat_final.z();
    this->odom_to_map_.transform.rotation.w = quat_final.w();

    this->broadcaster_.sendTransform(this->odom_to_map_);
    ////////////////////////////////////////////////////////////////////////////////
  }

  this->alg_.unlock();
}

void EkfFusionAlgNode::cb_getGpsOdomMsg(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  this->alg_.lock();

  double vx = odom_msg->twist.twist.linear.x;
  double vy = odom_msg->twist.twist.linear.y;
  double speed = sqrt(vx * vx + vy * vy);

  if (speed > this->min_speed_)
  {

    ekf::GnssObservation obs;

    //get yaw information
    tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
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
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, state(2));
    tf::StampedTransform tf_odom2base;
    try
    {
      this->listener_.lookupTransform(this->child_id_, "base_link", ros::Time(0), tf_odom2base);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("[draw_frames] TF exception cb_getGpsOdomMsg:\n%s", ex.what());
    }

    // generate 4x4 transform matrix odom2base
    Eigen::Affine3d af_odom2base;
    tf::transformTFToEigen(tf_odom2base, af_odom2base);
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
    this->odom_to_map_.header.stamp = ros::Time::now();
    this->odom_to_map_.transform.translation.x = tr_map2odom(0, 3);
    this->odom_to_map_.transform.translation.y = tr_map2odom(1, 3);
    this->odom_to_map_.transform.translation.z = tr_map2odom(2, 3);
    this->odom_to_map_.transform.rotation.x = quat_final.x();
    this->odom_to_map_.transform.rotation.y = quat_final.y();
    this->odom_to_map_.transform.rotation.z = quat_final.z();
    this->odom_to_map_.transform.rotation.w = quat_final.w();

    this->broadcaster_.sendTransform(this->odom_to_map_);
    ////////////////////////////////////////////////////////////////////////////////

    this->ekf_->flag_ekf_initialised_ = true;

  }

  this->alg_.unlock();
}

void EkfFusionAlgNode::cb_getRawOdomMsg(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  this->alg_.lock();

  if (this->ekf_->flag_ekf_initialised_)
  {

    this->odom_to_map_.header.stamp = ros::Time::now();
    this->broadcaster_.sendTransform(this->odom_to_map_);

    double yaw_frame, x_frame, y_frame;
    double yaw_frame_prev, x_frame_prev, y_frame_prev;
    double roll, pitch, yaw, yaw_use;
    //get yaw information
    tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
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
    geometry_msgs::PointStamped point_frame;
    geometry_msgs::PointStamped point_child;
    point_child.header.frame_id = this->child_id_;
    point_child.header.stamp = ros::Time(0); //ros::Time::now();
    point_child.point.x = odom_msg->pose.pose.position.x;
    point_child.point.y = odom_msg->pose.pose.position.y;
    point_child.point.z = 0.0;
    try
    {
      this->listener_.transformPoint(this->frame_id_, point_child, point_frame);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("[draw_frames] TF exception cb_getRawOdomMsg1:\n%s", ex.what());
      return;
    }
    x_frame = point_frame.point.x;
    y_frame = point_frame.point.y;

    geometry_msgs::PointStamped point_frame2;
    geometry_msgs::PointStamped point_child2;
    point_child2.header.frame_id = this->child_id_;
    point_child2.header.stamp = ros::Time(0); //ros::Time::now();
    point_child2.point.x = x_prev;
    point_child2.point.y = y_prev;
    point_child2.point.z = 0.0;
    try
    {
      this->listener_.transformPoint(this->frame_id_, point_child2, point_frame2);

    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("[draw_frames] TF exception cb_getRawOdomMsg2:\n%s", ex.what());
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
    ///// update ros message structure for plot
    Eigen::Matrix<double, 3, 1> state;
    Eigen::Matrix<double, 3, 3> covariance;
    this->ekf_->getStateAndCovariance(state, covariance);
    this->plot_pose_.header.frame_id = this->frame_id_;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, state(2));
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
    tf::StampedTransform tf_odom2base;
    try
    {
      this->listener_.lookupTransform(this->child_id_, "base_link", ros::Time(0), tf_odom2base);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("[draw_frames] TF exception cb_getRawOdomMsg3:\n%s", ex.what());
    }

    // generate 4x4 transform matrix odom2base
    Eigen::Affine3d af_odom2base;
    tf::transformTFToEigen(tf_odom2base, af_odom2base);
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
    this->odom_to_map_.header.stamp = ros::Time::now();
    this->odom_to_map_.transform.translation.x = tr_map2odom(0, 3);
    this->odom_to_map_.transform.translation.y = tr_map2odom(1, 3);
    this->odom_to_map_.transform.translation.z = tr_map2odom(2, 3);
    this->odom_to_map_.transform.rotation.x = quat_final.x();
    this->odom_to_map_.transform.rotation.y = quat_final.y();
    this->odom_to_map_.transform.rotation.z = quat_final.z();
    this->odom_to_map_.transform.rotation.w = quat_final.w();

    this->broadcaster_.sendTransform(this->odom_to_map_);
    ////////////////////////////////////////////////////////////////////////////////

  }

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
