/**
 * \file ekf_fusion_alg_node.h
 *
 *  Created on: 21 Nov 2018
 *      Author: m.a.munoz
 */

#ifndef _ekf_fusion_alg_node_h_
#define _ekf_fusion_alg_node_h_

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ekf.h"
#include <Eigen/Dense>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief Aurova ROS Specific Algorithm Class
 *
 */
class EkfFusionAlgNode : public rclcpp::Node
{
private:

  ekf::KalmanConfiguration kalman_config_;
  CEkfPtr ekf_;
  bool flag_plot_pose_;
  bool is_simulation_;
  std::string frame_id_;
  std::string child_id_;
  float min_speed_;
  geometry_msgs::msg::PoseWithCovarianceStamped plot_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_;
  geometry_msgs::msg::TransformStamped odom_to_map_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  // [publisher attributes]
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr plot_pose_pub_;

  // [subscriber attributes]
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_raw_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  
  /**
   * \brief callback for read initial pose messages
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getInitPoseMsg(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr init_msg);

  /**
   * \brief callback for read odometry messages
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getGpsOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  /**
   * \brief callback for read odometry messages
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getRawOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  // [service attributes]

  // [client attributes]

  // [action server attributes]

  // [action client attributes]

public:
  /**
   * \brief Constructor
   *
   * This constructor initializes specific class attributes and all ROS
   * communications variables to enable message exchange.
   */
  EkfFusionAlgNode(void);

  /**
   * \brief Destructor
   *
   * This destructor frees all necessary dynamic memory allocated within this
   * this class.
   */
  ~EkfFusionAlgNode(void);

  /**
   * \brief main node thread
   *
   * This is the main thread node function. Code written here will be executed
   * in every node loop while the algorithm is on running state. Loop frequency
   * can be tuned by modifying loop_rate attribute.
   *
   * Here data related to the process loop or to ROS topics (mainly data structs
   * related to the MSG and SRV files) must be updated. ROS publisher objects
   * must publish their data in this process. ROS client servers may also
   * request data to the corresponding server topics.
   */
  void mainNodeThread(void);

protected:

  /**
   * \brief dynamic reconfigure server callback
   *
   * This method is called whenever a new configuration is received through
   * the dynamic reconfigure. The derivated generic algorithm class must
   * implement it.
   *
   */
  rcl_interfaces::msg::SetParametersResult node_config_update(const std::vector<rclcpp::Parameter> & parameters);

};

#endif
