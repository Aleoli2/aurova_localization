#include "pose_simulation_alg_node.h"

PoseSimulationAlgNode::PoseSimulationAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<PoseSimulationAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]
  this->public_node_handle_.getParam("/pose_simulation/frame_id", this->frame_id_);
  this->public_node_handle_.getParam("/pose_simulation/child_id", this->child_id_);
  this->public_node_handle_.getParam("/pose_simulation/init_x", this->init_x_);
  this->public_node_handle_.getParam("/pose_simulation/init_y", this->init_y_);
  this->public_node_handle_.getParam("/pose_simulation/init_w", this->init_w_);

  // init tf
  this->pose_tf_.header.frame_id = this->frame_id_.c_str();
  this->pose_tf_.child_frame_id = this->child_id_.c_str();
  this->pose_tf_.header.stamp = ros::Time::now();
  this->pose_tf_.transform.translation.x = this->init_x_;
  this->pose_tf_.transform.translation.y = this->init_y_;
  this->pose_tf_.transform.translation.z = 0.0;
  this->pose_tf_.transform.rotation = tf::createQuaternionMsgFromYaw(this->init_w_);
  this->broadcaster_.sendTransform(this->pose_tf_);

  //init msg
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, this->init_w_);
  this->pose_sim_.header.frame_id = this->frame_id_.c_str();
  this->pose_sim_.pose.pose.position.x = this->init_x_;
  this->pose_sim_.pose.pose.position.y = this->init_y_;
  this->pose_sim_.pose.pose.position.z = 0;
  this->pose_sim_.pose.pose.orientation.x = quaternion[0];
  this->pose_sim_.pose.pose.orientation.y = quaternion[1];
  this->pose_sim_.pose.pose.orientation.z = quaternion[2];
  this->pose_sim_.pose.pose.orientation.w = quaternion[3];
  this->pose_sim_.pose.covariance[0] = 0.5; // TODO: calculation of variances !!!
  this->pose_sim_.pose.covariance[7] = 0.5;
  this->pose_sim_.pose.covariance[35] = 0.5;

  // [init publishers]
  this->pose_publisher_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped
      > ("/pose_sim", 1);

  // [init subscribers]
  this->ackermann_subscriber_ = this->public_node_handle_.subscribe("/desired_ackermann_state", 1,
                                                                    &PoseSimulationAlgNode::cb_ackermannState, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

PoseSimulationAlgNode::~PoseSimulationAlgNode(void)
{
  // [free dynamic memory]
}

void PoseSimulationAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->pose_tf_.header.stamp = ros::Time::now();
  this->broadcaster_.sendTransform(this->pose_tf_);
  this->pose_sim_.header.stamp = ros::Time::now();
  this->pose_publisher_.publish(this->pose_sim_);
}

/*  [subscriber callbacks] */
void PoseSimulationAlgNode::cb_ackermannState(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr& ackermann_state_msg)
{
  this->alg_.lock();

  this->ackermann_state_.drive.speed = ackermann_state_msg->drive.speed;
  this->ackermann_state_.drive.steering_angle = ackermann_state_msg->drive.steering_angle;

  // listen transform
  try
  {
    this->listener_.lookupTransform(this->frame_id_, this->child_id_, ros::Time(0), this->pose_current_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  this->alg_.generateNewPoseMsg2D(this->pose_current_, this->ackermann_state_, this->pose_sim_, this->pose_tf_,
                                  this->frame_id_, this->child_id_);

  this->broadcaster_.sendTransform(this->pose_tf_);
  this->pose_publisher_.publish(this->pose_sim_);

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void PoseSimulationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void PoseSimulationAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < PoseSimulationAlgNode > (argc, argv, "pose_simulation_alg_node");
}
