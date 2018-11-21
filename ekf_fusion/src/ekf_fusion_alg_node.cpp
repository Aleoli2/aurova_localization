#include "ekf_fusion_alg_node.h"

EkfFusionAlgNode::EkfFusionAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<EkfFusionAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]
  this->flagSendPose = false;

  // [init publishers]
  this->pose_publisher_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped
      > ("/initialpose", 1);

  // [init subscribers]
  this->amcl_pose_sub_ = this->public_node_handle_.subscribe("/amcl_pose", 1, &EkfFusionAlgNode::cb_getPoseMsg, this);
  this->odom_gps_sub_ = this->public_node_handle_.subscribe("/odometry/gps", 1, &EkfFusionAlgNode::cb_getGpsOdomMsg,
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
  if (this->flagSendPose)
  {
    this->pose_publisher_.publish(this->pose_filtered_);
    this->flagSendPose = false;
  }
}

/*  [subscriber callbacks] */
void EkfFusionAlgNode::cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  this->alg_.lock();
  this->alg_.unlock();
}

void EkfFusionAlgNode::cb_getGpsOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  this->alg_.lock();
  this->pose_filtered_.pose.pose.position = odom_msg->pose.pose.position;
  this->flagSendPose = true;
  this->alg_.unlock();
}

void EkfFusionAlgNode::cb_getRawOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  this->alg_.lock();
  this->pose_filtered_.pose.pose.orientation = odom_msg->pose.pose.orientation;
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
