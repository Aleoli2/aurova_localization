#include "geo_localization_alg_node.h"

GeoLocalizationAlgNode::GeoLocalizationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<GeoLocalizationAlgorithm>()
{

  this->public_node_handle_.getParam("/geo_localization/lat_zero", this->lat_zero_);
  this->public_node_handle_.getParam("/geo_localization/lon_zero", this->lon_zero_);
  this->public_node_handle_.getParam("/geo_localization/frame_id", this->frame_id_);
  this->public_node_handle_.getParam("/geo_localization/url_to_map", this->map_config_.url_to_map);

  //init class attributes if necessary
  if(!this->private_node_handle_.getParam("rate", this->config_.rate))
  {
    ROS_WARN("GeoLocalizationAlgNode::GeoLocalizationAlgNode: param 'rate' not found");
  }
  else
    this->setRate(this->config_.rate);

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

GeoLocalizationAlgNode::~GeoLocalizationAlgNode(void)
{
  // [free dynamic memory]
}

void GeoLocalizationAlgNode::mainNodeThread(void)
{
  //lock access to algorithm if necessary
  this->alg_.lock();
  ROS_DEBUG("GeoLocalizationAlgNode::mainNodeThread");
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  
  this->alg_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void GeoLocalizationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  if(config.rate!=this->getRate())
    this->setRate(config.rate);
  this->config_=config;
  this->alg_.unlock();
}

void GeoLocalizationAlgNode::addNodeDiagnostics(void)
{
}

void GeoLocalizationAlgNode::fromUtmTransform(void)
{

  this->tf_to_utm_.header.frame_id = "utm";
  this->tf_to_utm_.child_frame_id = this->frame_id_;
  this->tf_to_utm_.header.stamp = ros::Time::now();
  this->tf_to_utm_.transform.translation.x = 0.0;
  this->tf_to_utm_.transform.translation.y = 0.0;
  this->tf_to_utm_.transform.translation.z = 0.0;
  this->tf_to_utm_.transform.rotation = tf::createQuaternionMsgFromYaw(3.1415 / 2.0);

  this->broadcaster_.sendTransform(this->tf_to_utm_);

  return;
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<GeoLocalizationAlgNode>(argc, argv, "geo_localization_alg_node");
}
