#include "geo_localization_alg_node.h"

GeoLocalizationAlgNode::GeoLocalizationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<GeoLocalizationAlgorithm>()
{

  //// Init class attributes if necessary
  this->public_node_handle_.getParam("/geo_localization/lat_zero", this->lat_zero_);
  this->public_node_handle_.getParam("/geo_localization/lon_zero", this->lon_zero_);
  this->public_node_handle_.getParam("/geo_localization/frame_id", this->frame_id_);
  this->public_node_handle_.getParam("/geo_localization/url_to_map", this->map_config_.url_to_map);
  if(!this->private_node_handle_.getParam("rate", this->config_.rate))
  {
    ROS_WARN("GeoLocalizationAlgNode::GeoLocalizationAlgNode: param 'rate' not found");
  }
  else
    this->setRate(this->config_.rate);

  //// Generate transform between frame_id and utm (lat/long zero requiered).
  this->fromUtmTransform();
  this->mapToOdomInit();
  this->map_config_.utm2map_tr.x = this->tf_to_utm_.transform.translation.x;
  this->map_config_.utm2map_tr.y = this->tf_to_utm_.transform.translation.y;

  //// Read map from file.
  static_data_representation::InterfaceAP interface(this->map_config_);
  interface.readMapFromFile();
  this->map_ = interface.getMap();

  //// Localization init
  this->loc_config_.window_size = 50; // TODO: get from params
  this->optimization_ = new geo_referencing::OptimizationProcess(this->loc_config_);

  //// Plot map in Rviz (TODO: put it on function).
  this->parseMapToRosMarker(this->marker_array_);

  // [init publishers]
  this->marker_pub_ = this->public_node_handle_.advertise < visualization_msgs::MarkerArray > ("/map", 1);
  
  // [init subscribers]
  this->odom_subscriber_ = this->public_node_handle_.subscribe("odom", 1, &GeoLocalizationAlgNode::odom_callback, this);
  
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
  //ROS_DEBUG("GeoLocalizationAlgNode::mainNodeThread");
  //std::cout << "x: " << this->map_.at(0).at(0).x << ", y: " << this->map_.at(0).at(0).y << std::endl;

  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->tf_to_utm_.header.seq = this->tf_to_utm_.header.seq + 1;
  this->tf_to_utm_.header.stamp = ros::Time::now();
  this->broadcaster_.sendTransform(this->tf_to_utm_);

  this->tf_to_map_.header.seq = this->tf_to_map_.header.seq + 1;
  this->tf_to_map_.header.stamp = ros::Time::now();
  this->broadcaster_.sendTransform(this->tf_to_map_);

  this->marker_pub_.publish(this->marker_array_);
  
  this->alg_.unlock();
}

/*  [subscriber callbacks] */
void GeoLocalizationAlgNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("GpsOdomOptimizationAlgNode::odom_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  //this->odom_mutex_enter();

  

  //unlock previously blocked shared variables
  this->alg_.unlock();
  //this->odom_mutex_exit();
}

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
  Ellipsoid utm;
  double utm_x;
  double utm_y;
  char utm_zone[30];
  int ref_ellipsoid = 23;
  utm.LLtoUTM(ref_ellipsoid, this->lat_zero_, this->lon_zero_, utm_y, utm_x, utm_zone);

  this->tf_to_utm_.header.frame_id = "utm";
  this->tf_to_utm_.child_frame_id = this->frame_id_;
  this->tf_to_utm_.header.stamp = ros::Time::now();
  this->tf_to_utm_.transform.translation.x = utm_x;
  this->tf_to_utm_.transform.translation.y = utm_y;
  this->tf_to_utm_.transform.translation.z = 0.0;
  this->tf_to_utm_.transform.rotation = tf::createQuaternionMsgFromYaw(0.0/*3.1415 / 2.0*/);

  this->broadcaster_.sendTransform(this->tf_to_utm_);

  return;
}

void GeoLocalizationAlgNode::mapToOdomInit(void)
{

  this->tf_to_map_.header.frame_id = this->frame_id_;
  this->tf_to_map_.child_frame_id = "odom";
  this->tf_to_map_.header.stamp = ros::Time::now();

  this->tf_to_map_.transform.translation.x = 0.0;
  this->tf_to_map_.transform.translation.y = 0.0;
  this->tf_to_map_.transform.translation.z = 0.0;
  this->tf_to_map_.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

  this->broadcaster_.sendTransform(this->tf_to_map_);

  return;
}

int GeoLocalizationAlgNode::parseMapToRosMarker(visualization_msgs::MarkerArray& marker_array)
{
  visualization_msgs::Marker marker;
  visualization_msgs::Marker marker_line;

  marker.header.frame_id = this->frame_id_;
  marker.header.stamp = ros::Time::now();
  marker_line.header.frame_id = this->frame_id_;
  marker_line.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "nodes";
  marker_line.ns = "links";

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::SPHERE;
  marker_line.type = visualization_msgs::Marker::LINE_LIST;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  marker_line.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.05;
  marker_line.scale.x = 0.4;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker_line.color.r = 0.0f;
  marker_line.color.g = 0.0f;
  marker_line.color.b = 1.0f;
  marker_line.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_line.lifetime = ros::Duration();

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  geometry_msgs::Point point_line;
  int id = 0;
  for (int i = 0; i < this->map_.size(); i++){
    marker_line.points.clear();
    for (int j = 0; j < this->map_.at(i).size(); j++){
      marker.pose.position.x = this->map_.at(i).at(j).x;
      marker.pose.position.y = this->map_.at(i).at(j).y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.id = id;
      id++;
      marker_array.markers.push_back(marker);

      if (j < this->map_.at(i).size() - 1){
        point_line.x = this->map_.at(i).at(j).x;
        point_line.y = this->map_.at(i).at(j).y;
        point_line.z = 0.0;
        marker_line.points.push_back(point_line);
        point_line.x = this->map_.at(i).at(j+1).x;
        point_line.y = this->map_.at(i).at(j+1).y;
        point_line.z = 0.0;
        marker_line.points.push_back(point_line);
        marker_line.id = id;
        id++;
        marker_array.markers.push_back(marker_line);
      }
    }
  }

  return 0;
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<GeoLocalizationAlgNode>(argc, argv, "geo_localization_alg_node");
}
