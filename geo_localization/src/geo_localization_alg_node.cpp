#include "geo_localization_alg_node.h"

GeoLocalizationAlgNode::GeoLocalizationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<GeoLocalizationAlgorithm>()
{

  //// Init class attributes if necessary
  this->public_node_handle_.getParam("/geo_localization/lat_zero", this->lat_zero_);
  this->public_node_handle_.getParam("/geo_localization/lon_zero", this->lon_zero_);
  this->public_node_handle_.getParam("/geo_localization/frame_id", this->frame_id_);
  this->public_node_handle_.getParam("/geo_localization/url_to_map", this->map_config_.url_to_map);
  this->public_node_handle_.getParam("/geo_localization/sample_distance", this->map_config_.sample_distance);
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
  // Sample distance // TODO: get from params
  this->interface_ = new static_data_representation::InterfaceAP(this->map_config_);
  this->interface_->readMapFromFile();
  this->interface_->samplePolylineMap();
  this->map_ = this->interface_->getMap();

  //// Data association init
  this->associations_ = new static_data_association::AssociationProblem();

  //// Localization init
  this->loc_config_.window_size = 500; // TODO: get from params
  this->optimization_ = new geo_referencing::OptimizationProcess(this->loc_config_);
  this->optimization_->initializeState();

  //// Plot map in Rviz.
  this->parseMapToRosMarker(this->marker_array_);

  // [init publishers]
  this->localization_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("/localization", 1);
  this->marker_pub_ = this->public_node_handle_.advertise < visualization_msgs::MarkerArray > ("/map", 1);
  this->landmarks_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("/landmarks", 1);
  this->detection_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("/detections", 1);
  this->corregist_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("/corregistered", 1);
  
  // [init subscribers]
  this->odom_subscriber_ = this->public_node_handle_.subscribe("/odom", 1, &GeoLocalizationAlgNode::odom_callback, this);
  this->gnss_subscriber_ = this->public_node_handle_.subscribe("/odometry_gps", 1, &GeoLocalizationAlgNode::gnss_callback, this);
  this->detc_subscriber_ = this->public_node_handle_.subscribe("/ground_lines_pc", 1, &GeoLocalizationAlgNode::detc_callback, this);
  
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

  //this->tf_to_map_.header.seq = this->tf_to_map_.header.seq + 1;
  //this->tf_to_map_.header.stamp = ros::Time::now();
  //this->broadcaster_.sendTransform(this->tf_to_map_);

  //// Publish map only one time.
  static int count = 0;
  if (count == 30){
    this->marker_pub_.publish(this->marker_array_);
  }
  count = count + 1;
  
  this->alg_.unlock();
}

/*  [subscriber callbacks] */
void GeoLocalizationAlgNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("GeoLocalizationAlgNode::odom_callback: New Message Received");
  this->alg_.lock();

  double ini, end;
  ini = ros::Time::now().toSec();

  static bool exec = false;
  static nav_msgs::Odometry msg_prev;
  static int count = 0;

  double tf_yaw;
  double tf_dist;

  if (exec){ // To avoid firs execution.

    //////////////////////////////////////////////////////////////////////////////////////////////
    //// 1) ODOM: Propagate state by using odometry differential. 
    Eigen::Matrix<double, 3, 1> p_a(msg_prev.pose.pose.position.x, msg_prev.pose.pose.position.y, 0.0);
    Eigen::Quaternion<double> q_a(msg_prev.pose.pose.orientation.w, 0.0, 0.0, msg_prev.pose.pose.orientation.z);
    Eigen::Matrix<double, 3, 1> p_b(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);
    Eigen::Quaternion<double> q_b(msg->pose.pose.orientation.w, 0.0, 0.0, msg->pose.pose.orientation.z);
    int id = msg->header.seq;
    
    this->optimization_->propagateState (p_a, q_a, p_b, q_b, id);

    //// 2) ODOM: Generate odometry constraint
    geo_referencing::OdometryConstraint constraint_odom;
    constraint_odom.id_begin = msg_prev.header.seq;
    constraint_odom.id_end = id;
    constraint_odom.tf_q = q_a.conjugate() * q_b;
    constraint_odom.tf_p = q_a.conjugate() * (p_b - p_a);
    constraint_odom.covariance = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).covariance;
    constraint_odom.information = constraint_odom.covariance.inverse();

    this->optimization_->addOdometryConstraint (constraint_odom);

    //////////////////////////////////////////////////////////////////////////////////////////////
    //// 1) DA: Generate Landmarks in interface from map.
    static_data_representation::Pose2D position;
    position.x = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).p.x();
    position.y = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).p.y();
    float radious = 15.0; // TODO: Get from parameter. 
    this->interface_->createLandmarksFromMap(position, radious*2.0);

    // Transform landmarks to base(lidar) frame.
    static_data_representation::Tf tf_lidar2map;
    Eigen::Matrix<double, 3, 3> r = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).q.toRotationMatrix();
    Eigen::Matrix<double, 3, 1> t = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).p;
    tf_lidar2map.linear() = r;
    tf_lidar2map.translation() = t;
    this->interface_->applyTfFromLandmarksToBaseFrame(tf_lidar2map);

    // Parse interface landmarks to DA and plot it.
    pcl::PointCloud<pcl::PointXYZ>::Ptr landmarks_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    landmarks_pcl->header.frame_id = "os_sensor";
    this->associations_->clearLandmarks();
    for (int i = 0; i < this->interface_->getLandmarks().at(0).size(); i++){
      landmarks_pcl->push_back(pcl::PointXYZ(this->interface_->getLandmarks().at(0).at(i).x,
                                            this->interface_->getLandmarks().at(0).at(i).y, 0.0));
      static_data_association::DalmrPolyDetection landmark(Eigen::Vector3d(this->interface_->getLandmarks().at(0).at(i).x, 
                                                                           this->interface_->getLandmarks().at(0).at(i).y, 
                                                                           this->interface_->getLandmarks().at(0).at(i).z),
					                                                                 this->interface_->getLandmarks().at(0).at(i).id);
      this->associations_->addLandmark(std::make_unique<static_data_association::DalmrPolyDetection>(landmark));
    }
    this->landmarks_publisher_.publish(*landmarks_pcl);
    // DEBUG
    std::cout << "LANDMARKS N: " << this->associations_->getLandmarks().size() << std::endl;

    //// 2) DA: Generate detections in interface from msg.
    static_data_representation::PolylineMap detections;
    static_data_representation::Polyline positions_way;
    for (int i = 0; i < this->last_detect_pcl_.points.size(); i++){
			float point_sf_x = this->last_detect_pcl_.points.at(i).x;
			float point_sf_y = this->last_detect_pcl_.points.at(i).y;
			float distance = sqrt(pow(point_sf_x, 2) + pow(point_sf_y, 2));
			if (distance < radious){
        static_data_representation::PolylinePoint pt;
        pt.x = point_sf_x;
        pt.y = point_sf_y;
        pt.z = 0.0;
        pt.id = i;
        positions_way.push_back(pt);
      }
    }
    detections.push_back(positions_way);
    this->interface_->setDetections(detections);

    // Parse interface detections to DA and plot it.
    pcl::PointCloud<pcl::PointXYZ>::Ptr detections_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    detections_pcl->header.frame_id = "os_sensor";
    this->associations_->clearDetections();
    for (int i = 0; i < this->interface_->getDetections().at(0).size(); i++){
      detections_pcl->push_back(pcl::PointXYZ(this->interface_->getDetections().at(0).at(i).x,
                                              this->interface_->getDetections().at(0).at(i).y, 0.0));
      static_data_association::DalmrPolyDetection detection(Eigen::Vector3d(this->interface_->getDetections().at(0).at(i).x, 
                                                                            this->interface_->getDetections().at(0).at(i).y, 
                                                                            this->interface_->getDetections().at(0).at(i).z),
					                                                                  this->interface_->getDetections().at(0).at(i).id);
      this->associations_->addDetection(std::make_unique<static_data_association::DalmrPolyDetection>(detection));
    }
    this->detection_publisher_.publish(*detections_pcl);
    // DEBUG
    std::cout << "DETECTIONS M: " << this->associations_->getDetections().size() << std::endl;

    //// 3) DA: Compute data association
    //// ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(detections_pcl);
    icp.setInputTarget(landmarks_pcl);
  
    pcl::PointCloud<pcl::PointXYZ> coregistered_pcl;
    icp.align(coregistered_pcl);

    Eigen::Matrix4d tf = icp.getFinalTransformation().cast<double>();

    //std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    //std::cout << tf << std::endl;

    coregistered_pcl.header.frame_id = "os_sensor";
    this->corregist_publisher_.publish(coregistered_pcl);

    Eigen::Quaterniond tf_q(tf.block<3, 3>(0, 0));
    Eigen::Vector3d tf_p = tf.block<3, 1>(0, 3);
    Eigen::Vector3d tf_a = tf_q.toRotationMatrix().eulerAngles(0, 1, 2);
    tf_yaw = tf_a.z();
    tf_dist = sqrt(pow(tf_p(0), 2) + pow(tf_p(1), 2));
    std::cout << "YAW: " << tf_yaw << std::endl;
    std::cout << "DISTANCE: " << tf_dist << std::endl;

    //// 4) DA: Generate associations TF constraint
    if (count > 20 && this->associations_->getLandmarks().size() > 50){ // TODO: Get from param
      geo_referencing::AssoConstraint constraints_asso;
      constraints_asso.id = id;
      constraints_asso.p = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).p + 
                           this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).q * tf_p;
      constraints_asso.q = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).q * tf_q;
      constraints_asso.covariance = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).covariance;
      constraints_asso.information = constraints_asso.covariance.inverse();
      this->optimization_->addAssoConstraint (constraints_asso);
      count = 21;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    //// *) Compute optimization problem
    if (this->optimization_->getPriorConstraints().size() > 4){
      float x_min = this->optimization_->getPriorConstraints().at(this->optimization_->getPriorConstraints().size() - 5).p.x();
      float y_min = this->optimization_->getPriorConstraints().at(this->optimization_->getPriorConstraints().size() - 5).p.y();
      float x_max = this->optimization_->getPriorConstraints().at(this->optimization_->getPriorConstraints().size() - 1).p.x();
      float y_max = this->optimization_->getPriorConstraints().at(this->optimization_->getPriorConstraints().size() - 1).p.y();
      
      //std::cout << "DISTANCE PRIOR: " << sqrt(pow(x_max-x_min,2) + pow(y_max-y_min,2)) << std::endl;
      if (sqrt(pow(x_max-x_min,2) + pow(y_max-y_min,2)) > 5.0){
        this->computeOptimizationProblem();
        count++;
      } 
    }
    




    ////////////////////////////////////////////////////////////////////////////////
    //// REPRESENTATION (output)
    // POSE
    int size = this->optimization_->getTrajectoryEstimated().size();

    this->localization_msg_.header.seq = id;
    this->localization_msg_.header.stamp = ros::Time::now();
    this->localization_msg_.header.frame_id = "map";
    this->localization_msg_.child_frame_id = "";
    this->localization_msg_.pose.pose.position.x = this->optimization_->getTrajectoryEstimated().at(size-1).p.x();
    this->localization_msg_.pose.pose.position.y = this->optimization_->getTrajectoryEstimated().at(size-1).p.y();
    this->localization_msg_.pose.pose.position.z = this->optimization_->getTrajectoryEstimated().at(size-1).p.z();

    this->localization_msg_.pose.pose.orientation.x = this->optimization_->getTrajectoryEstimated().at(size-1).q.x();
    this->localization_msg_.pose.pose.orientation.y = this->optimization_->getTrajectoryEstimated().at(size-1).q.y();
    this->localization_msg_.pose.pose.orientation.z = this->optimization_->getTrajectoryEstimated().at(size-1).q.z();
    this->localization_msg_.pose.pose.orientation.w = this->optimization_->getTrajectoryEstimated().at(size-1).q.w();

    this->localization_msg_.pose.covariance[0] = (float)tf_dist;
    this->localization_msg_.pose.covariance[7] = (float)tf_dist;
    this->localization_msg_.pose.covariance[14] = 0.1;
    this->localization_msg_.pose.covariance[35] = (float)tf_yaw;

    this->localization_publisher_.publish(this->localization_msg_);
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    ///// generate MAP -> ODOM transform
    // generate 4x4 transform matrix odom2base
    Eigen::Matrix4d tr_odom2base;
    tr_odom2base.setIdentity();
    Eigen::Quaterniond rot1(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);
    tr_odom2base.block<3, 3>(0, 0) = rot1.toRotationMatrix();
    tr_odom2base.block<3, 1>(0, 3) = Eigen::Vector3d(msg->pose.pose.position.x, 
                                                     msg->pose.pose.position.y, 0.0);

    // generate 4x4 transform matrix map2base
    Eigen::Matrix4d tr_map2base;
    tr_map2base.setIdentity();
    Eigen::Quaterniond rot(this->optimization_->getTrajectoryEstimated().at(size-1).q.w(),
                           this->optimization_->getTrajectoryEstimated().at(size-1).q.x(),
                           this->optimization_->getTrajectoryEstimated().at(size-1).q.y(),
                           this->optimization_->getTrajectoryEstimated().at(size-1).q.z());
    tr_map2base.block<3, 3>(0, 0) = rot.toRotationMatrix();
    tr_map2base.block<3, 1>(0, 3) = Eigen::Vector3d(this->optimization_->getTrajectoryEstimated().at(size-1).p.x(), 
                                                    this->optimization_->getTrajectoryEstimated().at(size-1).p.y(), 0.0);

    // given: odom2base * map2odom = map2base
    // thenn: map2odom = map2base * odom2base^(-1)
    Eigen::Matrix4d tr_map2odom;
    tr_map2odom = tr_map2base * tr_odom2base.inverse();

    Eigen::Quaterniond quat_final(tr_map2odom.block<3, 3>(0, 0));

    this->tf_to_map_.header.frame_id = "map";
    this->tf_to_map_.child_frame_id = "odom";
    this->tf_to_map_.header.seq = this->tf_to_map_.header.seq + 1;
    this->tf_to_map_.header.stamp = ros::Time::now();

    this->tf_to_map_.transform.translation.x = tr_map2odom(0, 3);
    this->tf_to_map_.transform.translation.y = tr_map2odom(1, 3);
    this->tf_to_map_.transform.translation.z = tr_map2odom(2, 3);
    this->tf_to_map_.transform.rotation.x = quat_final.x();
    this->tf_to_map_.transform.rotation.y = quat_final.y();
    this->tf_to_map_.transform.rotation.z = quat_final.z();
    this->tf_to_map_.transform.rotation.w = quat_final.w();

    this->broadcaster_.sendTransform(this->tf_to_map_);
    ////////////////////////////////////////////////////////////////////////////////

  }else{
    exec = true;
  }

  //// Save previous odometry msg.
  msg_prev.pose.pose.position.x = msg->pose.pose.position.x;
  msg_prev.pose.pose.position.y = msg->pose.pose.position.y;
  msg_prev.pose.pose.position.z = msg->pose.pose.position.z;
  msg_prev.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  msg_prev.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  msg_prev.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  msg_prev.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  msg_prev.header.seq = msg->header.seq;

  // loop time
  end = ros::Time::now().toSec();
  std::cout << "TIME LOOP: " << end - ini << std::endl;

  this->alg_.unlock();
}

void GeoLocalizationAlgNode::gnss_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("GeoLocalizationAlgNode::gnss_callback: New Message Received");
  this->alg_.lock();

  Eigen::Matrix<double, 3, 1> p(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);

  geo_referencing::PriorConstraint constraint_prior;
  constraint_prior.id = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).id;
  constraint_prior.p = p;
  constraint_prior.covariance = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).covariance;
  constraint_prior.information = constraint_prior.covariance.inverse();

  this->optimization_->addPriorConstraint (constraint_prior);

  this->alg_.unlock();
}

void GeoLocalizationAlgNode::detc_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  //ROS_INFO("GeoLocalizationAlgNode::detc_callback: New Message Received");
  this->alg_.lock();

  pcl::PCLPointCloud2 detect_pcl2;
  //static pcl::PointCloud<pcl::PointXYZ> detect_pcl;
  this->last_detect_pcl_.clear();

  pcl_conversions::toPCL(*msg, detect_pcl2);
  pcl::fromPCLPointCloud2(detect_pcl2, this->last_detect_pcl_);

  this->alg_.unlock();
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
        //marker_line.points.push_back(point_line);
        point_line.x = this->map_.at(i).at(j+1).x;
        point_line.y = this->map_.at(i).at(j+1).y;
        point_line.z = 0.0;
        //marker_line.points.push_back(point_line);
        marker_line.id = id;
        id++;
        //marker_array.markers.push_back(marker_line);
      }
    }
  }

  return 0;
}

void GeoLocalizationAlgNode::computeOptimizationProblem (void)
{
	//////////////////////////////////////////////////////////////////////
	//// RESIDUALS GENERATION
	size_t index = this->optimization_->getTrajectoryEstimated().size() - 1;
	ceres::Problem problem;
	ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;
	ceres::LossFunction* loss_function = new ceres::HuberLoss(0.01); //nullptr;//new ceres::HuberLoss(0.01);
	if (index > 0) this->optimization_->generateOdomResiduals(loss_function, quaternion_local_parameterization, &problem);
  if (index > 0) this->optimization_->generatePriorResiduals(loss_function, quaternion_local_parameterization, &problem);
  if (index > 0) this->optimization_->generateAssoResiduals(loss_function, quaternion_local_parameterization, &problem);

	//////////////////////////////////////////////////////////////////////
	//// SOLVE OPTIMIZATION PROBLEM
	this->optimization_->solveOptimizationProblem(&problem);

  return;
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<GeoLocalizationAlgNode>(argc, argv, "geo_localization_alg_node");
}
