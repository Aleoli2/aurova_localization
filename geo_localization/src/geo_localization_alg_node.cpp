#include "geo_localization_alg_node.h"

GeoLocalizationAlgNode::GeoLocalizationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<GeoLocalizationAlgorithm>()
{

  //// Init class attributes if necessary
  this->public_node_handle_.getParam("/geo_localization/url_to_map", this->data_config_.url_to_map);
  this->public_node_handle_.getParam("/geo_localization/sample_distance", this->data_config_.sample_distance);
  this->public_node_handle_.getParam("/geo_localization/threshold_asso", this->data_config_.threshold_asso);
  this->public_node_handle_.getParam("/geo_localization/voxel_asso", this->data_config_.voxel_asso);
  this->public_node_handle_.getParam("/geo_localization/radious_dt", this->data_config_.radious_dt);
  this->public_node_handle_.getParam("/geo_localization/radious_lm", this->data_config_.radious_lm);
  this->public_node_handle_.getParam("/geo_localization/acum_tf_da", this->data_config_.acum_tf_da);
  this->public_node_handle_.getParam("/geo_localization/acum_tf_varfactor", this->data_config_.acum_tf_varfactor);
  this->public_node_handle_.getParam("/geo_localization/z_weight", this->data_config_.z_weight);
  this->public_node_handle_.getParam("/geo_localization/type", this->data_config_.type);
  this->public_node_handle_.getParam("/geo_localization/lambda", this->data_config_.lambda);
  this->public_node_handle_.getParam("/geo_localization/k", this->data_config_.k);
  this->public_node_handle_.getParam("/geo_localization/m", this->data_config_.m);
  this->public_node_handle_.getParam("/geo_localization/odom_preweight", this->data_config_.odom_preweight);
  this->public_node_handle_.getParam("/geo_localization/asso_preweight", this->asso_preweight_);

  this->public_node_handle_.getParam("/geo_localization/window_size", this->optimization_config_.window_size);
  this->public_node_handle_.getParam("/geo_localization/max_num_iterations_op", this->optimization_config_.max_num_iterations_op);

  this->public_node_handle_.getParam("/geo_localization/lat_zero", this->lat_zero_);
  this->public_node_handle_.getParam("/geo_localization/lon_zero", this->lon_zero_);
  this->public_node_handle_.getParam("/geo_localization/offset_map_x", this->offset_map_x_);
  this->public_node_handle_.getParam("/geo_localization/offset_map_y", this->offset_map_y_);
  this->public_node_handle_.getParam("/geo_localization/margin_asso_constraints", this->margin_asso_constraints_);
  this->public_node_handle_.getParam("/geo_localization/margin_gnss_constraints", this->margin_gnss_constraints_);
  this->public_node_handle_.getParam("/geo_localization/margin_gnss_distance", this->margin_gnss_distance_);
  this->public_node_handle_.getParam("/geo_localization/map_id", this->map_id_);
  this->public_node_handle_.getParam("/geo_localization/odom_id", this->odom_id_);
  this->public_node_handle_.getParam("/geo_localization/base_id", this->base_id_);
  this->public_node_handle_.getParam("/geo_localization/world_id", this->world_id_);
  this->public_node_handle_.getParam("/geo_localization/lidar_id", this->lidar_id_);

  this->public_node_handle_.getParam("/geo_localization/out_data", this->out_data_);
  this->public_node_handle_.getParam("/geo_localization/out_map", this->out_map_);
  this->public_node_handle_.getParam("/geo_localization/save_data", this->save_data_);
  this->public_node_handle_.getParam("/geo_localization/save_map", this->save_map_);

  this->public_node_handle_.getParam("/geo_localization/ground_truth", this->ground_truth_);
  this->public_node_handle_.getParam("/geo_localization/out_gt", this->out_gt_);
  this->public_node_handle_.getParam("/geo_localization/gt_last_frame", this->gt_last_frame_);
  this->public_node_handle_.getParam("/geo_localization/gt_key_frames", this->gt_key_frames_);

  if(!this->private_node_handle_.getParam("rate", this->config_.rate))
  {
    ROS_WARN("GeoLocalizationAlgNode::GeoLocalizationAlgNode: param 'rate' not found");
  }
  else
    this->setRate(this->config_.rate);
  
  this->count_ = 0;
  this->flag_gps_corr_ = false;
  this->asso_weight_ = 1.0;
  this->odom_weight_ = 100.0;

  //// Generate transform between map and utm (lat/long zero requiered).
  this->fromUtmTransform();
  this->mapToOdomInit();
  this->data_config_.utm2map_tr.x = this->tf_to_utm_.transform.translation.x;
  this->data_config_.utm2map_tr.y = this->tf_to_utm_.transform.translation.y;

  //// Read map from file.
  this->data_ = new data_processing::DataProcessing(this->data_config_);
  this->data_->readMapFromFile();
  this->data_->samplePolylineMap();
  this->data_->addRotationDaEvolution(1.57); // Initialize data association evolution variance
  this->data_->addTranslationDaEvolution(10.0);
  this->map_ = this->data_->getMap();

  //// Localization init
  this->optimization_ = new optimization_process::OptimizationProcess(this->optimization_config_);
  this->optimization_->initializeState();

  //// Plot map in Rviz.
  this->parseMapToRosMarker(this->marker_array_);

  // [init publishers]
  this->localization_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("/localization", 1);
  this->marker_pub_ = this->public_node_handle_.advertise < visualization_msgs::MarkerArray > ("/map", 1);
  this->landmarks_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("/landmarks", 1);
  this->detection_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("/detections", 1);
  this->corregist_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("/corregistered", 1);
  this->gpscorrected_publisher_ = this->public_node_handle_.advertise <nav_msgs::Odometry> ("/odometry_gps_corrected", 1);
  this->wa_publisher_ = this->public_node_handle_.advertise <std_msgs::Float64> ("/wa", 1);
  
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
    optimization_process::OdometryConstraint constraint_odom;
    float N = this->data_config_.odom_preweight + (float)this->data_->getAssociatedLmPcl()->points.size();
    this->odom_weight_ = (N + 1) * (2 - this->data_->dataInformation());
    constraint_odom.id_begin = msg_prev.header.seq;
    constraint_odom.id_end = id;
    constraint_odom.odom_weight = this->odom_weight_;
    constraint_odom.tf_q = q_a.conjugate() * q_b;
    constraint_odom.tf_p = q_a.conjugate() * (p_b - p_a);
    constraint_odom.covariance = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).covariance;
    constraint_odom.information = constraint_odom.covariance.inverse();

    this->optimization_->addOdometryConstraint (constraint_odom);

    //////////////////////////////////////////////////////////////////////////////////////////////
    //// *) Compute optimization problem
    if (this->optimization_->getPriorConstraints().size() > this->margin_gnss_constraints_){
      float x_min = this->optimization_->getPriorConstraints().at(this->optimization_->getPriorConstraints().size() - (this->margin_gnss_constraints_ + 1)).p.x();
      float y_min = this->optimization_->getPriorConstraints().at(this->optimization_->getPriorConstraints().size() - (this->margin_gnss_constraints_ + 1)).p.y();
      float x_max = this->optimization_->getPriorConstraints().at(this->optimization_->getPriorConstraints().size() - 1).p.x();
      float y_max = this->optimization_->getPriorConstraints().at(this->optimization_->getPriorConstraints().size() - 1).p.y();
      
      if (sqrt(pow(x_max-x_min,2) + pow(y_max-y_min,2)) > this->margin_gnss_distance_){
        this->computeOptimizationProblem();
        this->count_ = this->count_ + 1;
      } 
    }

    if (this->ground_truth_){
      this->optimization_->addPose3dToTrajectoryEstimatedGT (this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1));
      constraint_odom.odom_weight = 1.0;
      this->optimization_->addOdometryConstraintGT (constraint_odom);

      if(((int)msg->header.seq == (int)this->gt_key_frames_.at(0)) || 
         ((int)msg->header.seq == (int)this->gt_key_frames_.at(1)) ||
         ((int)msg->header.seq == (int)this->gt_key_frames_.at(2)) ||
         ((int)msg->header.seq == (int)this->gt_key_frames_.at(3)) ||
         ((int)msg->header.seq == (int)this->gt_key_frames_.at(4)) ||
         ((int)msg->header.seq == (int)this->gt_key_frames_.at(5)) ||
         ((int)msg->header.seq == (int)this->gt_key_frames_.at(6)) ||
         ((int)msg->header.seq == (int)this->gt_key_frames_.at(7)) ||
         ((int)msg->header.seq == (int)this->gt_key_frames_.at(8)) ||
         ((int)msg->header.seq == (int)this->gt_key_frames_.at(9)))
      {
        optimization_process::PriorConstraint constraint_prior;
        constraint_prior.id = msg->header.seq;
        constraint_prior.p = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).p;
        constraint_prior.covariance = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).covariance.block<3, 3>(0, 0);
        constraint_prior.information = constraint_prior.covariance.inverse();

        this->optimization_->addPriorConstraintGT (constraint_prior);
      }
      
      if (msg->header.seq == (this->gt_last_frame_ - 200)) this->computeOptimizationProblemGT();
    }

    ////////////////////////////////////////////////////////////////////////////////
    //// REPRESENTATION (output)
    // POSE
    int size = this->optimization_->getTrajectoryEstimated().size();

    this->localization_msg_.header.seq = id;
    this->localization_msg_.header.stamp = ros::Time::now();
    this->localization_msg_.header.frame_id = this->map_id_;
    this->localization_msg_.child_frame_id = "";
    this->localization_msg_.pose.pose.position.x = this->optimization_->getTrajectoryEstimated().at(size-1).p.x();
    this->localization_msg_.pose.pose.position.y = this->optimization_->getTrajectoryEstimated().at(size-1).p.y();
    this->localization_msg_.pose.pose.position.z = this->optimization_->getTrajectoryEstimated().at(size-1).p.z();

    this->localization_msg_.pose.pose.orientation.x = this->optimization_->getTrajectoryEstimated().at(size-1).q.x();
    this->localization_msg_.pose.pose.orientation.y = this->optimization_->getTrajectoryEstimated().at(size-1).q.y();
    this->localization_msg_.pose.pose.orientation.z = this->optimization_->getTrajectoryEstimated().at(size-1).q.z();
    this->localization_msg_.pose.pose.orientation.w = this->optimization_->getTrajectoryEstimated().at(size-1).q.w();

    this->localization_msg_.pose.covariance[0] = this->data_->getTranslationVarianceDaEvolution();
    this->localization_msg_.pose.covariance[7] = this->data_->getTranslationVarianceDaEvolution();
    this->localization_msg_.pose.covariance[14] = 0.1;
    this->localization_msg_.pose.covariance[35] = this->data_->getRotationVarianceDaEvolution();

    this->localization_publisher_.publish(this->localization_msg_);

    // yaw (z-axis rotation)
    float yaw;
    float x = this->optimization_->getTrajectoryEstimated().at(size-1).q.x();
    float y = this->optimization_->getTrajectoryEstimated().at(size-1).q.y();
    float z = this->optimization_->getTrajectoryEstimated().at(size-1).q.z();
    float w = this->optimization_->getTrajectoryEstimated().at(size-1).q.w();
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    ///// generate MAP -> ODOM transform
    // generate 4x4 transform matrix odom2base
    tf::StampedTransform tf_odom2base;
    try
    {
      this->listener_.lookupTransform(this->odom_id_, this->base_id_, ros::Time(0), tf_odom2base);
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
    /*
    Eigen::Matrix4d tr_odom2base;
    tr_odom2base.setIdentity();
    Eigen::Quaterniond rot1(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);
    tr_odom2base.block<3, 3>(0, 0) = rot1.toRotationMatrix();
    tr_odom2base.block<3, 1>(0, 3) = Eigen::Vector3d(msg->pose.pose.position.x, 
                                                     msg->pose.pose.position.y, 0.0);
    */

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

    this->tf_to_map_.header.frame_id = this->map_id_;
    this->tf_to_map_.child_frame_id = this->odom_id_;
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

    ////////////////////////////////////////////////////////////////////////////////
    ///// SAVE DATA
    std::cout << "SEQ: " << msg->header.seq << std::endl;
    if (this->save_data_){
      std::ostringstream out_path_pos2d;
      std::ostringstream out_pos2d;
      std::ofstream file_pos2d;

      out_path_pos2d << this->out_data_ << msg->header.seq << "_pose2d.csv";

      file_pos2d.open(out_path_pos2d.str().c_str(), std::ofstream::trunc);
      out_pos2d << msg->header.seq << ", " <<
                   this->optimization_->getTrajectoryEstimated().at(size-1).p.x() << ", " << 
                   this->optimization_->getTrajectoryEstimated().at(size-1).p.y() << ", " << 
                   yaw << ", " <<
                   this->optimization_->getPriorError().x() << ", " <<
                   this->optimization_->getPriorError().y() << ", " <<
                   this->data_->dataInformation() << "\n";
      file_pos2d << out_pos2d.str();
      file_pos2d.close();
    }
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
  //std::cout << "ODOMETRY TIME LOOP: " << end - ini << std::endl;
  //std::cout << "ODOMETRY SEQUENCE: " << msg->header.seq << std::endl;

  this->alg_.unlock();
}

void GeoLocalizationAlgNode::gnss_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("GeoLocalizationAlgNode::gnss_callback: New Message Received");
  this->alg_.lock();
  
  //// Prevent wrong corrections
  double x_bridge, y_bridge;
  if (this->flag_gps_corr_){
    x_bridge = msg->pose.pose.position.x - this->optimization_->getPriorError().x();
    y_bridge = msg->pose.pose.position.y - this->optimization_->getPriorError().y();
  }else{
    x_bridge = msg->pose.pose.position.x;
    y_bridge = msg->pose.pose.position.y;
  }
  Eigen::Matrix<double, 3, 1> p(x_bridge, y_bridge, 0.0);
  Eigen::Matrix<double, 3, 1> p_raw(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);

  //////////////////////////////////////////////////////////////////////////////////////////////
  //// 1) PRIOR: Generate position constraint.
  optimization_process::PriorConstraint constraint_prior;
  constraint_prior.id = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).id;
  constraint_prior.p = p;
  constraint_prior.p_raw = p_raw;
  constraint_prior.covariance = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).covariance.block<3, 3>(0, 0);
  constraint_prior.information = constraint_prior.covariance.inverse();

  this->optimization_->addPriorConstraint (constraint_prior);

  //// 2) PRIOR: Publish corrected GPS.
  nav_msgs::Odometry gps_corr;
  gps_corr.header = msg->header;
  gps_corr.pose = msg->pose;
  gps_corr.pose.pose.position.x = msg->pose.pose.position.x - this->optimization_->getPriorError().x();
  gps_corr.pose.pose.position.y = msg->pose.pose.position.y - this->optimization_->getPriorError().y();
  gps_corr.twist = msg->twist;
  gps_corr.child_frame_id = msg->child_frame_id;
  this->gpscorrected_publisher_.publish(gps_corr);

  //// DEBUG
  //std::cout << "x_error: " << this->optimization_->getPriorError().x() << std::endl;
  //std::cout << "y_error: " << this->optimization_->getPriorError().y() << std::endl;

  this->alg_.unlock();
}

void GeoLocalizationAlgNode::detc_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  //ROS_INFO("GeoLocalizationAlgNode::detc_callback: New Message Received");
  this->alg_.lock();

  double ini, end;
  ini = ros::Time::now().toSec();

  pcl::PCLPointCloud2 detect_pcl2;
  this->last_detect_pcl_.clear();
  pcl_conversions::toPCL(*msg, detect_pcl2);
  pcl::fromPCLPointCloud2(detect_pcl2, this->last_detect_pcl_);

  int id = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).id;

  //////////////////////////////////////////////////////////////////////////////////////////////
  //// 1) DA: Generate Landmarks in interface from map.
  data_processing::Pose2D position;
  position.x = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).p.x();
  position.y = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).p.y();
  this->data_->createLandmarksFromMap(position);

  // Transform landmarks to base frame.
  data_processing::Tf tf_base2map;
  tf_base2map.linear() = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).q.toRotationMatrix();
  tf_base2map.translation() = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).p;
  this->data_->applyTfFromLandmarksToBaseFrame(tf_base2map);

  // Parse interface landmarks to PCL and plot it.
  this->data_->parseLandmarksToPcl(this->base_id_);
  this->landmarks_publisher_.publish(*this->data_->getLandmarksPcl());

  //// 2) DA: Generate detections in interface from msg.
  data_processing::PolylineMap detections;
  data_processing::Polyline positions_way;
  for (int i = 0; i < this->last_detect_pcl_.points.size(); i++){
    float point_sf_x = this->last_detect_pcl_.points.at(i).x;
    float point_sf_y = this->last_detect_pcl_.points.at(i).y;
    float distance = sqrt(pow(point_sf_x, 2) + pow(point_sf_y, 2));
    if (distance < this->data_config_.radious_dt){
      data_processing::PolylinePoint pt;
      pt.x = point_sf_x;
      pt.y = point_sf_y;
      pt.z = 0.0;
      pt.id = i;
      positions_way.push_back(pt);
    }
  }
  detections.push_back(positions_way);
  this->data_->setDetections(detections);

  // Transform detections to base frame.
  tf::StampedTransform tf_lidar2base;
  try
  {
    this->listener_.lookupTransform(this->lidar_id_, this->base_id_, ros::Time(0), tf_lidar2base);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("[draw_frames] TF exception cb_getGpsOdomMsg:\n%s", ex.what());
  }
  Eigen::Affine3d af_lidar2base;
  tf::transformTFToEigen(tf_lidar2base, af_lidar2base);
  data_processing::Tf tr_lidar2base;
  tr_lidar2base.linear() = af_lidar2base.linear();
  tr_lidar2base.translation() = af_lidar2base.translation();
  this->data_->applyTfFromDetectionsToBaseFrame(tr_lidar2base);


  // Parse interface detections to PCL and plot it.
  this->data_->parseDetectionsToPcl(this->base_id_);

  //// 3) DA: Compute data association
  //// ICP
  std_msgs::Float64 asso_weight;
  Eigen::Matrix4d tf;
  data_processing::AssociationsVector associations;
  this->data_->dataAssociationIcp(this->base_id_, tf, associations);
  this->data_->parseAssociationsLmToPcl(this->map_id_, associations);
  this->data_->parseAssociationsDtToPcl(this->base_id_, associations);
  if (this->asso_preweight_ < 0) this->asso_weight_ = this->data_->dataInformation();
  else this->asso_weight_ = this->asso_preweight_;
  asso_weight.data = this->asso_weight_;
  this->wa_publisher_.publish(asso_weight);
  this->corregist_publisher_.publish(*this->data_->getAssociatedLmPcl());
  //this->detection_publisher_.publish(*this->data_->getAssociatedDtPcl());
  this->detection_publisher_.publish(*this->data_->getDetectionsPcl());

  //// 4) DA: Generate associations TF constraint
  //std::cout << "DATA INFORMATION: " << this->asso_weight_ << std::endl;
  bool key_frame = this->count_ > this->margin_asso_constraints_;
  if (key_frame){
    optimization_process::AssoPointsConstraintsSingleShot constraints_asso_pt_ss;
    for (int i = 0; i < associations.size(); i++){
      optimization_process::AssoPointsConstraint constraints_asso_pt;
      
      constraints_asso_pt.id = id;
      constraints_asso_pt.landmark = associations.at(i).first;
      constraints_asso_pt.detection = associations.at(i).second;

      constraints_asso_pt.asso_weight = this->asso_weight_;
      
      constraints_asso_pt.covariance = this->optimization_->getTrajectoryEstimated().at(this->optimization_->getTrajectoryEstimated().size()-1).covariance.block<3, 3>(0, 0);
      constraints_asso_pt.information = constraints_asso_pt.covariance.inverse();

      constraints_asso_pt_ss.push_back(constraints_asso_pt);
    }
    this->count_ = this->margin_asso_constraints_ + 1;
    this->optimization_->addAssoPointConstraintsSingleShot (constraints_asso_pt_ss);
    this->flag_gps_corr_ = true;
  }

  // loop time
  end = ros::Time::now().toSec();
  //std::cout << "DATA TIME LOOP: " << end - ini << std::endl;
  //std::cout << "DATA SEQUENCE: " << msg->header.seq << std::endl;

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
  this->asso_weight_ = config.asso_weight;
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

  this->tf_to_utm_.header.frame_id = this->world_id_;
  this->tf_to_utm_.child_frame_id = this->map_id_;
  this->tf_to_utm_.header.stamp = ros::Time::now();
  this->tf_to_utm_.transform.translation.x = utm_x + this->offset_map_x_;
  this->tf_to_utm_.transform.translation.y = utm_y + this->offset_map_y_;
  this->tf_to_utm_.transform.translation.z = 0.0;
  this->tf_to_utm_.transform.rotation = tf::createQuaternionMsgFromYaw(0.0/*3.1415 / 2.0*/);

  this->broadcaster_.sendTransform(this->tf_to_utm_);

  return;
}

void GeoLocalizationAlgNode::mapToOdomInit(void)
{

  this->tf_to_map_.header.frame_id = this->map_id_;
  this->tf_to_map_.child_frame_id = this->odom_id_;
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

  marker.header.frame_id = this->map_id_;
  marker.header.stamp = ros::Time::now();
  marker_line.header.frame_id = this->map_id_;
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
      marker.pose.position.z = 0.0; //this->map_.at(i).at(j).z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.id = id;
      id++;
      marker_array.markers.push_back(marker);

      ////////////////////////////////////////////////////////////////////////////////
      ///// SAVE MAP
      if (this->save_map_){
        std::ostringstream out_path_map;
        std::ostringstream out_map;
        std::ofstream file_map;

        out_path_map << this->out_map_ << id << "_landmark.csv";

        file_map.open(out_path_map.str().c_str(), std::ofstream::trunc);
        out_map << id << ", " <<
                   this->map_.at(i).at(j).x << ", " << 
                   this->map_.at(i).at(j).y << "\n";
        file_map << out_map.str();
        file_map.close();
      }
      ////////////////////////////////////////////////////////////////////////////////

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
  if (index > 0) this->optimization_->generateAssoPointResiduals(loss_function, quaternion_local_parameterization, &problem);
  if (index > 0) this->optimization_->generatePriorErrorResiduals(loss_function, quaternion_local_parameterization, &problem);

	//////////////////////////////////////////////////////////////////////
	//// SOLVE OPTIMIZATION PROBLEM
	this->optimization_->solveOptimizationProblem(&problem);

  return;
}

void GeoLocalizationAlgNode::computeOptimizationProblemGT (void)
{
	//////////////////////////////////////////////////////////////////////
	//// RESIDUALS GENERATION
  std::cout << "GT RESIDUALS GENERATION !!!" << std::endl;
	size_t index = this->optimization_->getTrajectoryEstimatedGT().size() - 1;
	ceres::Problem problem;
	ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;
	ceres::LossFunction* loss_function = new ceres::HuberLoss(0.01); //nullptr;//new ceres::HuberLoss(0.01);
	if (index > 0) this->optimization_->generateOdomResidualsGT(loss_function, quaternion_local_parameterization, &problem);
  if (index > 0) this->optimization_->generatePriorResidualsGT(loss_function, quaternion_local_parameterization, &problem);

	//////////////////////////////////////////////////////////////////////
	//// SOLVE OPTIMIZATION PROBLEM
  std::cout << "GT SOLVE OPTIMIZATION PROBLEM !!!" << std::endl;
	this->optimization_->solveOptimizationProblem(&problem);

  //////////////////////////////////////////////////////////////////////
	//// SAVE DATA FOR EVALUATION
  std::cout << "GT SAVE DATA FOR EVALUATION !!!" << std::endl;
  for (int i = 0; i < this->optimization_->getTrajectoryEstimatedGT().size(); i++){
    std::ostringstream out_path_pos2d;
    std::ostringstream out_pos2d;
    std::ofstream file_pos2d;

    float yaw;
    float x = this->optimization_->getTrajectoryEstimatedGT().at(i).q.x();
    float y = this->optimization_->getTrajectoryEstimatedGT().at(i).q.y();
    float z = this->optimization_->getTrajectoryEstimatedGT().at(i).q.z();
    float w = this->optimization_->getTrajectoryEstimatedGT().at(i).q.w();
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    out_path_pos2d << this->out_gt_ << i << "_pose2d.csv";

    file_pos2d.open(out_path_pos2d.str().c_str(), std::ofstream::trunc);
    out_pos2d << i << ", " <<
                 this->optimization_->getTrajectoryEstimatedGT().at(i).p.x() << ", " << 
                 this->optimization_->getTrajectoryEstimatedGT().at(i).p.y() << ", " << 
                 yaw << "\n";
    file_pos2d << out_pos2d.str();
    file_pos2d.close();
  }
  std::cout << "FINISHED GT SAVE DATA FOR EVALUATION !!!" << std::endl;
  
  return;
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<GeoLocalizationAlgNode>(argc, argv, "geo_localization_alg_node");
}
