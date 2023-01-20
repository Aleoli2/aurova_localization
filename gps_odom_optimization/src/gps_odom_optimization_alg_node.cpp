#include "gps_odom_optimization_alg_node.h"

GpsOdomOptimizationAlgNode::GpsOdomOptimizationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<GpsOdomOptimizationAlgorithm>()
{
  //init class attributes if necessary
  this->gps_received_ = false;
  this->optimization_ = new OptimizationProcess();
  if(!this->private_node_handle_.getParam("rate", this->config_.rate))
  {
	  ROS_WARN("GpsOdomOptimizationAlgNode::GpsOdomOptimizationAlgNode: param 'rate' not found");
  }
  else
	  this->setRate(this->config_.rate);
  this->public_node_handle_.param("x_model", this->config_.x_model,0.0);
  this->public_node_handle_.param("y_model", this->config_.y_model,0.0);
  this->public_node_handle_.param("theta_model", this->config_.theta_model,0.0);
  this->public_node_handle_.param("namespace", this->ns, std::string(""));

  // [init publishers]
  this->localization_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("localization", 1);
  
  // [init subscribers]
  this->odometry_gps_subscriber_ = this->public_node_handle_.subscribe("odometry_gps", 1, &GpsOdomOptimizationAlgNode::odometry_gps_callback, this);
  pthread_mutex_init(&this->odometry_gps_mutex_,NULL);

  this->odom_subscriber_ = this->public_node_handle_.subscribe("odom", 1, &GpsOdomOptimizationAlgNode::odom_callback, this);
  pthread_mutex_init(&this->odom_mutex_,NULL);

  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

GpsOdomOptimizationAlgNode::~GpsOdomOptimizationAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->odometry_gps_mutex_);
  pthread_mutex_destroy(&this->odom_mutex_);
}

void GpsOdomOptimizationAlgNode::mainNodeThread(void)
{
  //lock access to algorithm if necessary
  this->alg_.lock();

  ROS_DEBUG("GpsOdomOptimizationAlgNode::mainNodeThread");

  // [fill msg structures]

  // Uncomment the following line to publish the topic message
  //this->localization_publisher_.publish(this->localization_Odometry_msg_);

  this->alg_.unlock();
}

/*  [subscriber callbacks] */
void GpsOdomOptimizationAlgNode::odometry_gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("GpsOdomOptimizationAlgNode::odometry_gps_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  this->odometry_gps_mutex_enter();

  this->gps_odom_msg_ = *msg;
  this->gps_received_ = true;

  //unlock previously blocked shared variables
  this->alg_.unlock();
  this->odometry_gps_mutex_exit();
}

void GpsOdomOptimizationAlgNode::odometry_gps_mutex_enter(void)
{
  pthread_mutex_lock(&this->odometry_gps_mutex_);
}

void GpsOdomOptimizationAlgNode::odometry_gps_mutex_exit(void)
{
  pthread_mutex_unlock(&this->odometry_gps_mutex_);
}

void GpsOdomOptimizationAlgNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("GpsOdomOptimizationAlgNode::odom_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  this->odom_mutex_enter();

  static size_t id = 0;

  ////////////////////////////////////////////////////////////////////////////////
  ///// GENERATE CURRENT ODOM POSE
  tf::StampedTransform tf_odom2base;
  try
  {
    this->tf_listener_.lookupTransform(this->ns+"odom", this->ns+"base_link", ros::Time(0), tf_odom2base);
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

  
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////


  if (this->gps_received_){

	  this->gps_received_ = false;

    tf::StampedTransform tf_odom2gps;
    try
    {
      this->tf_listener_.lookupTransform(this->ns+"odom", this->ns+"gps", ros::Time(0), tf_odom2gps);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("[draw_frames] TF exception cb_getGpsOdomMsg:\n%s", ex.what());
    }
    geometry_msgs::Pose pose_odom2gps;
    tf::poseTFToMsg(tf_odom2gps, pose_odom2gps);

    double x = pose_odom2gps.position.x;
    double y = pose_odom2gps.position.y;

	  ////////////////////////////////////////////////////////////////////////////////
	  ///// GENERATE CURRENT GPS POSE
	  double x_gps = this->gps_odom_msg_.pose.pose.position.x;
	  double y_gps = this->gps_odom_msg_.pose.pose.position.y;
	  ////////////////////////////////////////////////////////////////////////////////

	  ////////////////////////////////////////////////////////////////////////////////
	  //// POINT CONSTRAINTS GENERATION
      PointsConstraint constraint_pt;

	  constraint_pt.id = id;
	  constraint_pt.detection.x() = x;
	  constraint_pt.detection.y() = y;
	  constraint_pt.detection.z() = 0.0;
	  constraint_pt.landmark.x() = x_gps;
	  constraint_pt.landmark.y() = y_gps;
	  constraint_pt.landmark.z() = 0.0;

    //Inverse square root matrix of the covariance.
    constraint_pt.covariance=Eigen::Matrix<double, 3, 3>::Identity();
    
	  constraint_pt.information = Eigen::Matrix<double, 3, 3>::Identity();
    double A=this->gps_odom_msg_.pose.covariance[0], B=this->gps_odom_msg_.pose.covariance[1],
    C=this->gps_odom_msg_.pose.covariance[6], D=this->gps_odom_msg_.pose.covariance[7],
    t=sqrt(A+D+2*sqrt(D*A-B*C));
    double A2=A+sqrt(A*D-B*C)/t, B2=B/t, C2=C/t, D2=C+sqrt(A*D-B*C)/t; 

    constraint_pt.information(0,0) =D2/(A2*D2-B2*C2);
    constraint_pt.information(0,1) =-B2/(A2*D2-B2*C2);
    constraint_pt.information(1,0) =-C2/(A2*D2-B2*C2);
    constraint_pt.information(1,1) =A2/(A2*D2-B2*C2);

	  this->optimization_->addPointConstraint(constraint_pt);
	  ////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////////////////////////////////////////////////////////////

	  if (this->optimization_->checkOptimization()){
		  ////////////////////////////////////////////////////////////////////////////////
		  //// COMPUTE OPTIMIZATION PROBLEM
		  // residuals generation
		  ceres::Problem problem;
		  ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;
		  ceres::LossFunction* loss_function = new ceres::HuberLoss(0.01); //nullptr;//new ceres::HuberLoss(0.01);
		  this->optimization_->generatePointResiduals(loss_function, quaternion_local_parameterization, &problem);

		  // solve optimization problem
		  this->optimization_->solveOptimizationProblem(&problem);
      this->optimization_->estimateCovariance(&problem);
      Covariance=this->optimization_->getMapToOdom().covariance;
		  ////////////////////////////////////////////////////////////////////////////////
		  ////////////////////////////////////////////////////////////////////////////////
	  }
  }

  else{
    //Update covariance, if the robot isn't stationary.
    double speed_x = (*msg).twist.twist.linear.x, speed_y = (*msg).twist.twist.linear.y;
    if(sqrt(pow(speed_x,2)+pow(speed_y,2))>0.01){
      Covariance(0,0)+=this->config_.x_model;
      Covariance(1,1)+=this->config_.y_model;
      Covariance(5,5)+=this->config_.theta_model;
    } 
  }


  ////////////////////////////////////////////////////////////////////////////////
  ///// GENERATE map -> odom TRANSFORM
  // generate 4x4 transform matrix map2odom
  //tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
  Eigen::Matrix4d tr_map2odom;
  tr_map2odom.setIdentity();
  Eigen::Quaterniond rot(this->optimization_->getMapToOdom().q.w(),
		  	  	  	  	 this->optimization_->getMapToOdom().q.x(),
						 this->optimization_->getMapToOdom().q.y(),
						 this->optimization_->getMapToOdom().q.z());
  tr_map2odom.block<3, 3>(0, 0) = rot.toRotationMatrix();
  tr_map2odom.block<3, 1>(0, 3) = Eigen::Vector3d(this->optimization_->getMapToOdom().p.x(),
		                                          this->optimization_->getMapToOdom().p.y(), 0.0);


  Eigen::Quaterniond quat_final(tr_map2odom.block<3, 3>(0, 0));

  this->transform_msg_.header.frame_id = "map";
  this->transform_msg_.child_frame_id = this->ns+"odom";
  this->transform_msg_.header.stamp = ros::Time::now();

  this->transform_msg_.transform.translation.x = tr_map2odom(0, 3);
  this->transform_msg_.transform.translation.y = tr_map2odom(1, 3);
  this->transform_msg_.transform.translation.z = tr_map2odom(2, 3);
  this->transform_msg_.transform.rotation.x = quat_final.x();
  this->transform_msg_.transform.rotation.y = quat_final.y();
  this->transform_msg_.transform.rotation.z = quat_final.z();
  this->transform_msg_.transform.rotation.w = quat_final.w();

  this->tf_broadcaster_.sendTransform(this->transform_msg_);
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////


  Eigen::Matrix4d tr_map2base;
  tr_map2base = tr_map2odom * tr_odom2base;

  Eigen::Quaterniond quat_msg(tr_map2base.block<3, 3>(0, 0));

  this->localization_Odometry_msg_.header.seq = id;
  this->localization_Odometry_msg_.header.stamp = ros::Time::now();
  this->localization_Odometry_msg_.header.frame_id = "map";
  this->localization_Odometry_msg_.child_frame_id = this->ns+"base_link";
  this->localization_Odometry_msg_.pose.pose.position.x = tr_map2base(0, 3);
  this->localization_Odometry_msg_.pose.pose.position.y = tr_map2base(1, 3);
  this->localization_Odometry_msg_.pose.pose.position.z = tr_map2base(2, 3);

  this->localization_Odometry_msg_.pose.pose.orientation.x = quat_msg.x();
  this->localization_Odometry_msg_.pose.pose.orientation.y = quat_msg.y();
  this->localization_Odometry_msg_.pose.pose.orientation.z = quat_msg.z();
  this->localization_Odometry_msg_.pose.pose.orientation.w = quat_msg.w();

  
  this->localization_Odometry_msg_.pose.covariance[0]=Covariance(0,0);
  this->localization_Odometry_msg_.pose.covariance[1]=Covariance(0,1);
  this->localization_Odometry_msg_.pose.covariance[6]=Covariance(1,0);
  this->localization_Odometry_msg_.pose.covariance[7]=Covariance(1,1);
  this->localization_Odometry_msg_.pose.covariance[14]=1;
  this->localization_Odometry_msg_.pose.covariance[35]=Covariance(5,5);

  this->localization_publisher_.publish(this->localization_Odometry_msg_);

  id++;

  //unlock previously blocked shared variables
  this->alg_.unlock();
  this->odom_mutex_exit();
}

void GpsOdomOptimizationAlgNode::odom_mutex_enter(void)
{
  pthread_mutex_lock(&this->odom_mutex_);
}

void GpsOdomOptimizationAlgNode::odom_mutex_exit(void)
{
  pthread_mutex_unlock(&this->odom_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void GpsOdomOptimizationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  if(config.rate!=this->getRate())
    this->setRate(config.rate);
  this->config_=config;
  this->alg_.unlock();
}

void GpsOdomOptimizationAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<GpsOdomOptimizationAlgNode>(argc, argv, "gps_odom_optimization_alg_node");
}


/*
//tf_listener example BEGIN
try{
  std::string target_frame             = "child_frame";
  std::string source_frame             = "parent_frame";
  ros::Time time                       = ros::Time::now();
  ros::Duration timeout                = ros::Duration(0.1);
  ros::Duration polling_sleep_duration = ros::Duration(0.01);
  this->alg_.unlock();
  bool tf_exists = this->tf_listener_.waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration);
  this->alg_.lock();
  if(tf_exists){
    geometry_msgs::PoseStamped stamped_pose_in;
    stamped_pose_in.header.stamp     = time;
    stamped_pose_in.header.frame_id  = source_frame;
    stamped_pose_in.pose.position.x    = 1.0;
    stamped_pose_in.pose.position.y    = 0.0;
    stamped_pose_in.pose.position.z    = 0.0;
    stamped_pose_in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    ROS_INFO("Original    pose in '%s' frame, with (x,y,z)=[%f,%f,%f], yaw=[%f]", stamped_pose_in.header.frame_id.c_str(), stamped_pose_in.pose.position.x, stamped_pose_in.pose.position.y, stamped_pose_in.pose.position.z, tf::getYaw(stamped_pose_in.pose.orientation));
    geometry_msgs::PoseStamped stamped_pose_out;
    this->tf_listener_.transformPose(target_frame, stamped_pose_in, stamped_pose_out);
    ROS_INFO("Transformed pose in '%s'  frame, with (x,y,z)=[%f,%f,%f], yaw=[%f]", stamped_pose_out.header.frame_id.c_str(), stamped_pose_out.pose.position.x, stamped_pose_out.pose.position.y, stamped_pose_out.pose.position.z, tf::getYaw(stamped_pose_out.pose.orientation));
    ROS_INFO("---");


    tf::StampedTransform tf_parent_child;
    tf::Transform tf_parent_point, tf_child_point;
    this->tf_listener_.lookupTransform(source_frame, target_frame, time, tf_parent_child);
    tf_parent_point.setOrigin(tf::Vector3(stamped_pose_in.pose.position.x, stamped_pose_in.pose.position.y, stamped_pose_in.pose.position.z));
    tf_parent_point.setRotation(tf::Quaternion(stamped_pose_in.pose.orientation.x, stamped_pose_in.pose.orientation.y, stamped_pose_in.pose.orientation.z, stamped_pose_in.pose.orientation.w));
    tf_child_point = tf_parent_child.inverse()*tf_parent_point;
    ROS_INFO("Transformed pose in '%s'  frame, with (x,y,z)=[%f,%f,%f], yaw=[%f]", target_frame.c_str(), tf_child_point.getOrigin().x(), tf_child_point.getOrigin().y(), tf_child_point.getOrigin().z(), tf::getYaw(tf_child_point.getRotation()));
    ROS_INFO("---");
  }else{
    ROS_WARN("No transform found from '%s' to '%s'", source_frame.c_str(), target_frame.c_str()); }
}catch (tf::TransformException &ex){
  ROS_ERROR("TF Exception: %s",ex.what()); }
///tf_listener example END
*/

// Initialize the topic message structure
//this->localization_PoseWithCovarianceStamped_msg_.data = my_var;


// [fill srv structure and make request to the server]

// [fill action structure and make request to the action server]

// [publish messages]

/*
//tf_broadcaster example
this->transform_msg_.header.stamp    = ros::Time::now();
this->transform_msg_.header.frame_id = "parent_frame";
this->transform_msg_.child_frame_id  = "child_frame";
geometry_msgs::Transform t;
t.translation.x = 0.0;
t.translation.y = 0.0;
t.translation.z = 0.0;
t.rotation = tf::createQuaternionMsgFromYaw(0.0);
this->transform_msg_.transform = t;
this->tf_broadcaster_.sendTransform(this->transform_msg_);
///tf_broadcaster example
*/
