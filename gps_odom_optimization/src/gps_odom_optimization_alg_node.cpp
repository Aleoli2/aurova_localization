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

  // [init publishers]
  this->localization_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("localization", 1);
  
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

  // Uncomment the following line to publish the topic message
  //this->localization_publisher_.publish(this->localization_PoseWithCovarianceStamped_msg_);

  
  this->alg_.unlock();
}

/*  [subscriber callbacks] */
void GpsOdomOptimizationAlgNode::odometry_gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("GpsOdomOptimizationAlgNode::odometry_gps_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  this->odometry_gps_mutex_enter();

  ////////////////////////////////////////////////////////////////////////////////
  ///// GENERATE CURRENT GPS POSE
  // get orientation information
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
		           msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw, x, y, w;
  m.getRPY(roll, pitch, yaw);

  // get position information
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  w = yaw;
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

  size_t index = this->optimization_->getTrajectoryOdom().size() - 1;

  ////////////////////////////////////////////////////////////////////////////////
  //// PRIOR CONSTRAINT GENERATION
  if (index > 0){
	  PriorConstraint constraint_prior;
	  constraint_prior.id = this->optimization_->getTrajectoryOdom().at(index).id;

	  constraint_prior.p.x() = x;
	  constraint_prior.p.y() = y;
	  constraint_prior.p.z() = 0.0;

	  Eigen::AngleAxisd yaw_angle(w, Eigen::Vector3d::UnitZ());
	  constraint_prior.q = yaw_angle;

	  constraint_prior.covariance = Eigen::Matrix<double, 6, 6>::Identity();
	  int count = 0;
	  for (int i = 0; i < 6; i++){
		  for (int j = 0; j < 6; j++){
			  constraint_prior.covariance(j, i) = msg->pose.covariance[count];
			  count++;
		  }
	  }
	  constraint_prior.covariance = constraint_prior.covariance + Eigen::Matrix<double, 6, 6>::Identity()*10.0;
	  constraint_prior.information = constraint_prior.covariance.inverse();

	  this->optimization_->addPriorConstraint(constraint_prior);

	  this->gps_received_ = true;
  }
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////


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
  // get orientation information
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
		           msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw, x, y, w;
  m.getRPY(roll, pitch, yaw);

  // get position information
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  w = yaw;
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////



  ////////////////////////////////////////////////////////////////////////////////
  ///// COMPUTE ODOMETRI AND PRE-ESTIMATION STATE
  this->optimization_->addPose3dToTrajectoryOdom(parsePose2dToPose3d (id, x, y, w, Eigen::Matrix<double, 6, 6>::Zero()));
  size_t index = this->optimization_->getTrajectoryOdom().size() - 1;
  this->optimization_->propagateState(index);
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////


  ////////////////////////////////////////////////////////////////////////////////
  //// ODOMETRY CONSTRAINTS GENERATION
  OdometryConstraint constraint_odom;
  if (index > 0){
	  Eigen::Matrix<double, 3, 1> p_a(this->optimization_->getTrajectoryOdom().at(index-1).p);
	  Eigen::Quaternion<double> q_a(this->optimization_->getTrajectoryOdom().at(index-1).q);
	  Eigen::Matrix<double, 3, 1> p_b(this->optimization_->getTrajectoryOdom().at(index).p);
	  Eigen::Quaternion<double> q_b(this->optimization_->getTrajectoryOdom().at(index).q);

	  // Compute the relative transformation between the two frames.
	  Eigen::Quaternion<double> q_a_inverse = q_a.conjugate();
	  Eigen::Quaternion<double> q_ab_estimated = q_a_inverse * q_b;

	  // Represent the displacement between the two frames in the A frame.
	  Eigen::Matrix<double, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

	  constraint_odom.id_begin = this->optimization_->getTrajectoryOdom().at(index-1).id;
	  constraint_odom.id_end = this->optimization_->getTrajectoryOdom().at(index).id;

	  constraint_odom.tf_p = p_ab_estimated;
	  constraint_odom.tf_q = q_ab_estimated;

	  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
	  constraint_odom.covariance = covariance;
	  constraint_odom.information = constraint_odom.covariance.inverse();

	  this->optimization_->addOdometryConstraint(constraint_odom);


  }
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////





		////////////////////////////////////////////////////////////////////////////////
		//// COMPUTE OPTIMIZATION PROBLEM
		double ini, end;
		ini = ros::Time::now().toSec();

		// residuals generation
		ceres::Problem problem;
		ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;
		ceres::LossFunction* loss_function = new ceres::HuberLoss(0.01); //nullptr;//new ceres::HuberLoss(0.01);
		this->optimization_->generatePriorResiduals(loss_function, quaternion_local_parameterization, &problem);
		this->optimization_->generateOdomResiduals(loss_function, quaternion_local_parameterization, &problem);

		// solve optimization problem
		if (index > 0) this->optimization_->solveOptimizationProblem(&problem);

		// loop time
		end = ros::Time::now().toSec();
		ROS_INFO("duration total: %f", end - ini);
		////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////




  ////////////////////////////////////////////////////////////////////////////////
  ///// GENERATE map -> odom TRANSFORM
  tf::StampedTransform tf_odom2base;
  try
  {
    this->tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), tf_odom2base);
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

  // generate 4x4 transform matrix map2base (TODO: Sustituir por optimization result!!!)
  //tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
  Eigen::Matrix4d tr_map2base;
  tr_map2base.setIdentity();
  Eigen::Quaterniond rot(this->optimization_->getTrajectoryEstimated().at(index).q.w(),
		                 this->optimization_->getTrajectoryEstimated().at(index).q.x(),
						 this->optimization_->getTrajectoryEstimated().at(index).q.y(),
						 this->optimization_->getTrajectoryEstimated().at(index).q.z());
  tr_map2base.block<3, 3>(0, 0) = rot.toRotationMatrix();
  tr_map2base.block<3, 1>(0, 3) = Eigen::Vector3d(this->optimization_->getTrajectoryEstimated().at(index).p.x(),
		                                          this->optimization_->getTrajectoryEstimated().at(index).p.y(), 0.0);

  // given: odom2base * map2odom = map2base
  // thenn: map2odom = map2base * odom2base^(-1)
  Eigen::Matrix4d tr_map2odom;
  tr_map2odom = tr_map2base * tr_odom2base.inverse();

  Eigen::Quaterniond quat_final(tr_map2odom.block<3, 3>(0, 0));

  this->transform_msg_.header.frame_id = "map";
  this->transform_msg_.child_frame_id = "odom";
  this->transform_msg_.header.stamp = ros::Time::now();

  if (this->gps_received_){
	  this->transform_msg_.transform.translation.x = tr_map2odom(0, 3);
	  this->transform_msg_.transform.translation.y = tr_map2odom(1, 3);
	  this->transform_msg_.transform.translation.z = tr_map2odom(2, 3);
	  this->transform_msg_.transform.rotation.x = quat_final.x();
	  this->transform_msg_.transform.rotation.y = quat_final.y();
	  this->transform_msg_.transform.rotation.z = quat_final.z();
	  this->transform_msg_.transform.rotation.w = quat_final.w();
  }else{
	  this->transform_msg_.transform.translation.x = 0.0;
	  this->transform_msg_.transform.translation.y = 0.0;
	  this->transform_msg_.transform.translation.z = 0.0;
	  this->transform_msg_.transform.rotation.x = 0.0;
	  this->transform_msg_.transform.rotation.y = 0.0;
	  this->transform_msg_.transform.rotation.z = 0.0;
	  this->transform_msg_.transform.rotation.w = 1.0;
  }

  this->tf_broadcaster_.sendTransform(this->transform_msg_);
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////


  ////////////////////////////////////////////////////////////////////////////////
  ///// GENERATE velodyne -> base_link TRANSFORM
  //// TODO: FROM URDF!!!!!!!!
  this->transform_msg_.header.frame_id = "velodyne";
  this->transform_msg_.child_frame_id = "base_link";
  this->transform_msg_.header.stamp = ros::Time::now();

  this->transform_msg_.transform.translation.x = -0.55;
  this->transform_msg_.transform.translation.y = 0.0;
  this->transform_msg_.transform.translation.z = -0.645;
  this->transform_msg_.transform.rotation.x = 0.0;
  this->transform_msg_.transform.rotation.y = 0.0;
  this->transform_msg_.transform.rotation.z = 0.0;
  this->transform_msg_.transform.rotation.w = 1.0;

  this->tf_broadcaster_.sendTransform(this->transform_msg_);
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////


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
