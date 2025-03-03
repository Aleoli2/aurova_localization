#include "pose_simulation_alg.h"

PoseSimulationAlgorithm::PoseSimulationAlgorithm(void)
{
  pthread_mutex_init(&this->access_, NULL);
}

PoseSimulationAlgorithm::~PoseSimulationAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void PoseSimulationAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

// PoseSimulationAlgorithm Public API
void PoseSimulationAlgorithm::generateNewPoseMsg2D(tf::StampedTransform pose_current,
                                                   ackermann_msgs::AckermannDrive ackermann_state,
                                                   geometry_msgs::PoseWithCovarianceStamped& pose_sim,
                                                   geometry_msgs::TransformStamped& pose_tf,
                                                   struct Covariance st_cobariance, float d_vehicle,
                                                   std::string frame_id, std::string child_id)
{
  static double t_1;
  static double t_2;
  static bool first_exec = true;

  //calculate increment of time
  if (first_exec)
  {
    t_2 = (double)ros::Time::now().toSec();
    first_exec = false;
  }
  t_1 = (double)ros::Time::now().toSec();
  float delta_t = (float)(t_1 - t_2);
  t_2 = (double)ros::Time::now().toSec();

  // get XYZ and RPY from current pose
  double r_current;
  double p_current;
  double w_current;
  double x_current = pose_current.getOrigin().x();
  double y_current = pose_current.getOrigin().y();
  double z_current = pose_current.getOrigin().z();
  tf::Quaternion quaternion_aux(pose_current.getRotation().x(), pose_current.getRotation().y(),
                                pose_current.getRotation().z(), pose_current.getRotation().w());
  tf::Matrix3x3 matrix(quaternion_aux);
  matrix.getRPY(r_current, p_current, w_current);

  //read information of low-level sensor
  float lineal_speed = ackermann_state.speed;
  float steering_radians = ackermann_state.steering_angle * M_PI / 180.0;

  // speed and pose calculations
  float angular_speed_yaw = (lineal_speed / d_vehicle) * sin(steering_radians);
  float pose_yaw = w_current + angular_speed_yaw * delta_t;
  float lineal_speed_x = lineal_speed * cos(pose_yaw) * cos(steering_radians);
  float lineal_speed_y = lineal_speed * sin(pose_yaw) * cos(steering_radians);
  float pose_x = x_current + lineal_speed_x * delta_t;
  float pose_y = y_current + lineal_speed_y * delta_t;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, pose_yaw);

  /////////////////////////////////////////////////
  //// GENERATE MESSAGE
  pose_sim.header.stamp = ros::Time::now();
  pose_sim.header.frame_id = frame_id.c_str();
  pose_sim.pose.pose.position.x = pose_x;
  pose_sim.pose.pose.position.y = pose_y;
  pose_sim.pose.pose.position.z = z_current;
  pose_sim.pose.pose.orientation.x = quaternion[0];
  pose_sim.pose.pose.orientation.y = quaternion[1];
  pose_sim.pose.pose.orientation.z = quaternion[2];
  pose_sim.pose.pose.orientation.w = quaternion[3];
  pose_sim.pose.covariance[0] = st_cobariance.x;
  pose_sim.pose.covariance[7] = st_cobariance.y;
  pose_sim.pose.covariance[14] = st_cobariance.z;
  pose_sim.pose.covariance[35] = st_cobariance.w;
  /////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////
  //// GENERATE MESSAGES TF
  pose_tf.header.frame_id = frame_id.c_str();
  pose_tf.child_frame_id = child_id.c_str();
  pose_tf.header.stamp = ros::Time::now();
  pose_tf.transform.translation.x = pose_x;
  pose_tf.transform.translation.y = pose_y;
  pose_tf.transform.translation.z = z_current;
  pose_tf.transform.rotation = tf::createQuaternionMsgFromYaw(pose_yaw);
  ////////////////////////////////////////////////////////////////

  return;
}
