#include "ekf_loose_integration_alg_node.h"
#include <XmlRpcException.h>

EkfLooseIntegrationAlgNode::EkfLooseIntegrationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<EkfLooseIntegrationAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  rover_state_ = Eigen::Matrix<double, 9, 1>::Zero();

  flag_imu_initialized_ = false;
  previous_timestamp_ = 0.0;
  current_timestamp_ = 0.0;

  number_of_imu_readings_for_initialization_ = 50; // about two and half seconds at 20 Hz
  number_of_imu_readings_ = 0;
  accumlator_acc_ = Eigen::Vector3d::Zero();
  mean_init_acc_  = Eigen::Vector3d::Zero();

  acc_reading_       = Eigen::Vector3d::Zero();
  acc_corrected_     = Eigen::Vector3d::Zero();

  acc_bias_          = Eigen::Vector3d::Zero();
  acc_scale_factor_  = Eigen::Matrix3d::Zero();
  acc_misaligment_   = Eigen::Matrix3d::Zero();

  gyro_reading_      = Eigen::Vector3d::Zero();
  gyro_corrected_    = Eigen::Vector3d::Zero();

  gyro_bias_         = Eigen::Vector3d::Zero();
  gyro_scale_factor_ = Eigen::Matrix3d::Zero();
  gyro_misaligment_  = Eigen::Matrix3d::Zero();

  // Using calibration results obtained with imu_tk
  acc_bias_(0) = 0.968207;
  acc_bias_(1) = -1.64146;
  acc_bias_(2) = -2.16982;

  acc_scale_factor_(0,0) = 0.997842;
  acc_scale_factor_(1,1) = 0.993924;
  acc_scale_factor_(2,2) = 0.975817;

  acc_misaligment_(0,0) = 1.0;
  acc_misaligment_(0,1) = -0.0294515;
  acc_misaligment_(0,2) = -0.00142695;

  acc_misaligment_(1,1) = 1.0;
  acc_misaligment_(1,2) = 0.00266962;

  acc_misaligment_(2,2) = 1.0;


  gyro_bias_(0) = -0.000684136;
  gyro_bias_(1) = -0.000123011;
  gyro_bias_(2) = -0.000412331;

  gyro_scale_factor_(0,0) = 0.972766;
  gyro_scale_factor_(1,1) = 0.983783;
  gyro_scale_factor_(2,2) = 1.014500;

  gyro_misaligment_(0,0) = 1.0;
  gyro_misaligment_(0,1) = -0.00661431;
  gyro_misaligment_(0,2) = 0.012135;

  gyro_misaligment_(1,0) = -0.00860842;
  gyro_misaligment_(1,1) = 1.0;
  gyro_misaligment_(1,2) = 0.0352662;

  gyro_misaligment_(2,0) = -0.0185842;
  gyro_misaligment_(2,1) = 0.00442738;
  gyro_misaligment_(2,2) = 1.0;



  // [init publishers]
  this->pose_publisher_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped
      > ("/estimated_pose", 1);
  
  this->odom_publisher_ = this->public_node_handle_.advertise < nav_msgs::Odometry > ("/estimated_odometry", 1);

  // [init subscribers]
  this->estimated_ackermann_subscriber_ = this->public_node_handle_.subscribe(
      "/estimated_ackermann_state", 1, &EkfLooseIntegrationAlgNode::cb_ackermannState, this);
  
  this->imu_subscriber_ = this->public_node_handle_.subscribe(
      "/imu/data", 1, &EkfLooseIntegrationAlgNode::cb_imuData, this);

  this->AMCL_pose_sub_ = this->public_node_handle_.subscribe(
      "/amcl_pose", 1, &EkfLooseIntegrationAlgNode::cb_AMCLPose, this);

  this->odom_GNSS_sub_ = this->public_node_handle_.subscribe(
      "/odometry_gps", 1, &EkfLooseIntegrationAlgNode::cb_GNSSOdom, this);

  XmlRpc::XmlRpcValue accMisalignMatrixConfig;
  if (this->public_node_handle_.hasParam("/acc_misalign_matrix"))
  {
    try
    {
      this->public_node_handle_.getParam("/acc_misalign_matrix", accMisalignMatrixConfig);

      ROS_ASSERT(accMisalignMatrixConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = acc_misaligment_.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << accMisalignMatrixConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> acc_misaligment_(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading IMU calibration: " <<
                       e.getMessage() <<
                       " for misalign_matrix (type: " <<
                       accMisalignMatrixConfig.getType() << ")");
    }
    std::cout << "Loaded acc_misalign_matrix using rosparam: " << std::endl;
    std::cout << acc_misaligment_ << std::endl;
  }

  XmlRpc::XmlRpcValue accScaleMatrixConfig;
  if (this->public_node_handle_.hasParam("/acc_scale_matrix"))
  {
    try
    {
      this->public_node_handle_.getParam("/acc_scale_matrix", accScaleMatrixConfig);

      ROS_ASSERT(accScaleMatrixConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = acc_scale_factor_.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << accScaleMatrixConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> acc_scale_factor_(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading IMU calibration: " <<
                       e.getMessage() <<
                       " for acc_scale_factor_matrix (type: " <<
                       accScaleMatrixConfig.getType() << ")");
    }
    std::cout << "Loaded acc_scale_factor_matrix using rosparam: " << std::endl;
    std::cout << acc_scale_factor_ << std::endl;
  }

  XmlRpc::XmlRpcValue accBiasVectorConfig;
  if (this->public_node_handle_.hasParam("/acc_bias_vector"))
  {
    try
    {
      this->public_node_handle_.getParam("/acc_bias_vector", accBiasVectorConfig);

      ROS_ASSERT(accBiasVectorConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = acc_bias_.rows();

      for (int i = 0; i < matSize; i++)
      {
        try
        {
          // These matrices can cause problems if all the types
          // aren't specified with decimal points. Handle that
          // using string streams.
          std::ostringstream ostr;
          ostr << accBiasVectorConfig[i];
          std::istringstream istr(ostr.str());
          istr >> acc_bias_(i);
        }
        catch(XmlRpc::XmlRpcException &e)
        {
          throw e;
        }
        catch(...)
        {
          throw;
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading IMU calibration: " <<
                       e.getMessage() <<
                       " for acc_bias_vector (type: " <<
                       accBiasVectorConfig.getType() << ")");
    }
    std::cout << "Loaded acc_bias_vector using rosparam: " << std::endl;
    std::cout << acc_bias_ << std::endl;
  }



  ////////////////////////////////// Loading Gyro calibration!!! //////////////////////////////////////////////


  XmlRpc::XmlRpcValue gyroMisalignMatrixConfig;
  if (this->public_node_handle_.hasParam("/gyro_misalign_matrix"))
  {
    try
    {
      this->public_node_handle_.getParam("/gyro_misalign_matrix", gyroMisalignMatrixConfig);

      ROS_ASSERT(gyroMisalignMatrixConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = gyro_misaligment_.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << gyroMisalignMatrixConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> gyro_misaligment_(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading IMU calibration: " <<
                       e.getMessage() <<
                       " for misalign_matrix (type: " <<
                       gyroMisalignMatrixConfig.getType() << ")");
    }
    std::cout << "Loaded gyro_misalign_matrix using rosparam: " << std::endl;
    std::cout << gyro_misaligment_ << std::endl;
  }

  XmlRpc::XmlRpcValue gyroScaleMatrixConfig;
  if (this->public_node_handle_.hasParam("/gyro_scale_matrix"))
  {
    try
    {
      this->public_node_handle_.getParam("/gyro_scale_matrix", gyroScaleMatrixConfig);

      ROS_ASSERT(gyroScaleMatrixConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = gyro_scale_factor_.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << gyroScaleMatrixConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> gyro_scale_factor_(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading IMU calibration: " <<
                       e.getMessage() <<
                       " for gyro_scale_factor_matrix (type: " <<
                       gyroScaleMatrixConfig.getType() << ")");
    }
    std::cout << "Loaded gyro_scale_factor_matrix using rosparam: " << std::endl;
    std::cout << gyro_scale_factor_ << std::endl;
  }

  XmlRpc::XmlRpcValue gyroBiasVectorConfig;
  if (this->public_node_handle_.hasParam("/gyro_bias_vector"))
  {
    try
    {
      this->public_node_handle_.getParam("/gyro_bias_vector", gyroBiasVectorConfig);

      ROS_ASSERT(gyroBiasVectorConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = gyro_bias_.rows();

      for (int i = 0; i < matSize; i++)
      {
        try
        {
          // These matrices can cause problems if all the types
          // aren't specified with decimal points. Handle that
          // using string streams.
          std::ostringstream ostr;
          ostr << gyroBiasVectorConfig[i];
          std::istringstream istr(ostr.str());
          istr >> gyro_bias_(i);
        }
        catch(XmlRpc::XmlRpcException &e)
        {
          throw e;
        }
        catch(...)
        {
          throw;
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading IMU calibration: " <<
                       e.getMessage() <<
                       " for gyro_bias_vector (type: " <<
                       gyroBiasVectorConfig.getType() << ")");
    }
    std::cout << "Loaded gyro_bias_vector using rosparam: " << std::endl;
    std::cout << gyro_bias_ << std::endl;
  }


  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

EkfLooseIntegrationAlgNode::~EkfLooseIntegrationAlgNode(void)
{
  // [free dynamic memory]
}

void EkfLooseIntegrationAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  this->estimated_odom_.pose.pose.position.x = acc_corrected_(0);//rover_state_(0);
  this->estimated_odom_.pose.pose.position.y = acc_corrected_(1);//rover_state_(1);
  this->estimated_odom_.pose.pose.position.z = acc_corrected_(2);//rover_state_(2);
  
  // Converting to quarternion to fill the ROS message
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(rover_state_(6),
                                                          rover_state_(7),
                                                          rover_state_(8));

  this->estimated_odom_.pose.pose.orientation.x = quaternion[0];
  this->estimated_odom_.pose.pose.orientation.y = quaternion[1];
  this->estimated_odom_.pose.pose.orientation.z = quaternion[2];
  this->estimated_odom_.pose.pose.orientation.w = quaternion[3];

  this->estimated_odom_.twist.twist.linear.x = rover_state_(3);
  this->estimated_odom_.twist.twist.linear.y = rover_state_(4);
  this->estimated_odom_.twist.twist.linear.z = rover_state_(5);


  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->odom_publisher_.publish(this->estimated_odom_);
}

/*  [subscriber callbacks] */
void EkfLooseIntegrationAlgNode::cb_ackermannState(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr& estimated_ackermann_state_msg)
{
  this->alg_.lock();

  this->estimated_ackermann_state_.drive.speed = estimated_ackermann_state_msg->drive.speed;
  this->estimated_ackermann_state_.drive.steering_angle = estimated_ackermann_state_msg->drive.steering_angle;

  this->alg_.unlock();
}

void EkfLooseIntegrationAlgNode::cb_imuData(const sensor_msgs::Imu::ConstPtr& IMU_msg)
{
  this->alg_.lock();

  this->estimated_odom_.header = IMU_msg->header;
  //this->estimated_odom_.child_frame_id = IMU_msg->child_frame_id;

  //std::cout << "IMU message received!" << std::endl;

  current_timestamp_ = IMU_msg->header.stamp.sec + (IMU_msg->header.stamp.nsec * 1e-9);

  if (flag_imu_initialized_)
  {
    double previous_x = rover_state_(0);
    double previous_y = rover_state_(1);
    double previous_z = rover_state_(2);

    double previous_vx = rover_state_(3);
    double previous_vy = rover_state_(4);
    double previous_vz = rover_state_(5);

    double previous_acc_x = acc_corrected_(0);
    double previous_acc_y = acc_corrected_(1);
    double previous_acc_z = acc_corrected_(2);

    double dt = current_timestamp_ - previous_timestamp_;
    //std::cout << "Delta t = " << dt << std::endl;

    double delta_vx = previous_acc_x * dt;
    double delta_vy = previous_acc_y * dt;
    double delta_vz = previous_acc_z * dt;

    double current_vx = previous_vx + delta_vx;
    double current_vy = previous_vy + delta_vy;
    double current_vz = previous_vz + delta_vz;

    double dt_squared = dt * dt;
    double current_x = previous_x + previous_vx * dt + 0.5 * previous_acc_x * dt_squared;
    double current_y = previous_y + previous_vy * dt + 0.5 * previous_acc_y * dt_squared;
    double current_z = previous_z + previous_vz * dt + 0.5 * previous_acc_z * dt_squared;

    double previous_roll  = rover_state_(6);
    double previous_pitch = rover_state_(7);
    double previous_yaw   = rover_state_(8);

    double previous_roll_rate  = gyro_corrected_(0);
    double previous_pitch_rate = gyro_corrected_(1);
    double previous_yaw_rate   = gyro_corrected_(2);

    double delta_roll  = previous_roll_rate  * dt;
    double delta_pitch = previous_pitch_rate * dt;
    double delta_yaw   = previous_yaw_rate   * dt;

    double current_roll  = previous_roll  + delta_roll;
    double current_pitch = previous_pitch + delta_pitch;
    double current_yaw   = previous_yaw   + delta_yaw;

    // Update state
    rover_state_(0) = current_x;
    rover_state_(1) = current_y;
    rover_state_(2) = current_z;

    rover_state_(3) = current_vx;
    rover_state_(4) = current_vy;
    rover_state_(5) = current_vz;

    rover_state_(6) = current_roll;
    rover_state_(7) = current_pitch;
    rover_state_(8) = current_yaw;

  }

  acc_reading_(0) = IMU_msg->linear_acceleration.x;
  acc_reading_(1) = IMU_msg->linear_acceleration.y;
  acc_reading_(2) = IMU_msg->linear_acceleration.z;

  acc_corrected_ = acc_misaligment_ * acc_scale_factor_ * (acc_reading_ - acc_bias_);

  // Correcting gravity vector
  //acc_corrected_(2) += 9.8; // to cancel out the gravity vector

  double x_gravity_component = G_ * sin(rover_state_(7)); //pitch
  double y_gravity_component = G_ * sin(rover_state_(6)); //roll
  double z_gravity_component = sqrt( G_* G_  - x_gravity_component * x_gravity_component - y_gravity_component * y_gravity_component);

  acc_corrected_(0) += x_gravity_component; //correcting pitch component
  acc_corrected_(1) += y_gravity_component; //correcting roll component
  acc_corrected_(2) += z_gravity_component;

  gyro_reading_(0) = IMU_msg->angular_velocity.x;
  gyro_reading_(1) = IMU_msg->angular_velocity.y;
  gyro_reading_(2) = IMU_msg->angular_velocity.z;

  gyro_corrected_ = gyro_misaligment_ * gyro_scale_factor_ * (gyro_reading_ - gyro_bias_);

  previous_timestamp_ = current_timestamp_;

  if(!flag_imu_initialized_)
  {
    number_of_imu_readings_++;
    accumlator_acc_ += acc_misaligment_ * acc_scale_factor_ * (acc_reading_ - acc_bias_);

    if(number_of_imu_readings_ >= number_of_imu_readings_for_initialization_ && !flag_imu_initialized_)
    {
      mean_init_acc_(0) = accumlator_acc_(0) / (float) number_of_imu_readings_;
      mean_init_acc_(1) = accumlator_acc_(1) / (float) number_of_imu_readings_;
      mean_init_acc_(2) = accumlator_acc_(2) / (float) number_of_imu_readings_;

      std::cout << "Mean accelerations during initialization period = " << std::endl << mean_init_acc_ << std::endl;

      double initial_roll  = atan2(mean_init_acc_(1), mean_init_acc_(2));
      std::cout << "initial_roll = " << initial_roll << std::endl;
      if ( initial_roll > 0.0 )
      {
        initial_roll -= M_PI;
      }
      else
      {
        initial_roll += M_PI;
      }

      double initial_pitch = atan2(mean_init_acc_(0), mean_init_acc_(2));
      std::cout << "initial_pitch = " << initial_pitch << std::endl;
      if ( initial_pitch > 0.0 )
      {
        initial_pitch -= M_PI;
      }
      else
      {
        initial_pitch += M_PI;
      }

      double initial_yaw   = 0.0; // yaw is not observable through the gravity vector

      std::cout << "Initial roll angle in degrees = " << initial_roll * 180.0 / M_PI << std::endl
          << "Initial pitch angle in degrees = " << initial_pitch * 180.0 / M_PI << std::endl;

      rover_state_(6) = initial_roll;
      rover_state_(7) = initial_pitch;
      rover_state_(8) = initial_yaw;

      flag_imu_initialized_ = true;
    }

  }

  this->alg_.unlock();
}

void EkfLooseIntegrationAlgNode::cb_AMCLPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void EkfLooseIntegrationAlgNode::cb_GNSSOdom(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  this->alg_.lock();

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void EkfLooseIntegrationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void EkfLooseIntegrationAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<EkfLooseIntegrationAlgNode>(argc, argv, "ekf_loose_integration_alg_node");
}
