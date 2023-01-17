// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _gps_odom_optimization_alg_node_h_
#define _gps_odom_optimization_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "gps_odom_optimization_alg.h"

#include <eigen_conversions/eigen_msg.h>

#include "optimization_process.hpp"

// [publisher subscriber headers]
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class GpsOdomOptimizationAlgNode : public algorithm_base::IriBaseAlgorithm<GpsOdomOptimizationAlgorithm>
{
  private:

	bool gps_received_;
	OptimizationProcessPtr optimization_;

  Eigen::Matrix<double,6,6> Covariance; 

    // [publisher attributes]
    tf::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::TransformStamped transform_msg_;

    ros::Publisher localization_publisher_;
    nav_msgs::Odometry localization_Odometry_msg_;
    nav_msgs::Odometry gps_odom_msg_;


    // [subscriber attributes]
    tf::TransformListener tf_listener_;

    ros::Subscriber odometry_gps_subscriber_;
    void odometry_gps_callback(const nav_msgs::Odometry::ConstPtr& msg);
    pthread_mutex_t odometry_gps_mutex_;
    void odometry_gps_mutex_enter(void);
    void odometry_gps_mutex_exit(void);

    ros::Subscriber odom_subscriber_;
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    pthread_mutex_t odom_mutex_;
    void odom_mutex_enter(void);
    void odom_mutex_exit(void);


    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;
    std::string ns;
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    GpsOdomOptimizationAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~GpsOdomOptimizationAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
