#ifndef _geo_localization_alg_node_h_
#define _geo_localization_alg_node_h_


#include <localization/data_processing.h>
#include <localization/optimization_process.h>
#include <localization/latlong_utm.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <fstream>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>


// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class GeoLocalizationAlgNode : public rclcpp::Node
{
  private:
  
    int seq_;
    double lat_zero_;
    double lon_zero_;
    float offset_map_x_;
    float offset_map_y_;
    int count_;
    int margin_asso_constraints_;
    int margin_gnss_constraints_;
    double asso_weight_;
    double odom_weight_;
    double asso_preweight_;
    float margin_gnss_distance_;
    bool flag_gps_corr_;
    bool save_data_;
    bool save_map_;
    bool ground_truth_;
    int gt_last_frame_;
    std::vector<double> gt_key_frames_;
    std::string out_data_;
    std::string out_map_;
    std::string out_gt_;
    std::string world_id_;
    std::string map_id_;
    std::string odom_id_;
    std::string base_id_;
    std::string lidar_id_;
    pcl::PointCloud<pcl::PointXYZ> last_detect_pcl_;
    data_processing::ConfigParams data_config_;
    data_processing::PolylineMap map_;
    data_processing::DataProcessing *data_;
    optimization_process::OptimizationProcess *optimization_;
    optimization_process::ConfigParams optimization_config_;
    geometry_msgs::msg::TransformStamped tf_to_utm_;
    geometry_msgs::msg::TransformStamped tf_to_map_;
		std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_{nullptr};
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    visualization_msgs::msg::MarkerArray marker_array_;
    
    // [publisher attributes]
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localization_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detection_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corregist_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gpscorrected_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wa_publisher_;
    nav_msgs::msg::Odometry localization_msg_;

    // [subscriber attributes]
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr detc_subscriber_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr  msg);
    void gnss_callback(const nav_msgs::msg::Odometry::SharedPtr  msg);
    void detc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

  public:
    std::shared_ptr<rclcpp::Rate> loop_rate;
    double rate;
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    GeoLocalizationAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~GeoLocalizationAlgNode(void);

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

  protected:

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    */
    rcl_interfaces::msg::SetParametersResult node_config_update(const std::vector<rclcpp::Parameter> & parameters);


    //// NEW LOCAL FUNCTIONS
    void fromUtmTransform(void);
    void mapToOdomInit(void);
    int parseMapToRosMarker(visualization_msgs::msg::MarkerArray& marker_array);
    void computeOptimizationProblem (void);
    void computeOptimizationProblemGT (void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif