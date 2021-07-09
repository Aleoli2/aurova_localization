# aurova_localization
Metapackage for localization nodes in AUROVA group. Each node contain a different localization algorithim. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "name_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**ekf_fusion**
This package contains a node that, as input, reads the topics /odometry_gps and /odom, of type nav_msgs::Odometry. This node fuses this sources using a Kalman Filter. The topic /initialpose of type geometry_msgs::PoseWithCovarianceStamped serves as pose input to use as manual relocation. The node output is published in the topic /pose_plot (that is the final output of our fusion system) of type geometry_msgs::PoseWithCovariance.

Parameters:
* ~ekf_fusion/frame_id (default: ""): Main coordinates frame for vehicle localization (usually "map").
* ~ekf_fusion/child_id (default: ""): Main coordinates frame for odometry (usually "odom").
* ~ekf_fusion/x_model (default: null): Variance for propagation model in the x axis.
* ~ekf_fusion/y_model (default: null): Variance for propagation model in the y axis.
* ~ekf_fusion/theta_model (default: null): Variance for propagation model in yaw component.
* ~ekf_fusion/outlier_mahalanobis (default: null): The threshold to discard observetions (usually 3-5).
* ~ekf_fusion/min_speed (default: null): Under this speed, the filter does not take into account the orientation in /odometry_gps.
* ~ekf_fusion/is_simulation (default: false): It shoul be true for gazebo simulation case, and false for real or .bag file case.

**ekf_loose_integration (incomplete/deprecated)**  
