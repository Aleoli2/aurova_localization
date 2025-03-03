### Example of usage:

You can run an example following the instructions in [applications](https://github.com/AUROVA-LAB/applications) (Examples).

# aurova_localization
Metapackage for localization nodes in AUROVA group. Each node contain a different localization algorithim. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "name_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**ekf_fusion**
This package contains a node that, as input, reads the topics /odometry_gps and /odom, of type nav_msgs::Odometry. This node fuses this sources using a Kalman Filter. The topic /initialpose of type geometry_msgs::PoseWithCovarianceStamped serves as pose input to use as manual relocation. The node output is published in the topic /pose_plot (that is the final output of our fusion system) of type geometry_msgs::PoseWithCovariance.

Parameters:
* ~ekf_fusion/frame_id (default: ""): Main coordinates frame for vehicle localization (usually "map").
* ~ekf_fusion/child_id (default: ""): Coordinates frame for odometry (usually "odom").
* ~ekf_fusion/x_model (default: null): Variance for propagation model in the x axis.
* ~ekf_fusion/y_model (default: null): Variance for propagation model in the y axis.
* ~ekf_fusion/theta_model (default: null): Variance for propagation model in yaw component.
* ~ekf_fusion/outlier_mahalanobis (default: null): The threshold to discard observetions (usually 3-5).
* ~ekf_fusion/min_speed (default: null): Under this speed, the filter does not take into account the orientation in /odometry_gps.
* ~ekf_fusion/is_simulation (default: false): It shoul be true for gazebo simulation case, and false for real or .bag file case.

**get_pose_from_tf**
This package contains a node that, as input, reads the /tf messages. This node calculates the transformation between two differents frames to obtain a odometry message. The node output is published in the topic /odometry_filtered of type nav_msgs::Odometry.

Parameters:
* ~frame_id_tf (default: ""): Parent frame name to transform.
* ~child_id_tf (default: ""): Child frame name to transform.

**pose_simulation**
This package contains a node that, as input, reads the topic /desired_ackermann_state of type ackermann_msgs::AckermannDrive and generates a simulated movement into virtual space integrating the Ackermann inputs. As output generates a topic /pose_sim of type geometry_msgs::PoseWithCovarianceStamped.

Parameters:
* ~pose_simulation/frame_id (default: ""): Coordinates frame for vehicle localization (usually "map" or "odom").
* ~pose_simulation/child_id (default: ""): Coordinates frame for robot base (usually "base_link").
* ~pose_simulation/d_vehicle (default: null): Distance between the rear and front axles of the vehicle.
* ~pose_simulation/init_x (default: null): Initial x axis value. 
* ~pose_simulation/init_y (default: null): Initial y axis value.
* ~pose_simulation/init_z (default: null): Initial z axis value.
* ~pose_simulation/init_w (default: null): Initial yaw value.
* ~pose_simulation/var_x (default: null): Variance in x axis.
* ~pose_simulation/var_y (default: null): Variance in y axis.
* ~pose_simulation/var_z (default: null): Variance in z axis.
* ~pose_simulation/var_w (default: null): Variance in yaw component. 

**gps_odom_optimization**
This package contains a node that, as input, reads the topics /odometry_gps and /odom, of type nav_msgs::Odometry. This node fuses this sources using a Gauss-Newton (GN) non-linear least squares. The node output is published in the topic /localization (that is the final output of our fusion system) of type nav_msgs::Odometry.

Parameters:
* ~gps_odom_optimization/frame_id (default: ""): Main coordinates frame for vehicle localization (usually "map").
* ~gps_odom_optimization/child_id (default: ""): Coordinates frame for odometry (usually "odom").
