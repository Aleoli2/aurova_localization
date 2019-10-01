# aurova_localization
Metapackage for localization nodes in AUROVA group. Each node contain a different localization algorithim. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "name_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**ekf_fusion**
This package contains a node that, as input, reads the topics /amcl_pose, of type geometry_msgs::PoseWithCovariance, and /odometry_gps and /odometry, of type nav_msgs::Odometry. This node fuses this sources using a Kalman Filter. The node output is published in the topics /initialpose (used to replace the RVIZ-manual relocalization of the AMCL algorithm)
and /initialpose_plot (that is the final output of our fusion system) of type geometry_msgs::PoseWithCovariance.

**ekf_loose_integration**
This package will contain a node to fuse IMU, odometry, GNSS position and velocity and AMCL, estimating the biases, escale factors and the TF error between GNSS-UTM and map.  
