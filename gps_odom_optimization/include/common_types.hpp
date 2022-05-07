#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H
#pragma once

#include <Eigen/Dense>

/**
 * @brief OdometryConstraint: The Constraint for odometry in the pose graph
 */
struct OdometryConstraint {
	size_t id_begin, id_end;

    // transformation from begin pose to end pose
	Eigen::Vector3d tf_p;
    Eigen::Quaterniond tf_q;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Covariance and information matrix (inverse of covariance)
    Eigen::Matrix<double, 6, 6> covariance;
    Eigen::Matrix<double, 6, 6> information;
};

/**
 * @brief PriorConstraint: The Constraint for GPS pose in the graph
 */
struct PriorConstraint {
	size_t id;

	Eigen::Vector3d p;
    Eigen::Quaterniond q;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<double, 6, 6> covariance;
    Eigen::Matrix<double, 6, 6> information;
};

using Pose3dWithCovariance = PriorConstraint;
using PriorConstraintVector = std::vector<PriorConstraint>;
using OdometryConstraintsVector = std::vector<OdometryConstraint>;
using Trajectory = std::vector<Pose3dWithCovariance>;
using Tf = Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>;

#endif // COMMON_TYPES_H
