#ifndef _ekf_h_
#define _ekf_h_

#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "math.h"

#define PI 3.14159265358979323846

namespace ekf
{

struct GnssObservation
{
  double x, y, theta;
  double sigma_x, sigma_y, sigma_theta;
};

struct OdomAction
{
  double delta_x, delta_y, delta_theta;
  double sigma_x, sigma_y, sigma_theta;
};

struct SlamObservation
{
  double x,y,theta;
  double sigma_x, sigma_y, sigma_theta;
};

struct KalmanConfiguration
{
  double x_ini, y_ini, theta_ini;
  double x_model, y_model, theta_model;
  double outlier_mahalanobis_threshold;
};
}

class CEkf;
typedef CEkf* CEkfPtr;

class CEkf
{
private:
  Eigen::Matrix<double, 3, 1> X_;
  Eigen::Matrix<double, 3, 3> F_X_;
  Eigen::Matrix<double, 3, 3> F_u_;
  Eigen::Matrix<double, 3, 3> F_q_;
  Eigen::Matrix<double, 3, 3> Q_;
  Eigen::Matrix<double, 3, 3> P_;
  Eigen::Matrix<double, 3, 3> H_;

  ekf::KalmanConfiguration config_;

  double wheelbase_;

  bool debug_;

public:

  bool flag_ekf_initialised_;

  CEkf(ekf::KalmanConfiguration kalman_configuration);

  ~CEkf(void);

  void predict(ekf::OdomAction act);

  double update(ekf::GnssObservation obs);

  double update(ekf::SlamObservation obs);

  void getStateAndCovariance(Eigen::Matrix<double, 3, 1>& state, Eigen::Matrix<double, 3, 3>& covariance);

  void setDebug(bool debug)
  {
    debug_ = debug;
  }
};

#endif
