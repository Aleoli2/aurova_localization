#ifndef _ekf_h_
#define _ekf_h_

#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "math.h"

namespace ekf
{
struct ClearObservation
{
  double v,steering;
  double sigma_v, sigma_steering;
};

struct GnssObservation
{
  double x,y;
  double sigma_x, sigma_y;
};

struct ImuObservation
{
  double theta;
  double sigma_theta;
};

struct OdomObservation
{
  double x,y,theta;
  double sigma_x, sigma_y, sigma_theta;
};

struct SlamObservation
{
  double x,y,theta;
  double sigma_x, sigma_y, sigma_theta;
};

struct KalmanConfiguration
{
  double x_ini, y_ini, theta_ini, v_ini, steering_ini;
  double v_model, steering_model;
  double outlier_mahalanobis_threshold;
};
}

class CEkf;
typedef CEkf* CEkfPtr;

class CEkf
{
private:
  Eigen::Matrix<double, 5, 1> X_;
  Eigen::Matrix<double, 5, 5> F_X_;
  Eigen::Matrix<double, 5, 2> F_q_;
  Eigen::Matrix<double, 2, 2> Q_;
  Eigen::Matrix<double, 5, 5> P_;

  ekf::KalmanConfiguration config_;

  double wheelbase_;

  bool flag_ekf_initialised_;
  bool debug_;
  bool debug2_;

  void calculateStateJacobian(double dt);

public:

  CEkf(ekf::KalmanConfiguration kalman_configuration, double wheelbase);

  ~CEkf(void);

  void predict(void);

  double update(ekf::ClearObservation obs);

  double update(ekf::GnssObservation obs);

  double update(ekf::ImuObservation obs);

  double update(ekf::SlamObservation obs);

  void getStateAndCovariance(Eigen::Matrix<double, 5, 1>& state, Eigen::Matrix<double, 5, 5>& covariance);

  void setDebug(bool debug)
  {
    debug_ = debug;
  }
};

#endif
