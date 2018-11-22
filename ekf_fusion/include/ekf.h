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
  float v,steering;
  float sigma_v, sigma_steering;
};

struct GnssObservation
{
  float x,y;
  float sigma_x, sigma_y;
};

struct ImuObservation
{
  float theta;
  float sigma_theta;
};

struct OdomObservation
{
  float x,y,theta;
  float sigma_x, sigma_y, sigma_theta;
};

struct SlamObservation
{
  float x,y,theta;
  float sigma_x, sigma_y, sigma_theta;
};

struct KalmanConfiguration
{
  float x_ini, y_ini, theta_ini, v_ini, steering_ini;
  float v_model, steering_model;
  float outlier_mahalanobis_threshold;
};
}


class CEkf
{
private:
  Eigen::Matrix<float, 5, 1> X_;
  Eigen::Matrix<float, 5, 5> F_X_;
  Eigen::Matrix<float, 5, 2> F_q_;
  Eigen::Matrix<float, 2, 2> Q_;
  Eigen::Matrix<float, 5, 5> P_;

  ekf::KalmanConfiguration config_;

  float t_last_;
  float dt_;
  float wheelbase_;

  bool flag_ekf_initialised_;
  bool debug_;

  void calculateStateJacobian(float dt);

public:

  CEkf(ekf::KalmanConfiguration kalman_configuration, float wheelbase);

  ~CEkf(void);

  void predict(void);

  float update(ekf::ClearObservation obs);

  float update(ekf::GnssObservation obs);

  float update(ekf::ImuObservation obs);

  float update(ekf::SlamObservation obs);

  void getStateAndCovariance(Eigen::Matrix<float, 5, 1>& state, Eigen::Matrix<float, 5, 5>& covariance);

  void setDebug(bool debug)
  {
    debug_ = debug;
  }
};

#endif
