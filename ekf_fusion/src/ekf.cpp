#include "ekf.h"

CEkf::CEkf(ekf::KalmanConfiguration kalman_configuration, float wheelbase)
{
  config_ = kalman_configuration;
  wheelbase_ = wheelbase;

  flag_ekf_initialised_ = false;
  debug_ = false;

  //State vector
  X_(0, 0) = 0.0;
  X_(1, 0) = 0.0;
  X_(2, 0) = 0.0;
  X_(3, 0) = 0.0;
  X_(4, 0) = 0.0;

  // State covariance matrix
  P_(0, 0) = pow(config_.x_ini, 2); // initial value for x variance;
  P_(0, 1) = 0.0;
  P_(0, 2) = 0.0;
  P_(0, 3) = 0.0;
  P_(0, 4) = 0.0;

  P_(1, 0) = 0.0;
  P_(1, 1) = pow(config_.y_ini, 2); // y variance
  P_(1, 2) = 0.0;
  P_(1, 3) = 0.0;
  P_(1, 4) = 0.0;

  P_(2, 0) = 0.0;
  P_(2, 1) = 0.0;
  P_(2, 2) = pow(config_.theta_ini, 2); // orientation variance
  P_(2, 3) = 0.0;
  P_(2, 4) = 0.0;

  P_(3, 0) = 0.0;
  P_(3, 1) = 0.0;
  P_(3, 2) = 0.0;
  P_(3, 3) = pow(config_.v_ini, 2); // velocity variance
  P_(3, 4) = 0.0;

  P_(4, 0) = 0.0;
  P_(4, 1) = 0.0;
  P_(4, 2) = 0.0;
  P_(4, 3) = 0.0;
  P_(4, 4) = pow(config_.steering_ini, 2); // steering variance

  // Model noise Jacobian matrix
  F_q_(0, 0) = 0.0;
  F_q_(0, 1) = 0.0;

  F_q_(1, 0) = 0.0;
  F_q_(1, 1) = 0.0;

  F_q_(2, 0) = 0.0;
  F_q_(2, 1) = 0.0;

  F_q_(3, 0) = 1.0;
  F_q_(3, 1) = 0.0;

  F_q_(4, 0) = 0.0;
  F_q_(4, 1) = 1.0;

  // Model noise covariance matrix
  Q_(0, 0) = pow(config_.v_model, 2.0); //velocity noise variance
  Q_(0, 1) = 0.0;

  Q_(1, 0) = 0.0;
  Q_(1, 1) = pow(config_.steering_model, 2.0); //steering  noise variance
}

CEkf::~CEkf(void)
{

}

void CEkf::calculateStateJacobian(float dt)
{
  //f(x)
  F_X_(0, 0) = 1.0;
  F_X_(0, 1) = 0.0;
  F_X_(0, 2) = -X_(3, 0) * sin(X_(2, 0)) * dt;
  F_X_(0, 3) = cos(X_(2, 0)) * dt;
  F_X_(0, 4) = 0.0;

  //f(y)
  F_X_(1, 0) = 0.0;
  F_X_(1, 1) = 1.0;
  F_X_(1, 2) = X_(3, 0) * cos(X_(2, 0)) * dt;
  F_X_(1, 3) = sin(X_(2, 0)) * dt;
  F_X_(1, 4) = 0.0;

  //f(theta)
  F_X_(2, 0) = 0.0;
  F_X_(2, 1) = 0.0;
  F_X_(2, 2) = 1.0;
  F_X_(2, 3) = (tan(X_(4)) / wheelbase_) * dt;
  assert(fabs(X_(4) - M_PI / 2.0) > 0.01 && "ERROR in CEkf::calculateStateJacobian, steering == pi/2.0");
  F_X_(2, 4) = (X_(3) * dt) / (wheelbase_ * cos(X_(4)) * cos(X_(4)));

  //f(v)
  F_X_(3, 0) = 0.0;
  F_X_(3, 1) = 0.0;
  F_X_(3, 2) = 0.0;
  F_X_(3, 3) = 1.0;
  F_X_(3, 4) = 0.0;

  //f(alpha)
  F_X_(4, 0) = 0.0;
  F_X_(4, 1) = 0.0;
  F_X_(4, 2) = 0.0;
  F_X_(4, 3) = 0.0;
  F_X_(4, 4) = 1.0;

  if (debug_)
  {
    std::cout << "CEkf::calculateStateJacobian --> F_X_ = " << std::endl << F_X_ << std::endl;
    std::cout << "Delta t = " << dt << std::endl;
  }
}

void CEkf::predict(void)
{
  if (flag_ekf_initialised_)
  {
    float t_now = (float)ros::Time::now().toSec();
    dt_ = t_now - t_last_;

    // State prediction
    // x coordinate
    X_(0) = X_(0) + X_(3) * dt_ * cos(X_(2));

    // y coordinate
    X_(1) = X_(1) + X_(3) * dt_ * sin(X_(2));

    //theta (robot orientation)
    assert(fabs(X_(4) - M_PI / 2.0) > 0.01 && "ERROR in CEkf::predict, steering == pi/2.0");
    X_(2) = X_(2) + (tan(X_(4)) / wheelbase_) * X_(3) * dt_;

    //velocity (assumed constant plus zero mean perturbation noise) X_(3) = X_(3) + 0.0;
    //steering (assumed constant plus zero mean perturbation noise) X_(4) = X_(4) + 0.0;

    // Covariance prediction
    calculateStateJacobian(dt_); // update the F_X_
    P_ = F_X_ * P_ * F_X_.transpose() + F_q_ * Q_ * F_q_.transpose();

    // Preparing for the next filter iteration
    t_last_ = (float)ros::Time::now().toSec();

  }

}

float CEkf::update(ekf::ClearObservation obs)
{
  const float INVALID_DISTANCE = -1.0;
  float mahalanobis_distance = INVALID_DISTANCE;
  float likelihood = INVALID_DISTANCE;

  if (flag_ekf_initialised_)
  {
    float t_now = (float)ros::Time::now().toSec();
    dt_ = t_now - t_last_;

    //Filling the observation vector
    Eigen::Matrix<float, 2, 1> y;
    y(0) = obs.v;
    y(1) = obs.steering;

    calculateStateJacobian(dt_);

    if (debug_)
    {
      std::cout << "CEkf::Update X: " << X_.transpose() << std::endl;
      std::cout << "CEkf::Update P_: " << P_ << std::endl;
      std::cout << "CEkf::Update Q_: " << Q_ << std::endl;
      std::cout << "CEkf::Update F_X_: " << F_X_ << std::endl;
      std::cout << "CEkf::Update F_q_: " << F_q_ << std::endl;
    }

    // Expectation
    Eigen::Vector2f e, z; // expectation, innovation
    e(0) = X_(3);
    e(1) = X_(4);

    Eigen::Matrix<float, 2, 5> H = Eigen::Matrix<float, 2, 5>::Zero();
    // Jacobian of the observation function
    H.block<2, 3>(0, 0).setZero();
    H.block<2, 2>(0, 3).setIdentity();

    if (debug_)
      std::cout << "CEkf::Update H: " << H << std::endl;

    // Innovation
    z = y - e;

    // Innovation covariance

    Eigen::Matrix<float, 2, 2> R = Eigen::Matrix<float, 2, 2>::Zero();

    R(0, 0) = obs.sigma_v;
    R(1, 1) = obs.sigma_steering;

    Eigen::Matrix<float, 2, 2> Z = Eigen::Matrix<float, 2, 2>::Zero();

    Z = H * P_ * H.transpose() + R;

    if (debug_)
      std::cout << "CEkf::Update R: " << R << std::endl;

    mahalanobis_distance = sqrt(z.transpose() * Z.inverse() * z);

    likelihood = exp(-0.5 * mahalanobis_distance) / sqrt(Z.determinant());

    if (debug_)
      std::cout << "CEkf::Update mahalanobis_distance: " << mahalanobis_distance << std::endl;

    if (mahalanobis_distance < config_.outlier_mahalanobis_threshold)
    {
      // Kalman gain
      Eigen::Matrix<float, 5, 2> K = Eigen::Matrix<float, 5, 2>::Zero();
      K = P_ * H.transpose() * Z.inverse();

      if (debug_)
        std::cout << "CEkf::Update K: " << K << std::endl;

      // State correction
      X_ = X_ + K * z;

      if (debug_)
        std::cout << "CEkf::Update X_: " << X_ << std::endl;

      // State covariance correction
      P_ = P_ - K * Z * K.transpose();

      if (debug_)
        std::cout << "CEkf::Update P_: " << P_ << std::endl;
    }

    // Preparing for the next filter iteration
    t_last_ = (float)ros::Time::now().toSec();
  }
  return (likelihood);
}

float CEkf::update(ekf::GnssObservation obs)
{
  const float INVALID_DISTANCE = -1.0;
  float mahalanobis_distance = INVALID_DISTANCE;
  float likelihood = INVALID_DISTANCE;

  if (flag_ekf_initialised_)
  {
    float t_now = (float)ros::Time::now().toSec();
    dt_ = t_now - t_last_;

    //Filling the observation vector
    Eigen::Matrix<float, 2, 1> y;
    y(0) = obs.x;
    y(1) = obs.y;

    calculateStateJacobian(dt_);

    if (debug_)
    {
      std::cout << "CEkf::Update X_: " << X_.transpose() << std::endl;
      std::cout << "CEkf::Update P_: " << P_ << std::endl;
      std::cout << "CEkf::Update Q_: " << Q_ << std::endl;
      std::cout << "CEkf::Update F_X_: " << F_X_ << std::endl;
      std::cout << "CEkf::Update F_q_: " << F_q_ << std::endl;
    }

    // Expectation
    Eigen::Vector2f e, z; // expectation, innovation
    e(0) = X_(0);
    e(1) = X_(1);

    Eigen::Matrix<float, 2, 5> H = Eigen::Matrix<float, 2, 5>::Zero();
    // Jacobian of the observation function
    H.block<2, 2>(0, 0).setIdentity();
    H.block<2, 3>(0, 2).setZero();

    if (debug_)
      std::cout << "CEkf::Update H: " << H << std::endl;

    // Innovation
    z = y - e;

    // Innovation covariance

    Eigen::Matrix<float, 2, 2> R = Eigen::Matrix<float, 2, 2>::Zero();

    R(0, 0) = obs.sigma_x;
    R(1, 1) = obs.sigma_y;

    Eigen::Matrix<float, 2, 2> Z = Eigen::Matrix<float, 2, 2>::Zero();

    Z = H * P_ * H.transpose() + R;

    if (debug_)
      std::cout << "CEkf::Update R: " << R << std::endl;

    mahalanobis_distance = sqrt(z.transpose() * Z.inverse() * z);

    likelihood = exp(-0.5 * mahalanobis_distance) / sqrt(Z.determinant());

    if (debug_)
      std::cout << "CEkf::Update mahalanobis_distance: " << mahalanobis_distance << std::endl;

    if (mahalanobis_distance < config_.outlier_mahalanobis_threshold)
    {
      // Kalman gain
      Eigen::Matrix<float, 5, 2> K = Eigen::Matrix<float, 5, 2>::Zero();
      K = P_ * H.transpose() * Z.inverse();

      if (debug_)
        std::cout << "CEkf::Update K: " << K << std::endl;

      // State correction
      X_ = X_ + K * z;

      if (debug_)
        std::cout << "CEkf::Update X_: " << X_ << std::endl;

      // State covariance correction
      P_ = P_ - K * Z * K.transpose();

      if (debug_)
        std::cout << "CEkf::Update P_: " << P_ << std::endl;
    }

    // Preparing for the next filter iteration
    t_last_ = (float)ros::Time::now().toSec();
  }
  return (likelihood);
}

float CEkf::update(ekf::ImuObservation obs)
{
  const float INVALID_DISTANCE = -1.0;
  float mahalanobis_distance = INVALID_DISTANCE;
  float likelihood = INVALID_DISTANCE;

  if (flag_ekf_initialised_)
  {
    float t_now = (float)ros::Time::now().toSec();
    dt_ = t_now - t_last_;

    //Filling the observation vector
    float y;
    y = obs.theta;

    calculateStateJacobian(dt_);

    if (debug_)
    {
      std::cout << "CEkf::Update X_: " << X_.transpose() << std::endl;
      std::cout << "CEkf::Update P_: " << P_ << std::endl;
      std::cout << "CEkf::Update Q_: " << Q_ << std::endl;
      std::cout << "CEkf::Update F_X_: " << F_X_ << std::endl;
      std::cout << "CEkf::Update F_q_: " << F_q_ << std::endl;
    }

    // Expectation
    float e, z; // expectation, innovation
    e = X_(2);

    Eigen::Matrix<float, 1, 5> H = Eigen::Matrix<float, 1, 5>::Zero();
    // Jacobian of the observation function
    H(0, 2) = 1;

    if (debug_)
      std::cout << "CEkf::Update H: " << H << std::endl;

    // Innovation
    z = y - e;

    // Innovation covariance

    float R;

    R = obs.sigma_theta;

    float Z;

    Z = H * P_ * H.transpose() + R;

    if (debug_)
      std::cout << "CEkf::Update R: " << R << std::endl;

    mahalanobis_distance = sqrt(z * z / Z);

    likelihood = exp(-0.5 * mahalanobis_distance) / sqrt(Z);

    if (debug_)
      std::cout << "CEkf::Update mahalanobis_distance: " << mahalanobis_distance << std::endl;

    if (mahalanobis_distance < config_.outlier_mahalanobis_threshold)
    {
      // Kalman gain
      Eigen::Matrix<float, 5, 1> K = Eigen::Matrix<float, 5, 1>::Zero();
      K = P_ * H.transpose() / Z;

      if (debug_)
        std::cout << "CEkf::Update K: " << K << std::endl;

      // State correction
      X_ = X_ + K * z;

      if (debug_)
        std::cout << "CEkf::Update X_: " << X_ << std::endl;

      // State covariance correction
      P_ = P_ - K * Z * K.transpose();

      if (debug_)
        std::cout << "CEkf::Update P_: " << P_ << std::endl;
    }

    // Preparing for the next filter iteration
    t_last_ = (float)ros::Time::now().toSec();
  }
  return (likelihood);
}

float CEkf::update(ekf::SlamObservation obs)
{
  const float INVALID_DISTANCE = -1.0;
  float mahalanobis_distance = INVALID_DISTANCE;
  float likelihood = INVALID_DISTANCE;

  if (!flag_ekf_initialised_)
  {
    calculateStateJacobian(0.0); // just to initialise
    X_(0) = obs.x;
    X_(1) = obs.y;
    X_(2) = obs.theta;
    flag_ekf_initialised_ = true;
  }
  else
  {
    //Filling the observation vector
    Eigen::Matrix<float, 3, 1> y;
    y(0) = obs.x;
    y(1) = obs.y;
    y(2) = obs.theta;

    float t_now = (float)ros::Time::now().toSec();
    dt_ = t_now - t_last_;

    calculateStateJacobian(dt_);

    if (debug_)
    {
      std::cout << "CEkf::Update X_: " << X_.transpose() << std::endl;
      std::cout << "CEkf::Update P_: " << P_ << std::endl;
      std::cout << "CEkf::Update Q_: " << Q_ << std::endl;
      std::cout << "CEkf::Update F_X_: " << F_X_ << std::endl;
      std::cout << "CEkf::Update F_q_: " << F_q_ << std::endl;
    }

    // Expectation
    Eigen::Vector3f e, z; // expectation, innovation
    e(0) = X_(0);
    e(1) = X_(1);
    e(2) = X_(2);

    Eigen::Matrix<float, 3, 5> H = Eigen::Matrix<float, 3, 5>::Zero();
    // Jacobian of the observation function
    H.block<3, 3>(0, 0).setIdentity();
    H.block<3, 2>(0, 3).setZero();

    if (debug_)
      std::cout << "CEkf::Update H: " << H << std::endl;

    // Innovation
    z = y - e;

    // Innovation covariance
    Eigen::Matrix<float, 3, 3> R = Eigen::Matrix<float, 3, 3>::Zero();

    R(0, 0) = obs.sigma_x;
    R(1, 1) = obs.sigma_y;
    R(2, 2) = obs.sigma_theta;

    Eigen::Matrix<float, 3, 3> Z = Eigen::Matrix<float, 3, 3>::Zero();

    Z = H * P_ * H.transpose() + R;

    if (debug_)
      std::cout << "CEkf::Update R: " << R << std::endl;

    mahalanobis_distance = sqrt(z.transpose() * Z.inverse() * z);

    likelihood = exp(-0.5 * mahalanobis_distance) / sqrt(Z.determinant());

    if (debug_)
      std::cout << "CEkf::Update mahalanobis_distance: " << mahalanobis_distance << std::endl;

    if (mahalanobis_distance < config_.outlier_mahalanobis_threshold)
    {
      // Kalman gain
      Eigen::Matrix<float, 5, 3> K = Eigen::Matrix<float, 5, 3>::Zero();
      K = P_ * H.transpose() * Z.inverse();

      if (debug_)
        std::cout << "CEkf::Update K: " << K << std::endl;

      // State correction
      X_ = X_ + K * z;

      if (debug_)
        std::cout << "CEkf::Update X_: " << X_ << std::endl;

      // State covariance correction
      P_ = P_ - K * Z * K.transpose();

      if (debug_)
        std::cout << "CEkf::Update P_: " << P_ << std::endl;
    }
  }
  // Preparing for the next filter iteration
  float t_last = (float)ros::Time::now().toSec();
  return (likelihood);
}

void CEkf::getStateAndCovariance(Eigen::Matrix<float, 5, 1>& state, Eigen::Matrix<float, 5, 5>& covariance)
{
  state = X_;
  covariance = P_;
}
