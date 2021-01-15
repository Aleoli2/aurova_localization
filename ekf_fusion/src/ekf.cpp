#include "ekf.h"

CEkf::CEkf(ekf::KalmanConfiguration kalman_configuration)
{
  config_ = kalman_configuration;

  flag_ekf_initialised_ = false;
  debug_ = false;

  //State vector
  X_(0, 0) = 0.0;
  X_(1, 0) = 0.0;
  X_(2, 0) = 0.0;

  // State covariance matrix
  P_(0, 0) = pow(config_.x_ini, 2); // initial value for x variance;
  P_(0, 1) = 0.0;
  P_(0, 2) = 0.0;

  P_(1, 0) = 0.0;
  P_(1, 1) = pow(config_.y_ini, 2); // y variance
  P_(1, 2) = 0.0;

  P_(2, 0) = 0.0;
  P_(2, 1) = 0.0;
  P_(2, 2) = pow(config_.theta_ini, 2); // orientation variance

  // Model noise covariance matrix
  Q_(0, 0) = pow(config_.x_model, 2.0); //x noise variance
  Q_(0, 1) = 0.0;
  Q_(0, 2) = 0.0;

  Q_(1, 0) = 0.0;
  Q_(1, 1) = pow(config_.y_model, 2.0); //y  noise variance
  Q_(1, 2) = 0.0;

  Q_(2, 0) = 0.0;
  Q_(2, 1) = 0.0;
  Q_(2, 2) = pow(config_.theta_model, 2.0); //theta  noise variance

  // Identity matrix
  F_X_.setIdentity();
  F_q_.setIdentity();
  F_u_.setIdentity();
  H_.setIdentity();
}

CEkf::~CEkf(void)
{

}

void CEkf::predict(ekf::OdomAction act)
{
  static bool first_exec = true;
  static double t_last = 0.0;
  double dt = 0.0;

  if (flag_ekf_initialised_)
  {
    // State prediction
    Eigen::Matrix<double, 3, 1> u = Eigen::Matrix<double, 3, 1>::Zero();

    u(0) = act.delta_x;
    u(1) = act.delta_y;
    u(2) = act.delta_theta;
    X_(0) = X_(0) + u(0);
    X_(1) = X_(1) + u(1);
    X_(2) = X_(2) + u(2);

    //angle correction
    if (X_(2) > PI)
      X_(2) = X_(2) - 2 * PI;
    else if (X_(2) < -1 * PI)
      X_(2) = X_(2) + 2 * PI;

    // Covariance prediction
    P_ = F_X_ * P_ * F_X_.transpose() + F_q_ * Q_ * F_q_.transpose();
  }
}

double CEkf::update(ekf::GnssObservation obs)
{
  const double INVALID_DISTANCE = -1.0;
  double mahalanobis_distance = INVALID_DISTANCE;
  double likelihood = INVALID_DISTANCE;

  if (!flag_ekf_initialised_)
  {
    X_(0) = obs.x;
    X_(1) = obs.y;
    X_(2) = obs.theta;
    P_(0, 0) = obs.sigma_x * 100; // initial value for x variance;
    P_(1, 1) = obs.sigma_y * 100; // initial value for y variance
    P_(2, 2) = obs.sigma_theta * 100; // initial value for orientation variance
    flag_ekf_initialised_ = true;
  }
  else
  {
    //Filling the observation vector
    Eigen::Matrix<double, 3, 1> y;
    y(0) = obs.x;
    y(1) = obs.y;
    y(2) = obs.theta;

    // Expectation
    Eigen::Matrix<double, 3, 1> e, z; // expectation, innovation
    e(0) = X_(0);
    e(1) = X_(1);
    e(2) = X_(2);

    //for differential problems
    if (e(2) < -1 * PI / 2 && y(2) > PI / 2)
      e(2) = e(2) + 2 * PI;
    else if (y(2) < -1 * PI / 2 && e(2) > PI / 2)
      y(2) = y(2) + 2 * PI;

    // Innovation
    z = y - e;

    // Innovation covariance
    Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Zero();

    R(0, 0) = obs.sigma_x;
    R(1, 1) = obs.sigma_y;
    R(2, 2) = obs.sigma_theta;

    Eigen::Matrix<double, 3, 3> Z = Eigen::Matrix<double, 3, 3>::Zero();

    Z = H_ * P_ * H_.transpose() + R;

    if (debug_)
      std::cout << "CEkf::Update R: " << R << std::endl;

    mahalanobis_distance = sqrt(z.transpose() * Z.inverse() * z);

    likelihood = exp(-0.5 * mahalanobis_distance) / sqrt(Z.determinant());

    if (debug_)
      std::cout << "CEkf::Update mahalanobis_distance: " << mahalanobis_distance << std::endl;

    if (mahalanobis_distance < config_.outlier_mahalanobis_threshold)
    {
      // Kalman gain
      Eigen::Matrix<double, 3, 3> K = Eigen::Matrix<double, 3, 3>::Zero();
      K = P_ * H_.transpose() * Z.inverse();

      if (debug_)
        std::cout << "CEkf::Update K: " << K << std::endl;

      // State correction
      X_ = X_ + K * z;

      if (debug_)
        std::cout << "CEkf::Update X_: " << X_ << std::endl;

      // State covariance correction
      //P_ = P_ - K * Z * K.transpose();
      // State covariance correction (Joseph form)
      Eigen::Matrix<double, 3, 3> I = Eigen::Matrix<double, 3, 3>::Identity();
      P_ = (I - K * H_) * P_;

      if (debug_)
        std::cout << "CEkf::Update P_: " << P_ << std::endl;
    }
  }
  return (likelihood);
}

double CEkf::update(ekf::SlamObservation obs)
{
  const double INVALID_DISTANCE = -1.0;
  double mahalanobis_distance = INVALID_DISTANCE;
  double likelihood = INVALID_DISTANCE;

  if (!flag_ekf_initialised_)
  {
    X_(0) = obs.x;
    X_(1) = obs.y;
    X_(2) = obs.theta;
    P_(0, 0) = obs.sigma_x * 100; // initial value for x variance;
    P_(1, 1) = obs.sigma_y * 100; // initial value for y variance
    P_(2, 2) = obs.sigma_theta * 100; // initial value for orientation variance
    flag_ekf_initialised_ = true;
  }
  else
  {

    //Filling the observation vector
    Eigen::Matrix<double, 3, 1> y;
    y(0) = obs.x;
    y(1) = obs.y;
    y(2) = obs.theta;

    // Expectation
    Eigen::Matrix<double, 3, 1> e, z; // expectation, innovation
    e(0) = X_(0);
    e(1) = X_(1);
    e(2) = X_(2);

    //for differential problems
    if (e(2) < -1 * PI / 2 && y(2) > PI / 2)
      e(2) = e(2) + 2 * PI;
    else if (y(2) < -1 * PI / 2 && e(2) > PI / 2)
      y(2) = y(2) + 2 * PI;

    // Innovation
    z = y - e;

    // Innovation covariance
    Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Zero();

    R(0, 0) = obs.sigma_x;
    R(1, 1) = obs.sigma_y;
    R(2, 2) = obs.sigma_theta;

    Eigen::Matrix<double, 3, 3> Z = Eigen::Matrix<double, 3, 3>::Zero();

    Z = H_ * P_ * H_.transpose() + R;

    if (debug_)
      std::cout << "CEkf::Update R: " << R << std::endl;

    mahalanobis_distance = sqrt(z.transpose() * Z.inverse() * z);

    likelihood = exp(-0.5 * mahalanobis_distance) / sqrt(Z.determinant());

    if (debug_)
      std::cout << "CEkf::Update mahalanobis_distance: " << mahalanobis_distance << std::endl;

    if (mahalanobis_distance < config_.outlier_mahalanobis_threshold)
    {
      // Kalman gain
      Eigen::Matrix<double, 3, 3> K = Eigen::Matrix<double, 3, 3>::Zero();
      K = P_ * H_.transpose() * Z.inverse();

      if (debug_)
        std::cout << "CEkf::Update K: " << K << std::endl;

      // State correction
      X_ = X_ + K * z;

      if (debug_)
        std::cout << "CEkf::Update X_: " << X_ << std::endl;

      // State covariance correction
      //P_ = P_ - K * Z * K.transpose();
      // State covariance correction (Joseph form)
      Eigen::Matrix<double, 3, 3> I = Eigen::Matrix<double, 3, 3>::Identity();
      P_ = (I - K * H_) * P_;

      if (debug_)
        std::cout << "CEkf::Update P_: " << P_ << std::endl;
    }
  }
  return (likelihood);
}

void CEkf::getStateAndCovariance(Eigen::Matrix<double, 3, 1>& state, Eigen::Matrix<double, 3, 3>& covariance)
{
  state = X_;
  covariance = P_;
}

void CEkf::setStateAndCovariance(Eigen::Matrix<double, 3, 1> state, Eigen::Matrix<double, 3, 3> covariance)
{
  X_ = state;
  P_ = covariance;
}
