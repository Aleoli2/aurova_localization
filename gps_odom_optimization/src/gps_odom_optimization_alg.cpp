#include "gps_odom_optimization_alg.h"

GpsOdomOptimizationAlgorithm::GpsOdomOptimizationAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

GpsOdomOptimizationAlgorithm::~GpsOdomOptimizationAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void GpsOdomOptimizationAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// GpsOdomOptimizationAlgorithm Public API
