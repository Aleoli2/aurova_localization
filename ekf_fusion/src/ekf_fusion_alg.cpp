#include "ekf_fusion_alg.h"

EkfFusionAlgorithm::EkfFusionAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

EkfFusionAlgorithm::~EkfFusionAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void EkfFusionAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// EkfFusionAlgorithm Public API
