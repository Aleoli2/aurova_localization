#include "ekf_loose_integration_alg.h"

EkfLooseIntegrationAlgorithm::EkfLooseIntegrationAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

EkfLooseIntegrationAlgorithm::~EkfLooseIntegrationAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void EkfLooseIntegrationAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// EkfLooseIntegrationAlgorithm Public API
