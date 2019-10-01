#include "ekf_loose_integration_alg_node.h"

EkfLooseIntegrationAlgNode::EkfLooseIntegrationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<EkfLooseIntegrationAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

EkfLooseIntegrationAlgNode::~EkfLooseIntegrationAlgNode(void)
{
  // [free dynamic memory]
}

void EkfLooseIntegrationAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void EkfLooseIntegrationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void EkfLooseIntegrationAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<EkfLooseIntegrationAlgNode>(argc, argv, "ekf_loose_integration_alg_node");
}
