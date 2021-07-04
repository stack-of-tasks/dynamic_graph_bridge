#include "impl_test_sot_external_interface.hh"

ImplTestSotExternalInterface::ImplTestSotExternalInterface()
{}

ImplTestSotExternalInterface::~ImplTestSotExternalInterface()
{}

void ImplTestSotExternalInterface::setupSetSensors
(std::map<std::string,
 dynamicgraph::sot::SensorValues> &)
{
  std::cout << "ImplTestSotExternalInterface::setupSetSensors" << std::endl;
}

void ImplTestSotExternalInterface::nominalSetSensors
(std::map<std::string,
 dynamicgraph::sot::SensorValues> &)
{
  std::cout << "ImplTestSotExternalInterface::nominalSetSensors" << std::endl;
}

void ImplTestSotExternalInterface::cleanupSetSensors
(std::map<std::string,
 dynamicgraph::sot::SensorValues> &)
{
  std::cout << "ImplTestSotExternalInterface::cleanupSetSensors" << std::endl;
}

void ImplTestSotExternalInterface::setSecondOrderIntegration
(void)
{
  std::cout << "ImplTestSotExternalInterface::setSecondOrderIntegration" <<  std::endl;
}

void ImplTestSotExternalInterface::setNoIntegration
(void)
{
  std::cout << "setNoIntegration" << std::endl;
}
