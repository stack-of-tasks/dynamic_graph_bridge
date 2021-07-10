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

void ImplTestSotExternalInterface::getControl
(std::map<std::string,
 dynamicgraph::sot::SensorValues> &)
{
  std::cout << "ImplTestSotExternalInterface::getControl" << std::endl;
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

extern "C" {
  dynamicgraph::sot::AbstractSotExternalInterface *createSotExternalInterface()
  { return new ImplTestSotExternalInterface; }
}

extern "C" {
  void destroySotExternalInterface(dynamicgraph::sot::AbstractSotExternalInterface *p) { delete p; }
}
