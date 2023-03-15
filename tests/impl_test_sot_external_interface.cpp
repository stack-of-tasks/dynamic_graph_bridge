#include "impl_test_sot_external_interface.hh"

#include "dynamic_graph_bridge/ros.hpp"

using namespace dynamic_graph_bridge;

ImplTestSotExternalInterface::ImplTestSotExternalInterface()
    : device_(new ImplTestSotMockDevice("RobotName")) {
  init();
}

ImplTestSotExternalInterface::ImplTestSotExternalInterface(
    std::string RobotName)
    : device_(new ImplTestSotMockDevice(RobotName)) {
  init();
}

ImplTestSotExternalInterface::ImplTestSotExternalInterface(
    const char RobotName[])
    : device_(new ImplTestSotMockDevice(RobotName)) {
  init();
}

ImplTestSotExternalInterface::~ImplTestSotExternalInterface() {}

void ImplTestSotExternalInterface::init() {
  std::vector<double> ctrl_vector;
  ctrl_vector.resize(2);

  // Dummy values for the control vector
  ctrl_vector[0] = 0.0;
  ctrl_vector[1] = 0.0;

  // Creates the named control vector
  named_ctrl_vec_.setName("control");
  named_ctrl_vec_.setValues(ctrl_vector);

  // Creates the base eff vector
  named_base_ff_vec_.setName("baseff");

  ctrl_vector.resize(7);
  for (std::vector<double>::size_type i = 0; i < 6; i++) ctrl_vector[i] = 0.0;
  ctrl_vector[6] = 0.0;
  named_base_ff_vec_.setValues(ctrl_vector);

  // rosInit is called here only to initialize ros.
  // No spinner is initialized.
  py_interpreter_srv_ =
      boost::shared_ptr<dynamic_graph_bridge::RosPythonInterpreterServer>(
          new dynamic_graph_bridge::RosPythonInterpreterServer());

  RosNodePtr py_inter_ptr = get_ros_node("python_interpreter");
  // Set the control time step parameter to 0.001
  double ts = 0.001;

  py_inter_ptr->declare_parameter<double>("/sot_controller/dt", ts);
  device_->timeStep(ts);
}

void ImplTestSotExternalInterface::setupSetSensors(
    std::map<std::string, dynamicgraph::sot::SensorValues> &) {
  std::cout << "ImplTestSotExternalInterface::setupSetSensors" << std::endl;
}

void ImplTestSotExternalInterface::nominalSetSensors(
    std::map<std::string, dynamicgraph::sot::SensorValues> &) {
  std::cout << "ImplTestSotExternalInterface::nominalSetSensors" << std::endl;
}

void ImplTestSotExternalInterface::cleanupSetSensors(
    std::map<std::string, dynamicgraph::sot::SensorValues> &) {
  std::cout << "ImplTestSotExternalInterface::cleanupSetSensors" << std::endl;
}

void ImplTestSotExternalInterface::getControl(
    std::map<std::string, dynamicgraph::sot::ControlValues> &controlValues) {
  // Put the default named_ctrl_vec inside the map controlValues.
  controlValues["control"] = named_ctrl_vec_;
  controlValues["baseff"] = named_base_ff_vec_;
}
void ImplTestSotExternalInterface::setSecondOrderIntegration(void) {
  std::cout << "ImplTestSotExternalInterface::setSecondOrderIntegration"
            << std::endl;
}

void ImplTestSotExternalInterface::setNoIntegration(void) {
  std::cout << "setNoIntegration" << std::endl;
}

extern "C" {
dynamicgraph::sot::AbstractSotExternalInterface *createSotExternalInterface() {
  return new ImplTestSotExternalInterface;
}
}

extern "C" {
void destroySotExternalInterface(
    dynamicgraph::sot::AbstractSotExternalInterface *p) {
  delete p;
}
}
