#ifndef _DGB_IMPL_TEST_SOT_EXTERNAL_INTEFACE_HH_
#define _DGB_IMPL_TEST_SOT_EXTERNAL_INTEFACE_HH_

#include <dynamic_graph_bridge/ros_python_interpreter_server.hpp>
#include <iostream>
#include <sot/core/abstract-sot-external-interface.hh>

#include "impl_test_sot_mock_device.hh"

namespace dynamic_graph_bridge {
class ImplTestSotExternalInterface
    : public dynamicgraph::sot::AbstractSotExternalInterface {
 public:
  ImplTestSotExternalInterface();
  ImplTestSotExternalInterface(const char robotName[]);
  ImplTestSotExternalInterface(std::string robotName);
  virtual ~ImplTestSotExternalInterface();

  virtual void setupSetSensors(
      std::map<std::string, dynamicgraph::sot::SensorValues> &) final;

  virtual void nominalSetSensors(
      std::map<std::string, dynamicgraph::sot::SensorValues> &) final;

  virtual void cleanupSetSensors(
      std::map<std::string, dynamicgraph::sot::SensorValues> &) final;
  virtual void getControl(
      std::map<std::string, dynamicgraph::sot::ControlValues> &) final;

  virtual void setSecondOrderIntegration(void);

  virtual void setNoIntegration(void);

  /// Embedded python interpreter accessible via Corba/ros
  boost::shared_ptr<dynamic_graph_bridge::RosPythonInterpreterServer>
      py_interpreter_srv_;

 protected:
  // Named ctrl vector
  dynamicgraph::sot::ControlValues named_ctrl_vec_;

  // Named base free flyer vector
  dynamicgraph::sot::ControlValues named_base_ff_vec_;

  // Update output port with the control computed from the
  // dynamic graph.
  void updateRobotState(std::vector<double> &anglesIn);

  /// Run a python command
  void runPython(std::ostream &file, const std::string &command,
                 dynamicgraph::Interpreter &interpreter);

  void init();

  ImplTestSotMockDevice *device_;
};
}  // namespace dynamic_graph_bridge
#endif
