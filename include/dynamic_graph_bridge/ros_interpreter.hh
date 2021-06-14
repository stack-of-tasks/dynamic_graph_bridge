#ifndef DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
#define DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
#pragma GCC diagnostic push
#pragma GCC system_header
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#pragma GCC diagnostic pop

#include <dynamic_graph_bridge_msgs/srv/run_command.hpp>
#include <dynamic_graph_bridge_msgs/srv/run_python_file.hpp>
#include <dynamic-graph/python/interpreter.hh>

#include <boost/function.hpp>

namespace dynamicgraph {
/// \brief This class wraps the implementation of the runCommand
/// service.
///
/// This takes as input a ROS node handle and do not handle the
/// callback so that the service behavior can be controlled from
/// the outside.
class Interpreter {
 public:
  typedef boost::function<bool(dynamic_graph_bridge_msgs::srv::RunCommand::Request&,
                               dynamic_graph_bridge_msgs::srv::RunCommand::Response&)>
      runCommandCallback_t;

  typedef boost::function<bool(dynamic_graph_bridge_msgs::srv::RunPythonFile::Request&,
                               dynamic_graph_bridge_msgs::srv::RunPythonFile::Response&)>
      runPythonFileCallback_t;

  explicit Interpreter(rclcpp::Node& nodeHandle);

  /// \brief Method to start python interpreter and deal with messages.
  /// \param Command string to execute, result, stdout, stderr strings.
  void runCommand(const std::string& command, std::string& result, std::string& out, std::string& err);

  /// \brief Method to parse python scripts.
  /// \param Input file name to parse.
  void runPythonFile(std::string ifilename);

  /// Initialize service run_command
  void startRosService();

 protected:
  /// \brief Run a Python command and return result, stderr and stdout.
  void runCommandCallback(dynamic_graph_bridge_msgs::srv::RunCommand::Request& req,
                          dynamic_graph_bridge_msgs::srv::RunCommand::Response& res);

  /// \brief Run a Python file.
  void runPythonFileCallback(dynamic_graph_bridge_msgs::srv::RunPythonFile::Request& req,
                             dynamic_graph_bridge_msgs::srv::RunPythonFile::Response& res);

 private:
  python::Interpreter interpreter_;
  rclcpp::Node& nodeHandle_;
  rclcpp::Service<dynamic_graph_bridge_msgs::srv::RunCommand>::SharedPtr runCommandSrv_;
  rclcpp::Service<dynamic_graph_bridge_msgs::srv::RunPythonFile>::SharedPtr runPythonFileSrv_;
};
}  // end of namespace dynamicgraph.

#endif  //! DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
