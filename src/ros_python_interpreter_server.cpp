/**
 * @file ros_interpreter.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include "dynamic_graph_bridge/ros_python_interpreter_server.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <functional>
#include <memory>

namespace dynamic_graph_bridge {

RosPythonInterpreterServer::RosPythonInterpreterServer()
    : interpreter_(),
      ros_node_(get_ros_node("python_interpreter")),
      run_python_command_srv_(nullptr),
      run_python_file_srv_(nullptr) {
  ros_add_node_to_executor("python_interpreter");
}

RosPythonInterpreterServer::~RosPythonInterpreterServer() {}

void RosPythonInterpreterServer::start_ros_service(
    const rclcpp::QoS& qos, rclcpp::CallbackGroup::SharedPtr group) {
  run_python_command_callback_t runCommandCb =
      std::bind(&RosPythonInterpreterServer::runCommandCallback, this,
                std::placeholders::_1, std::placeholders::_2);
  run_python_command_srv_ = ros_node_->create_service<RunPythonCommandSrvType>(
      "/dynamic_graph_bridge/run_python_command", runCommandCb, qos, group);
  RCLCPP_INFO(
      rclcpp::get_logger("dynamic_graph_bridge"),
      "RosPythonInterpreterServer::start_ros_service - run_python_command...");

  run_python_file_callback_t runPythonFileCb =
      std::bind(&RosPythonInterpreterServer::runPythonFileCallback, this,
                std::placeholders::_1, std::placeholders::_2);
  run_python_file_srv_ = ros_node_->create_service<RunPythonFileSrvType>(
      "/dynamic_graph_bridge/run_python_file", runPythonFileCb, qos, group);

  RCLCPP_INFO(
      rclcpp::get_logger("dynamic_graph_bridge"),
      "RosPythonInterpreterServer::start_ros_service - run_python_file...");
}

void RosPythonInterpreterServer::runCommandCallback(
    RunPythonCommandRequestPtr req, RunPythonCommandResponsePtr res) {
  std::stringstream buffer;
  std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

  run_python_command(req->input, res->result, res->standardoutput,
                     res->standarderror);

  if (!buffer.str().empty()) {
    if (!boost::algorithm::ends_with(res->standardoutput, "\n") &&
        !res->standardoutput.empty()) {
      res->standardoutput += "\n";
    }
    res->standardoutput += buffer.str();
  }
  std::cout.rdbuf(old);

  std::cout << res->standardoutput << std::endl;
}

void RosPythonInterpreterServer::runPythonFileCallback(
    RunPythonFileRequestPtr req, RunPythonFileResponsePtr res) {
  RCLCPP_INFO(rclcpp::get_logger("dynamic_graph_bridge"),
              "RosPythonInterpreterServer::runPythonFileCallback- begin");
  run_python_file(req->input, res->result);
  RCLCPP_INFO(rclcpp::get_logger("dynamic_graph_bridge"),
              "RosPythonInterpreterServer::runPythonFileCallback- end");
}

void RosPythonInterpreterServer::run_python_command(const std::string& command,
                                                    std::string& result,
                                                    std::string& out,
                                                    std::string& err) {
  interpreter_.python(command, result, out, err);
}

void RosPythonInterpreterServer::run_python_file(const std::string ifilename,
                                                 std::string& result) {
  interpreter_.runPythonFile(ifilename, result);
}

}  // namespace dynamic_graph_bridge
