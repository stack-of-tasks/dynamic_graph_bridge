#include "dynamic_graph_bridge/ros_interpreter.hh"

namespace dynamicgraph {
static const int queueSize = 5;

  Interpreter::Interpreter(rclcpp::Node& nodeHandle)
    : interpreter_(), nodeHandle_(nodeHandle), runCommandSrv_(), runPythonFileSrv_() {}
  
  void Interpreter::startRosService() {
    runCommandCallback_t runCommandCb = std::bind(&Interpreter::runCommandCallback, this,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2);
    runCommandSrv_ = nodeHandle_.create_service<dynamic_graph_bridge_msgs::srv::RunCommand>
      ("run_command", runCommandCb);
    
    runPythonFileCallback_t runPythonFileCb = std::bind(&Interpreter::runPythonFileCallback, this,
                                                        std::placeholders::_1,
                                                        std::placeholders::_2);
    runPythonFileSrv_ = nodeHandle_.create_service<dynamic_graph_bridge_msgs::srv::RunPythonFile>
      ("run_script", runPythonFileCb);
  }
  
  void Interpreter::runCommandCallback(dynamic_graph_bridge_msgs::srv::RunCommand::Request& req,
                                       dynamic_graph_bridge_msgs::srv::RunCommand::Response& res) {
    interpreter_.python(req.input, res.result, res.standardoutput, res.standarderror);
  }
  
  void Interpreter::runPythonFileCallback(dynamic_graph_bridge_msgs::srv::RunPythonFile::Request& req,
                                          dynamic_graph_bridge_msgs::srv::RunPythonFile::Response& res) {
    interpreter_.runPythonFile(req.input);
    res.result = "File parsed";  // FIX: It is just an echo, is there a way to
    // have a feedback?
  }
  
  void Interpreter::runCommand(const std::string& command, std::string& result, std::string& out, std::string& err) {
    interpreter_.python(command, result, out, err);
    if (err.size() > 0) {
      RCLCPP_ERROR(rclcpp::get_logger("dynamic_graph_bridge"),err.c_str());
    }
  }
  
  void Interpreter::runPythonFile(std::string ifilename) { interpreter_.runPythonFile(ifilename); }
  
}  // end of namespace dynamicgraph.
