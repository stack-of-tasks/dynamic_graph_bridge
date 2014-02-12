#include "dynamic_graph_bridge/ros_interpreter.hh"

namespace dynamicgraph
{
  static const int queueSize = 5;

  Interpreter::Interpreter (ros::NodeHandle& nodeHandle)
    : interpreter_ (),
      nodeHandle_ (nodeHandle),
      runCommandSrv_ ()
  {
  }

  void Interpreter::startRosService ()
  {
    runCommandCallback_t runCommandCb =
      boost::bind (&Interpreter::runCommandCallback, this, _1, _2);
    runCommandSrv_ =
      nodeHandle_.advertiseService ("run_command", runCommandCb);
  }

  bool
  Interpreter::runCommandCallback
  (dynamic_graph_bridge_msgs::RunCommand::Request& req,
   dynamic_graph_bridge_msgs::RunCommand::Response& res)
  {
    interpreter_.python(req.input, res.result, res.stdout, res.stderr);
    return true;
  }

  std::string
  Interpreter::runCommand
  (const std::string& command)
  {
    return interpreter_.python(command);
  }

  void Interpreter::runCommand
  (const std::string & command, 
   std::string &result,
   std::string &out, 
   std::string &err)
  {
    interpreter_.python(command, result, out, err);
  }

} // end of namespace dynamicgraph.

