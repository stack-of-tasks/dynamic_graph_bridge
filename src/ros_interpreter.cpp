#include "dynamic_graph_bridge/ros_interpreter.hh"

namespace dynamicgraph
{
  static const int queueSize = 5;

  Interpreter::Interpreter (ros::NodeHandle& nodeHandle)
    : interpreter_ (),
      nodeHandle_ (nodeHandle),
      runCommandSrv_ ()
  {
    runCommandCallback_t runCommandCb =
      boost::bind (&Interpreter::runCommandCallback, this, _1, _2);

    runCommandSrv_ =
      nodeHandle_.advertiseService ("run_command", runCommandCb);
  }

  bool
  Interpreter::runCommandCallback(dynamic_graph_bridge::RunCommand::Request& req,
				  dynamic_graph_bridge::RunCommand::Response& res)
  {
    interpreter_.python(req.input, res.result, res.stdout, res.stderr);
    return true;
  }
} // end of namespace dynamicgraph.

