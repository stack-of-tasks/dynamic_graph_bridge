#ifndef DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
# define DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
# include <ros/ros.h>
# include <dynamic_graph_bridge_msgs/RunCommand.h>
# include <dynamic-graph/python/interpreter.hh>

namespace dynamicgraph
{
  /// \brief This class wraps the implementation of the runCommand
  /// service.
  ///
  /// This takes as input a ROS node handle and do not handle the
  /// callback so that the service behavior can be controlled from
  /// the outside.
  class Interpreter
  {
  public:
    typedef boost::function<
    bool (dynamic_graph_bridge_msgs::RunCommand::Request&,
    	  dynamic_graph_bridge_msgs::RunCommand::Response&)>
    runCommandCallback_t;

    explicit Interpreter (ros::NodeHandle& nodeHandle);

    /// \brief Run a command and return result.
    std::string runCommand (const std::string& command) __attribute__ ((deprecated));
    /// \brief Method to start python interpreter and deal with messages.
    /// \param Command string to execute, result, stdout, stderr strings.
    void runCommand(const std::string & command, std::string &result,
		    std::string &out, std::string &err);

    /// Initialize service run_command
    void startRosService ();

  protected:
    /// \brief Run a Python command and return result, stderr and stdout.
    bool runCommandCallback (dynamic_graph_bridge_msgs::RunCommand::Request& req,
			     dynamic_graph_bridge_msgs::RunCommand::Response& res);

  private:
    python::Interpreter interpreter_;
    ros::NodeHandle& nodeHandle_;
    ros::ServiceServer runCommandSrv_;
  };
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
