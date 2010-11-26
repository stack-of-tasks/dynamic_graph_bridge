#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <dynamic-graph/factory.h>

#include "ros_import.hh"

namespace dynamicgraph
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosImport, "RosImport");

  const char* rosInit()
  {
    int argc = 1;
    char* arg0 = strdup("ros_import");
    char* argv[] = {arg0, 0};
    ros::init(argc, argv, "ros_import");
    free (arg0);
    return "dynamic_graph";
  }

  RosImport::RosImport (const std::string& n)
    : dynamicgraph::Entity(n),
      nh_ (rosInit ()),
      bindedSignal_ ()
  {
  }

  RosImport::~RosImport ()
  {
  }

  void RosImport::display (std::ostream& os) const
  {
    os << CLASS_NAME << std::endl;
  }

  void RosImport::commandLine (const std::string& cmdLine,
			       std::istringstream& cmdArgs,
			       std::ostream& os)
  {
    std::string type;
    std::string signal;
    std::string topic;

    if (cmdLine == "help")
      {
	os << "RosImport: "<< std::endl
	   << "  - add <TYPE> <SIGNAL> <TOPIC>" << std::endl
	   << "  - rm <SIGNAL>" << std::endl
	   << "  - clear" << std::endl
	   << "  - list" << std::endl;
	Entity::commandLine (cmdLine, cmdArgs, os);
      }
    else if (cmdLine == "add")
      {
	cmdArgs >> type >> signal >> topic;
	if (type == "double")
	  add<double> (signal, topic);
	else if (type == "matrix")
	  add<ml::Matrix> (signal, topic);
	else if (type == "vector")
	  add<ml::Vector> (signal, topic);
	else
	  throw "bad type";
      }
    else if (cmdLine == "rm")
      {
	cmdArgs >> signal;
	rm (signal);
      }
    else if (cmdLine == "clear")
      clear ();
    else if (cmdLine == "list")
      list ();
    else
      Entity::commandLine (cmdLine, cmdArgs, os);
  }

  const std::string& RosImport::getClassName ()
  {
    return CLASS_NAME;
  }

  void RosImport::rm (const std::string& signal)
  {
    bindedSignal_.erase (signal);
  }

  void RosImport::list ()
  {
    std::cout << CLASS_NAME << std::endl;
  }

  void RosImport::clear ()
  {
    bindedSignal_.clear ();
  }

} // end of namespace dynamicgraph.
