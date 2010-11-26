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

  RosImport::RosImport (const std::string& n)
    : dynamicgraph::Entity(n),
      nh_ (),
      bindedSignal_ ()
  {
    int argc = 1;
    char* arg0 = strdup("ros_import");
    char* argv[] = {arg0, 0};
    ros::init(argc, argv, "ros_import");
    free (arg0);

    nh_ = ros::NodeHandle ("dynamic_graph");

    //ros::Publisher blob_pub = n.advertise<hueblob::Blob>("blob", 1000);

    // ros::Rate loop_rate(10);

    // while (ros::ok())
    //   {
    // 	hueblob::Blob blob;
    // 	blob_pub.publish(blob);

    // 	ros::spinOnce();
    // 	loop_rate.sleep();
    //   }
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
    std::string signal;
    std::string topic;

    if (cmdLine == "help")
      {
	os << "RosImport: "<< std::endl
	   << "  - add <SIGNAL> <TOPIC>" << std::endl;
	Entity::commandLine (cmdLine, cmdArgs, os);
      }
    else if (cmdLine == "add")
      {
	cmdArgs >> signal >> topic;
	add<double> (signal, topic);
      }
    else if (cmdLine == "rm")
      {
	cmdArgs >> signal >> topic;
	rm (signal, topic);
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

  void RosImport::rm (const std::string& signal, const std::string& topic)
  {
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
