#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include "dynamic_graph_bridge/ros_init.hh"

namespace dynamicgraph
{
  boost::shared_ptr<ros::NodeHandle> nodeHandle;

  ros::NodeHandle& rosInit()
  {
    if (!nodeHandle)
      {
	int argc = 1;
	char* arg0 = strdup("dynamic_graph_bridge");
	char* argv[] = {arg0, 0};
	ros::init(argc, argv, "dynamic_graph_bridge");
	free (arg0);

	nodeHandle = boost::make_shared<ros::NodeHandle> ("dynamic_graph");
      }
    return *nodeHandle;
  }
} // end of namespace dynamicgraph.
