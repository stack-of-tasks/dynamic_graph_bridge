#include <stdexcept>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include "dynamic_graph_bridge/ros_init.hh"

namespace dynamicgraph
{
  struct GlobalRos
  {
    ~GlobalRos ()
    {
      if (spinner)
	spinner->stop ();
      if (nodeHandle)
	nodeHandle->shutdown ();
    }

    boost::shared_ptr<ros::NodeHandle> nodeHandle;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
  };
  GlobalRos ros;

  ros::NodeHandle& rosInit (bool createAsyncSpinner)
  {
    if (!ros.nodeHandle)
      {
	int argc = 1;
	char* arg0 = strdup("dynamic_graph_bridge");
	char* argv[] = {arg0, 0};
	ros::init(argc, argv, "dynamic_graph_bridge");
	free (arg0);

	ros.nodeHandle = boost::make_shared<ros::NodeHandle> ("");
      }
    if (!ros.spinner && createAsyncSpinner)
      {
	ros.spinner = boost::make_shared<ros::AsyncSpinner> (1);
	ros.spinner->start ();
      }
    return *ros.nodeHandle;
  }

  ros::AsyncSpinner& spinner ()
  {
    if (!ros.spinner)
      throw std::runtime_error ("spinner has not been created");
    return *ros.spinner;
  }
} // end of namespace dynamicgraph.
