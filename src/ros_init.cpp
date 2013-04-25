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
    boost::shared_ptr<ros::MultiThreadedSpinner> mtSpinner;
  };
  GlobalRos ros;

  ros::NodeHandle& rosInit (bool createAsyncSpinner, bool createMultiThreadedSpinner)
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
	ros.spinner = boost::make_shared<ros::AsyncSpinner> (4);
	ros.spinner->start ();
      }
    else 
      {
	if (!ros.mtSpinner && createMultiThreadedSpinner)
	  {
	    ros.mtSpinner = boost::make_shared<ros::MultiThreadedSpinner>(4);
	  }
      }
    return *ros.nodeHandle;
  }

  ros::AsyncSpinner& spinner ()
  {
    if (!ros.spinner)
      throw std::runtime_error ("spinner has not been created");
    return *ros.spinner;
  }

  ros::MultiThreadedSpinner& mtSpinner ()
  {
    if (!ros.mtSpinner)
      throw std::runtime_error ("spinner has not been created");
    return *ros.mtSpinner;
  }

} // end of namespace dynamicgraph.
