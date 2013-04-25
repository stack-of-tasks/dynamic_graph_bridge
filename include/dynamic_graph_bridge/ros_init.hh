#ifndef ROS_INIT_HH
# define ROS_INIT_HH
# include <ros/ros.h>

namespace dynamicgraph
{
  ros::NodeHandle& rosInit (bool createAsyncSpinner=false, bool createMultiThreadSpinner=true);

  /// \brief Return spinner or throw an exception if spinner
  /// creation has been disabled at startup.
  ros::AsyncSpinner& spinner ();

  /// \brief Return multi threaded spinner or throw an exception if spinner
  /// creation has been disabled at startup.
  ros::MultiThreadedSpinner& mtSpinner ();


} // end of namespace dynamicgraph.

#endif //! ROS_INIT_HH
