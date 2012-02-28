#include <dynamic_graph_bridge/ros_init.hh>
#include <dynamic_graph_bridge/ros_interpreter.hh>

int main ()
{
  // we spin explicitly so we do not need an async spinner here.
  ros::NodeHandle& nodeHandle = dynamicgraph::rosInit (false);
  dynamicgraph::Interpreter interpreter (nodeHandle);
  ros::spin ();
}
