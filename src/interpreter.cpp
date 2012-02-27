#include <dynamic_graph_bridge/ros_init.hh>
#include <dynamic_graph_bridge/ros_interpreter.hh>

int main ()
{
  ros::NodeHandle& nodeHandle = dynamicgraph::rosInit ();
  dynamicgraph::Interpreter interpreter (nodeHandle);
  ros::spin ();
}
