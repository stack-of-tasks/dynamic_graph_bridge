#ifndef _ROS_DYNAMIC_GRAPH_PARAMETER_
#define _ROS_DYNAMIC_GRAPH_PARAMETER_

#include "rclcpp/node.hpp"
namespace dynamicgraph {

bool parameter_server_read_robot_description(rclcpp::Node::SharedPtr nh);

}
#endif /* _ROS_DYNAMIC_GRAPH_PARAMETER_ */
