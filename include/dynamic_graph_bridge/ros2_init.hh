#ifndef ROS2_INIT_HH
#define ROS2_INIT_HH
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

namespace dynamicgraph {
  class RosContext {
  public:

    rclcpp::Node::SharedPtr rosInit();
    rclcpp::executors::MultiThreadedExecutor::SharedPtr rosInitGetExecutor();

    std::shared_ptr<rclcpp::Node> nodeHandle;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> mtExecutor;

    typedef std::shared_ptr<RosContext> SharedPtr;
  };

}  // end of namespace dynamicgraph.

#endif  //! ROS_INIT_HH
