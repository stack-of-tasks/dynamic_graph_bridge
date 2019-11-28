/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 */
#include <ros/console.h>

#define ENABLE_RT_LOG
#include <dynamic-graph/real-time-logger.h>

#include <dynamic_graph_bridge/sot_loader.hh>

class LoggerROSStream : public ::dynamicgraph::LoggerStream {
public:
  void write(const char *c) { ROS_ERROR(c); }
};

int main(int argc, char *argv[]) {
  ::dynamicgraph::RealTimeLogger::instance()
    .addOutputStream(::dynamicgraph::LoggerStreamPtr_t(new LoggerROSStream()));

  ros::init(argc, argv, "sot_ros_encapsulator");
  SotLoader aSotLoader;
  if (aSotLoader.parseOptions(argc, argv) < 0) return -1;

  aSotLoader.initializeRosNode(argc, argv);

  ros::spin();
  return 0;
}
