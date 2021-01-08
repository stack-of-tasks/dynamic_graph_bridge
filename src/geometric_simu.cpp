/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 */
#include <iostream>
#include <ros/console.h>

#define ENABLE_RT_LOG
#include <dynamic-graph/real-time-logger.h>

#include <dynamic_graph_bridge/sot_loader.hh>

int main(int argc, char *argv[]) {
  ::dynamicgraph::RealTimeLogger::instance()
    .addOutputStream(::dynamicgraph::LoggerStreamPtr_t(new dynamicgraph::LoggerIOStream(std::cout)));

  ros::init(argc, argv, "sot_ros_encapsulator");
  SotLoader aSotLoader;
  aSotLoader.initializeRosNode(argc, argv);
  
  if (aSotLoader.parseOptions(argc, argv) < 0) return -1;

  ros::waitForShutdown ();
  return 0;
}
