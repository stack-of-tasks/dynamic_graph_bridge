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
  if (aSotLoader.parseOptions(argc, argv) < 0) return -1;

  // Advertize service "(start|stop)_dynamic_graph" and
  // load parameter "robot_description in SoT.
  aSotLoader.initializeRosNode(argc, argv);
  // Load dynamic library and run python prologue.
  aSotLoader.Initialization();

  ros::waitForShutdown ();
  return 0;
}
