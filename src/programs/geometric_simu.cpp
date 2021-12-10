/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 */
#include <iostream>


#define ENABLE_RT_LOG
#include <dynamic-graph/real-time-logger.h>

#include <dynamic_graph_bridge/sot_loader.hh>
#include <dynamic_graph_bridge/ros.hpp>

int main(int argc, char *argv[]) {
  ::dynamicgraph::RealTimeLogger::instance()
    .addOutputStream(::dynamicgraph::LoggerStreamPtr_t(new dynamicgraph::LoggerIOStream(std::cout)));

  rclcpp::init(argc, argv);

  SotLoader aSotLoader;
  if (aSotLoader.parseOptions(argc, argv) < 0) return -1;

  // Advertize service "(start|stop)_dynamic_graph" and
  // load parameter "robot_description in SoT.
  aSotLoader.initialize();

  // Load dynamic library and run python prologue.
  aSotLoader.initPublication();

  aSotLoader.initializeServices();

  dynamic_graph_bridge::ros_spin();
  return 0;
}
