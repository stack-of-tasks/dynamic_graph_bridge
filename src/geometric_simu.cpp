/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 */
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include <iostream>

#define ENABLE_RT_LOG
#include <dynamic-graph/real-time-logger.h>

#include <dynamic_graph_bridge/sot_loader.hh>

int main(int argc, char *argv[]) {
  dgADD_OSTREAM_TO_RTLOG(std::cerr);

  ros::init(argc, argv, "sot_ros_encapsulator");
  SotLoader aSotLoader;
  if (aSotLoader.parseOptions(argc, argv) < 0) return -1;

  aSotLoader.initializeRosNode(argc, argv);

  ros::spin();
  return 0;
}
