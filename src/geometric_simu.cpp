/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 * This file is part of dynamic_graph_bridge.
 * dynamic_graph_bridge is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * dynamic_graph_bridge is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with dynamic_graph_bridge.  If not, see <http://www.gnu.org/licenses/>.
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
