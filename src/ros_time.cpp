///
/// Copyright 2012-2021 CNRS
///
/// Authors: Florent Lamiraux, Olivier Stasse

#include <rclcpp/clock.hpp>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/signal-caster.h>
#include <dynamic-graph/signal-cast-helper.h>

#include "ros_time.hh"
#include "converter.hh"

namespace dynamicgraph {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTime, "RosTime");

using namespace boost::posix_time;

const std::string RosTime::docstring_(
    "Export ROS time into dynamic-graph.\n"
    "\n"
    "  Signal \"time\" provides time as given by ros::time as\n"
    "  boost::posix_time::ptime type.\n");

RosTime::RosTime(const std::string& _name)
    : Entity(_name), now_("RosTime(" + _name + ")::output(boost::posix_time::ptime)::time") {
  signalRegistration(now_);
  now_.setConstant(rosTimeToPtime(rclcpp::Clock().now()));
  now_.setFunction(boost::bind(&RosTime::update, this, _1, _2));
}

ptime& RosTime::update(ptime& time, const int&) {
  time = rosTimeToPtime(rclcpp::Clock().now());
  return time;
}

std::string RosTime::getDocString() const { return docstring_; }
}  // namespace dynamicgraph
