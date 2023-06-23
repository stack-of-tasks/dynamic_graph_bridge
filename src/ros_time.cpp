///
/// Copyright 2012 CNRS
///
/// Author: Florent Lamiraux

#include "ros_time.hh"

#include <dynamic-graph/factory.h>
#include <dynamic-graph/signal-cast-helper.h>
#include <dynamic-graph/signal-caster.h>

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
    : Entity(_name),
      now_("RosTime(" + _name + ")::output(boost::posix_time::ptime)::time") {
  signalRegistration(now_);
  now_.setConstant(rosTimeToPtime(ros::Time::now()));
  now_.setFunction(boost::bind(&RosTime::update, this, _1, _2));
}

ptime& RosTime::update(ptime& time, const sigtime_t&) {
  time = rosTimeToPtime(ros::Time::now());
  return time;
}

std::string RosTime::getDocString() const { return docstring_; }
}  // namespace dynamicgraph
