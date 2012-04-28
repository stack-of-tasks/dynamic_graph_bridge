///
/// Copyright 2012 CNRS
///
/// Author: Florent Lamiraux

#include <dynamic-graph/factory.h>
#include <dynamic-graph/signal-caster.h>
#include <dynamic-graph/signal-cast-helper.h>

#include "ros_time.hh"
#include "converter.hh"

namespace dynamicgraph {

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTime, "RosTime");

  using namespace boost::posix_time;

  RosTime::RosTime (const std::string& name) :
    Entity (name),
    now_ ("RosTime("+name+")::output(boost::posix_time::ptime)::time")
  {
    signalRegistration (now_);
    now_.setConstant (rosTimeToPtime (ros::Time::now()));
    now_.setFunction (boost::bind (&RosTime::update, this, _1, _2));
  }

  ptime&
  RosTime::update (ptime& time, const int& t)
  {
    time = rosTimeToPtime (ros::Time::now ());
    return time;
  }

} // namespace dynamicgraph
