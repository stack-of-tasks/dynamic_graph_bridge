///
/// Copyright 2012 CNRS
///
/// Author: Florent Lamiraux

#ifndef DYNAMIC_GRAPH_ROS_TIME_HH
#define DYNAMIC_GRAPH_ROS_TIME_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <ros/time.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace dynamicgraph {

class RosTime : public dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  Signal<boost::posix_time::ptime, sigtime_t> now_;
  RosTime(const std::string& name);
  virtual std::string getDocString() const;

 protected:
  boost::posix_time::ptime& update(boost::posix_time::ptime& time,
                                   const sigtime_t& t);

 private:
  static const std::string docstring_;
};  // class RosTime

template <>
struct signal_io<boost::posix_time::ptime>
    : signal_io_unimplemented<boost::posix_time::ptime> {};

}  // namespace dynamicgraph

#endif  // DYNAMIC_GRAPH_ROS_TIME_HH
