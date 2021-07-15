//
// Copyright (c) 2017-2018 CNRS
// Authors: Joseph Mirabel
//
//

#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>

#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/u_int32.h>

#include <dynamic-graph/factory.h>

#include "dynamic_graph_bridge/ros2_init.hh"
#include "ros_queued_subscribe.hh"

#include "dynamic_graph_bridge_msgs/msg/matrix.hpp"
#include "dynamic_graph_bridge_msgs/msg/vector.hpp"

namespace dynamicgraph {
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosQueuedSubscribe, "RosQueuedSubscribe");
  
namespace command {
namespace rosQueuedSubscribe {
Add::Add(RosQueuedSubscribe& entity, const std::string& docstring)
    : Command(entity, boost::assign::list_of(Value::STRING)(Value::STRING)(Value::STRING), docstring) {}

Value Add::doExecute() {
  RosQueuedSubscribe& entity = static_cast<RosQueuedSubscribe&>(owner());
  std::vector<Value> values = getParameterValues();

  const std::string& type = values[0].value();
  const std::string& signal = values[1].value();
  const std::string& topic = values[2].value();

  if (type == "double")
    entity.add<double>(type, signal, topic);
  else if (type == "unsigned")
    entity.add<unsigned int>(type, signal, topic);
  else if (type == "matrix")
    entity.add<Matrix>(type, signal, topic);
  else if (type == "vector")
    entity.add<Vector>(type, signal, topic);
  else if (type == "vector3")
    entity.add<specific::Vector3>(type, signal, topic);
  else if (type == "matrixHomo")
    entity.add<sot::MatrixHomogeneous>(type, signal, topic);
  else if (type == "twist")
    entity.add<specific::Twist>(type, signal, topic);
  else
    throw std::runtime_error("bad type");
  return Value();
}
}  // namespace rosQueuedSubscribe
}  // end of namespace command.

const std::string RosQueuedSubscribe::docstring_(
    "Subscribe to a ROS topics and convert it into a dynamic-graph signals.\n"
    "\n"
    "  Use command \"add\" to subscribe to a new signal.\n");

RosQueuedSubscribe::RosQueuedSubscribe(const std::string& n)
    : dynamicgraph::Entity(n), nh_(), bindedSignal_(), readQueue_(-1) {
  std::string docstring =
      "\n"
      "  Add a signal reading data from a ROS topic\n"
      "\n"
      "  Input:\n"
      "    - type: string among ['double', 'matrix', 'vector', 'vector3',\n"
      "                          'matrixHomo', 'twist'],\n"
      "    - signal: the signal name in dynamic-graph,\n"
      "    - topic:  the topic name in ROS.\n"
      "\n";
  addCommand("add", new command::rosQueuedSubscribe::Add(*this, docstring));
}

RosQueuedSubscribe::~RosQueuedSubscribe() {}

void RosQueuedSubscribe::initializeRosContext( dynamicgraph::RosContext::SharedPtr ros_context) {
  nh_ = ros_context->nodeHandle;
}

void RosQueuedSubscribe::display(std::ostream& os) const { os << CLASS_NAME << std::endl; }

void RosQueuedSubscribe::rm(const std::string& signal) {
  std::string signalTs = signal + "Timestamp";

  signalDeregistration(signal);
  bindedSignal_.erase(signal);

  if (bindedSignal_.find(signalTs) != bindedSignal_.end()) {
    signalDeregistration(signalTs);
    bindedSignal_.erase(signalTs);
  }
}

std::vector<std::string> RosQueuedSubscribe::list() {
  std::vector<std::string> result(bindedSignal_.size());
  std::transform(bindedSignal_.begin(), bindedSignal_.end(),
      result.begin(), [](const auto& pair) { return pair.first; });
  return result;
}

void RosQueuedSubscribe::clear() {
  std::map<std::string, bindedSignal_t>::iterator it = bindedSignal_.begin();
  for (; it != bindedSignal_.end();) {
    rm(it->first);
    it = bindedSignal_.begin();
  }
}

void RosQueuedSubscribe::clearQueue(const std::string& signal) {
  if (bindedSignal_.find(signal) != bindedSignal_.end()) {
    bindedSignal_[signal]->clear();
  }
}

std::size_t RosQueuedSubscribe::queueSize(const std::string& signal) const {
  std::map<std::string, bindedSignal_t>::const_iterator _bs = bindedSignal_.find(signal);
  if (_bs != bindedSignal_.end()) {
    return _bs->second->size();
  }
  return static_cast<std::size_t>(-1);
}

void RosQueuedSubscribe::readQueue(int beginReadingAt) {
  // Prints signal queues sizes
  /*for (std::map<std::string, bindedSignal_t>::const_iterator it =
         bindedSignal_.begin (); it != bindedSignal_.end (); it++) {
    std::cout << it->first << " : " << it->second->size() << '\n';
  }*/
  readQueue_ = beginReadingAt;
}

std::string RosQueuedSubscribe::getDocString() const { return docstring_; }
}  // end of namespace dynamicgraph.
