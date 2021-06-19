#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>

//#include <ros/ros.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/u_int32.h>

#include <dynamic-graph/factory.h>

#include "dynamic_graph_bridge/ros2_init.hh"
#include "ros_subscribe.hh"

namespace dynamicgraph {
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosSubscribe, "RosSubscribe");

namespace command {
namespace rosSubscribe {
Add::Add(RosSubscribe& entity, const std::string& docstring)
    : Command(entity, boost::assign::list_of(Value::STRING)(Value::STRING)(Value::STRING), docstring) {}

Value Add::doExecute() {
  RosSubscribe& entity = static_cast<RosSubscribe&>(owner());
  std::vector<Value> values = getParameterValues();

  const std::string& type = values[0].value();
  const std::string& signal = values[1].value();
  const std::string& topic = values[2].value();

  if (type == "double")
    entity.add<double>(signal, topic);
  else if (type == "unsigned")
    entity.add<unsigned int>(signal, topic);
  else if (type == "matrix")
    entity.add<dg::Matrix>(signal, topic);
  else if (type == "vector")
    entity.add<dg::Vector>(signal, topic);
  else if (type == "vector3")
    entity.add<specific::Vector3>(signal, topic);
  else if (type == "vector3Stamped")
    entity.add<std::pair<specific::Vector3, dg::Vector> >(signal, topic);
  else if (type == "matrixHomo")
    entity.add<sot::MatrixHomogeneous>(signal, topic);
  else if (type == "matrixHomoStamped")
    entity.add<std::pair<sot::MatrixHomogeneous, dg::Vector> >(signal, topic);
  else if (type == "twist")
    entity.add<specific::Twist>(signal, topic);
  else if (type == "twistStamped")
    entity.add<std::pair<specific::Twist, dg::Vector> >(signal, topic);
  else if (type == "string")
    entity.add<std::string>(signal, topic);
  else
    throw std::runtime_error("bad type");
  return Value();
}
}  // namespace rosSubscribe
}  // end of namespace command.

const std::string RosSubscribe::docstring_(
    "Subscribe to a ROS topics and convert it into a dynamic-graph signals.\n"
    "\n"
    "  Use command \"add\" to subscribe to a new signal.\n");

RosSubscribe::RosSubscribe(const std::string& n) : dynamicgraph::Entity(n), nh_(rosInit()), bindedSignal_() {
  std::string docstring =
      "\n"
      "  Add a signal reading data from a ROS topic\n"
      "\n"
      "  Input:\n"
      "    - type: string among ['double', 'matrix', 'vector', 'vector3',\n"
      "                          'vector3Stamped', 'matrixHomo', "
      "'matrixHomoStamped',\n"
      "                          'twist', 'twistStamped'],\n"
      "    - signal: the signal name in dynamic-graph,\n"
      "    - topic:  the topic name in ROS.\n"
      "\n";
  addCommand("add", new command::rosSubscribe::Add(*this, docstring));
}

RosSubscribe::~RosSubscribe() {}

void RosSubscribe::display(std::ostream& os) const { os << CLASS_NAME << std::endl; }

void RosSubscribe::rm(const std::string& signal) {
  std::string signalTs = signal + "Timestamp";

  signalDeregistration(signal);
  bindedSignal_.erase(signal);

  if (bindedSignal_.find(signalTs) != bindedSignal_.end()) {
    signalDeregistration(signalTs);
    bindedSignal_.erase(signalTs);
  }
}

std::vector<std::string> RosSubscribe::list() {
  std::vector<std::string> result(bindedSignal_.size());
  std::transform(bindedSignal_.begin(), bindedSignal_.end(),
      result.begin(), [](const auto& pair) { return pair.first; });
  return result;
}

void RosSubscribe::clear() {
  std::map<std::string, bindedSignal_t>::iterator it = bindedSignal_.begin();
  for (; it != bindedSignal_.end();) {
    rm(it->first);
    it = bindedSignal_.begin();
  }
}

std::string RosSubscribe::getDocString() const { return docstring_; }
}  // end of namespace dynamicgraph.
