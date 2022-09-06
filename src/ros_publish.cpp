#include "ros_publish.hh"

#include <dynamic-graph/command.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/linear-algebra.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>

#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <stdexcept>

#include "dynamic_graph_bridge/ros_init.hh"

#define ENABLE_RT_LOG
#include <dynamic-graph/real-time-logger.h>

namespace dynamicgraph {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosPublish, "RosPublish");
const double RosPublish::ROS_JOINT_STATE_PUBLISHER_RATE = 0.01;

namespace command {
namespace rosPublish {

Add::Add(RosPublish& entity, const std::string& docstring)
    : Command(
          entity,
          boost::assign::list_of(Value::STRING)(Value::STRING)(Value::STRING),
          docstring) {}

Value Add::doExecute() {
  RosPublish& entity = static_cast<RosPublish&>(owner());
  std::vector<Value> values = getParameterValues();

  const std::string& type = values[0].value();
  const std::string& signal = values[1].value();
  const std::string& topic = values[2].value();

  if (type == "boolean")
    entity.add<bool>(signal, topic);
  else if (type == "double")
    entity.add<double>(signal, topic);
  else if (type == "unsigned")
    entity.add<unsigned int>(signal, topic);
  else if (type == "int")
    entity.add<int>(signal, topic);
  else if (type == "matrix")
    entity.add<Matrix>(signal, topic);
  else if (type == "vector")
    entity.add<Vector>(signal, topic);
  else if (type == "vector3")
    entity.add<specific::Vector3>(signal, topic);
  else if (type == "vector3Stamped")
    entity.add<std::pair<specific::Vector3, Vector> >(signal, topic);
  else if (type == "matrixHomo")
    entity.add<sot::MatrixHomogeneous>(signal, topic);
  else if (type == "matrixHomoStamped")
    entity.add<std::pair<sot::MatrixHomogeneous, Vector> >(signal, topic);
  else if (type == "twist")
    entity.add<specific::Twist>(signal, topic);
  else if (type == "twistStamped")
    entity.add<std::pair<specific::Twist, Vector> >(signal, topic);
  else if (type == "string")
    entity.add<std::string>(signal, topic);
  else
    throw std::runtime_error("bad type");
  return Value();
}

}  // namespace rosPublish
}  // end of namespace command.

const std::string RosPublish::docstring_(
    "Publish dynamic-graph signals as ROS topics.\n"
    "\n"
    "  Use command \"add\" to publish a new ROS topic.\n");

RosPublish::RosPublish(const std::string& n)
    : dynamicgraph::Entity(n),
      // RosPublish do not use callback so do not create a useless spinner.
      nh_(rosInit(false)),
      bindedSignal_(),
      trigger_(boost::bind(&RosPublish::trigger, this, _1, _2), sotNOSIGNAL,
               MAKE_SIGNAL_STRING(name, true, "int", "trigger")),
      rate_(0, 10000000),
      nextPublication_() {
  aofs_.open("/tmp/ros_publish.txt");
  dgADD_OSTREAM_TO_RTLOG(aofs_);

  try {
    if (ros::Time::isSimTime())
      nextPublication_ = ros::Time::now();
    else {
      clock_gettime(CLOCK_REALTIME, &nextPublicationRT_);
    }
  } catch (const std::exception& exc) {
    throw std::runtime_error("Failed to call ros::Time::now ():" +
                             std::string(exc.what()));
  }
  signalRegistration(trigger_);
  trigger_.setNeedUpdateFromAllChildren(true);

  std::string docstring =
      "\n"
      "  Add a signal writing data to a ROS topic\n"
      "\n"
      "  Input:\n"
      "    - type: string among ['double', 'matrix', 'vector', 'vector3',\n"
      "                          'vector3Stamped', 'matrixHomo', "
      "'matrixHomoStamped',\n"
      "                          'twist', 'twistStamped'],\n"
      "    - signal: the signal name in dynamic-graph,\n"
      "    - topic:  the topic name in ROS.\n"
      "\n";
  addCommand("add", new command::rosPublish::Add(*this, docstring));
}

RosPublish::~RosPublish() { aofs_.close(); }

void RosPublish::display(std::ostream& os) const {
  os << CLASS_NAME << std::endl;
}

void RosPublish::rm(const std::string& signal) {
  if (bindedSignal_.find(signal) == bindedSignal_.end()) return;

  if (signal == "trigger") {
    std::cerr << "The trigger signal should not be removed. Aborting."
              << std::endl;
    return;
  }

  // lock the mutex to avoid deleting the signal during a call to trigger
  boost::mutex::scoped_lock lock(mutex_);

  signalDeregistration(signal);
  bindedSignal_.erase(signal);
}

std::vector<std::string> RosPublish::list() const {
  std::vector<std::string> result(bindedSignal_.size());
  std::transform(bindedSignal_.begin(), bindedSignal_.end(), result.begin(),
                 [](const auto& pair) { return pair.first; });
  return result;
}

void RosPublish::clear() {
  std::map<std::string, bindedSignal_t>::iterator it = bindedSignal_.begin();
  for (; it != bindedSignal_.end();) {
    if (it->first != "trigger") {
      rm(it->first);
      it = bindedSignal_.begin();
    } else {
      ++it;
    }
  }
}

int& RosPublish::trigger(int& dummy, int t) {
  typedef std::map<std::string, bindedSignal_t>::iterator iterator_t;
  ros::Time aTime;
  if (ros::Time::isSimTime()) {
    aTime = ros::Time::now();
    if (aTime <= nextPublication_) return dummy;

    nextPublication_ = aTime + rate_;
  } else {
    struct timespec aTimeRT;
    clock_gettime(CLOCK_REALTIME, &aTimeRT);
    nextPublicationRT_.tv_sec = aTimeRT.tv_sec + rate_.sec;
    nextPublicationRT_.tv_nsec = aTimeRT.tv_nsec + rate_.nsec;
    if (nextPublicationRT_.tv_nsec > 1000000000) {
      nextPublicationRT_.tv_nsec -= 1000000000;
      nextPublicationRT_.tv_sec += 1;
    }
  }

  boost::mutex::scoped_lock lock(mutex_);

  for (iterator_t it = bindedSignal_.begin(); it != bindedSignal_.end(); ++it) {
    boost::get<1>(it->second)(t);
  }
  return dummy;
}

std::string RosPublish::getDocString() const { return docstring_; }

}  // end of namespace dynamicgraph.
