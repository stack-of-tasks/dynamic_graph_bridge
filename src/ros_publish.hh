#ifndef DYNAMIC_GRAPH_ROS_PUBLISH_HH
#define DYNAMIC_GRAPH_ROS_PUBLISH_HH
#include <dynamic-graph/command.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/tuple/tuple.hpp>
#include <fstream>
#include <map>

#include "converter.hh"
#include "sot_to_ros.hh"

namespace dynamicgraph {
class RosPublish;

namespace command {
namespace rosPublish {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

#define ROS_PUBLISH_MAKE_COMMAND(CMD)                      \
  class CMD : public Command {                             \
   public:                                                 \
    CMD(RosPublish& entity, const std::string& docstring); \
    virtual Value doExecute();                             \
  }

ROS_PUBLISH_MAKE_COMMAND(Add);

#undef ROS_PUBLISH_MAKE_COMMAND

}  // namespace rosPublish
}  // end of namespace command.

/// \brief Publish dynamic-graph information into ROS.
class RosPublish : public dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  typedef boost::function<void(sigtime_t)> callback_t;

  typedef boost::tuple<boost::shared_ptr<dynamicgraph::SignalBase<sigtime_t> >,
                       callback_t>
      bindedSignal_t;

  static const double ROS_JOINT_STATE_PUBLISHER_RATE;

  RosPublish(const std::string& n);
  virtual ~RosPublish();

  virtual std::string getDocString() const;
  void display(std::ostream& os) const;

  void add(const std::string& signal, const std::string& topic);
  void rm(const std::string& signal);
  std::vector<std::string> list() const;
  void clear();

  int& trigger(int&, sigtime_t);

  template <typename T>
  void sendData(
      boost::shared_ptr<
          realtime_tools::RealtimePublisher<typename SotToRos<T>::ros_t> >
          publisher,
      boost::shared_ptr<typename SotToRos<T>::signalIn_t> signal,
      sigtime_t time);

  template <typename T>
  void add(const std::string& signal, const std::string& topic);

 private:
  static const std::string docstring_;
  ros::NodeHandle& nh_;
  std::map<std::string, bindedSignal_t> bindedSignal_;
  dynamicgraph::SignalTimeDependent<int, sigtime_t> trigger_;
  ros::Duration rate_;
  ros::Time nextPublication_;
  boost::mutex mutex_;
  std::ofstream aofs_;
  struct timespec nextPublicationRT_;
};
}  // end of namespace dynamicgraph.

#include "ros_publish.hxx"
#endif  //! DYNAMIC_GRAPH_ROS_PUBLISH_HH
