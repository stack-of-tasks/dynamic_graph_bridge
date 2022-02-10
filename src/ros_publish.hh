#ifndef DYNAMIC_GRAPH_ROS_PUBLISH_HH
#define DYNAMIC_GRAPH_ROS_PUBLISH_HH
#include <map>

#include <boost/tuple/tuple.hpp>
#include <boost/thread/mutex.hpp>

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/command.h>

#include <rclcpp/node.hpp>

#include <realtime_tools/realtime_publisher.h>


#include "converter.hh"
#include "dynamic_graph_bridge/ros2_init.hh"
#include "sot_to_ros2.hh"
#include <fstream>

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
  typedef boost::function<void(int)> callback_t;

  typedef boost::tuple<std::shared_ptr<dynamicgraph::SignalBase<int> >, callback_t> bindedSignal_t;

  static const double ROS_JOINT_STATE_PUBLISHER_RATE;

  RosPublish(const std::string& n);
  virtual ~RosPublish();

  virtual std::string getDocString() const;
  void display(std::ostream& os) const;

  void add(const std::string& signal, const std::string& topic);
  void rm(const std::string& signal);
  std::vector<std::string> list() const;
  void clear();

  int& trigger(int&, int);

  template <typename T>
  void sendData(std::shared_ptr<realtime_tools::RealtimePublisher<typename SotToRos<T>::ros_t> > publisher,
                std::shared_ptr<typename SotToRos<T>::signalIn_t> signal, int time);

  template <typename T>
  void add(const std::string& signal, const std::string& topic);

  void initializeRosContext(dynamicgraph::RosContext::SharedPtr ros_context);

 private:
  static const std::string docstring_;
  rclcpp::Node::SharedPtr nh_;
  std::map<std::string, bindedSignal_t> bindedSignal_;
  dynamicgraph::SignalTimeDependent<int, int> trigger_;
  rclcpp::Duration rate_;
  rclcpp::Time nextPublication_;
  boost::mutex mutex_;
  std::ofstream aofs_;
  struct timespec nextPublicationRT_;
};
}  // end of namespace dynamicgraph.

#include "ros_publish.hxx"
#endif  //! DYNAMIC_GRAPH_ROS_PUBLISH_HH
