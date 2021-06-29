#ifndef DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
#define DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
#include <map>

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/command.h>
#include <sot/core/matrix-geometry.hh>

#include <rclcpp/subscription_base.hpp>
#include <rclcpp/node.hpp>

#include "converter.hh"
#include "sot_to_ros2.hh"

namespace dynamicgraph {

class RosSubscribe;

namespace command {
namespace rosSubscribe {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

#define ROS_SUBSCRIBE_MAKE_COMMAND(CMD)                      \
  class CMD : public Command {                               \
   public:                                                   \
    CMD(RosSubscribe& entity, const std::string& docstring); \
    virtual Value doExecute();                               \
  }

ROS_SUBSCRIBE_MAKE_COMMAND(Add);

#undef ROS_SUBSCRIBE_MAKE_COMMAND

}  // namespace rosSubscribe
}  // end of namespace command.

namespace internal {
template <typename T>
struct Add;
}  // namespace internal

/// \brief Publish ROS information in the dynamic-graph.
class RosSubscribe : public dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();
  typedef boost::posix_time::ptime ptime;

 public:

  typedef std::pair<std::shared_ptr<dynamicgraph::SignalBase<int> >,
                    rclcpp::SubscriptionBase::SharedPtr  >
  bindedSignal_t;

  RosSubscribe(const std::string& n);
  virtual ~RosSubscribe();

  virtual std::string getDocString() const;
  void display(std::ostream& os) const;

  void add(const std::string& signal, const std::string& topic);
  void rm(const std::string& signal);
  std::vector<std::string> list();
  void clear();

  template <typename T>
  void add(const std::string& signal, const std::string& topic);

  std::map<std::string, bindedSignal_t>& bindedSignal() { return bindedSignal_; }

  rclcpp::Node::SharedPtr nh() { return nh_; }

  // RosSubcriptionTypeShrPt Shared pointer to the ros subscription type
  // SoTType: sot type
  template <typename RosSubscriptionTypeShrPt, typename SoTType>
  void callback(std::shared_ptr<dynamicgraph::SignalPtr<SoTType, int> > signal, const RosSubscriptionTypeShrPt data);

  template <typename RosSubscriptionTypeShrPt>
  void callbackTimestamp(std::shared_ptr<dynamicgraph::SignalPtr<ptime, int> > signal, const RosSubscriptionTypeShrPt data);

  template <typename T>
  friend class internal::Add;

 private:
  static const std::string docstring_;
  rclcpp::Node::SharedPtr nh_;
  std::map<std::string, bindedSignal_t> bindedSignal_;
};
}  // end of namespace dynamicgraph.

#include "ros_subscribe.hxx"
#endif  //! DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
