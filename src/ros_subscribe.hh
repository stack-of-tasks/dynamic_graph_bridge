#ifndef DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
#define DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
#include <dynamic-graph/command.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <map>
#include <sot/core/matrix-geometry.hh>

#include "converter.hh"
#include "sot_to_ros.hh"

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
  typedef std::pair<boost::shared_ptr<dynamicgraph::SignalBase<sigtime_t> >,
                    boost::shared_ptr<ros::Subscriber> >
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

  std::map<std::string, bindedSignal_t>& bindedSignal() {
    return bindedSignal_;
  }

  ros::NodeHandle& nh() { return nh_; }

  template <typename R, typename S>
  void callback(boost::shared_ptr<dynamicgraph::SignalPtr<S, sigtime_t> >
                signal, const R& data);

  template <typename R>
  void callbackTimestamp(
      boost::shared_ptr<dynamicgraph::SignalPtr<ptime, sigtime_t> > signal,
      const R& data);

  template <typename T>
  friend class internal::Add;

 private:
  static const std::string docstring_;
  ros::NodeHandle& nh_;
  std::map<std::string, bindedSignal_t> bindedSignal_;
};
}  // end of namespace dynamicgraph.

#include "ros_subscribe.hxx"
#endif  //! DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
