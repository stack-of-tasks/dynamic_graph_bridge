//
// Copyright (c) 2017-2018 CNRS
// Authors: Joseph Mirabel
//
//

#ifndef DYNAMIC_GRAPH_ROS_QUEUED_SUBSCRIBE_HH
#define DYNAMIC_GRAPH_ROS_QUEUED_SUBSCRIBE_HH
#include <dynamic-graph/command.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <map>
#include <sot/core/matrix-geometry.hh>

#include "converter.hh"
#include "sot_to_ros.hh"

namespace dynamicgraph {
class RosQueuedSubscribe;

namespace command {
namespace rosQueuedSubscribe {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

#define ROS_QUEUED_SUBSCRIBE_MAKE_COMMAND(CMD)                     \
  class CMD : public Command {                                     \
   public:                                                         \
    CMD(RosQueuedSubscribe& entity, const std::string& docstring); \
    virtual Value doExecute();                                     \
  }

ROS_QUEUED_SUBSCRIBE_MAKE_COMMAND(Add);

#undef ROS_QUEUED_SUBSCRIBE_MAKE_COMMAND

}  // end of namespace rosQueuedSubscribe.
}  // end of namespace command.

class RosQueuedSubscribe;

namespace internal {
template <typename T>
struct Add;

struct BindedSignalBase {
  typedef boost::shared_ptr<ros::Subscriber> Subscriber_t;

  BindedSignalBase(RosQueuedSubscribe* e) : entity(e) {}
  virtual ~BindedSignalBase() {}

  virtual void clear() = 0;
  virtual std::size_t size() const = 0;

  virtual bool receivedData() const = 0;
  virtual void receivedData(bool) = 0;

  Subscriber_t subscriber;
  RosQueuedSubscribe* entity;
};

template <typename T, int BufferSize>
struct BindedSignal : BindedSignalBase {
  typedef dynamicgraph::Signal<T, int> Signal_t;
  typedef boost::shared_ptr<Signal_t> SignalPtr_t;
  typedef std::vector<T> buffer_t;
  typedef typename buffer_t::size_type size_type;

  BindedSignal(RosQueuedSubscribe* e)
      : BindedSignalBase(e),
        frontIdx(0),
        backIdx(0),
        buffer(BufferSize),
        init(false),
        receivedData_(false) {}
  ~BindedSignal() {
    signal.reset();
    clear();
  }

  /// See comments in reader and writer for details about synchronisation.
  void clear() {
    // synchronize with method writer
    wmutex.lock();
    if (!empty()) {
      if (backIdx == 0)
        last = buffer[BufferSize - 1];
      else
        last = buffer[backIdx - 1];
    }
    // synchronize with method reader
    rmutex.lock();
    frontIdx = backIdx = 0;
    rmutex.unlock();
    wmutex.unlock();
  }

  bool empty() const { return frontIdx == backIdx; }

  bool full() const { return ((backIdx + 1) % BufferSize) == frontIdx; }

  size_type size() const {
    if (frontIdx <= backIdx)
      return backIdx - frontIdx;
    else
      return backIdx + BufferSize - frontIdx;
  }

  /// @brief Returns the value stored in receivedData_ i.e.
  /// whether the signal has received atleast one data point
  /// or not 
  bool receivedData() const {return receivedData_;}

  /// @brief Set the value of data acquisition status of the signal
  /// @param status 
  void receivedData(bool status) {receivedData_ = status;}

  SignalPtr_t signal;
  /// Index of the next value to be read.
  size_type frontIdx;
  /// Index of the slot where to write next value (does not contain valid data).
  size_type backIdx;
  buffer_t buffer;
  boost::mutex wmutex, rmutex;
  T last;
  bool init;

  template <typename R>
  void writer(const R& data);
  T& reader(T& val, int time);

 private:
  /// Indicates whether the signal has received atleast one data point
  bool receivedData_;
};
}  // namespace internal

/// \brief Publish ROS information in the dynamic-graph.
class RosQueuedSubscribe : public dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();
  typedef boost::posix_time::ptime ptime;

 public:
  typedef boost::shared_ptr<internal::BindedSignalBase> bindedSignal_t;

  RosQueuedSubscribe(const std::string& n);
  virtual ~RosQueuedSubscribe();

  virtual std::string getDocString() const;
  void display(std::ostream& os) const;

  void rm(const std::string& signal);
  std::vector<std::string> list();
  std::vector<std::string> listTopics();
  void clear();
  void clearQueue(const std::string& signal);
  void readQueue(int beginReadingAt);
  std::size_t queueSize(const std::string& signal) const;
  bool queueReceivedData(const std::string& signal) const;
  void setQueueReceivedData(const std::string& signal, bool status);

  template <typename T>
  void add(const std::string& type, const std::string& signal,
           const std::string& topic);

  std::map<std::string, bindedSignal_t>& bindedSignal() {
    return bindedSignal_;
  }
  std::map<std::string, std::string>& topics() { return topics_; }

  ros::NodeHandle& nh() { return nh_; }

  template <typename R, typename S>
  void callback(boost::shared_ptr<dynamicgraph::SignalPtr<S, int> > signal,
                const R& data);

  template <typename R>
  void callbackTimestamp(
      boost::shared_ptr<dynamicgraph::SignalPtr<ptime, int> > signal,
      const R& data);

  template <typename T>
  friend class internal::Add;

 private:
  static const std::string docstring_;
  ros::NodeHandle& nh_;
  std::map<std::string, bindedSignal_t> bindedSignal_;
  std::map<std::string, std::string> topics_;

  int readQueue_;
  // Signal<bool, int> readQueue_;

  template <typename T>
  friend class internal::BindedSignal;
};
}  // end of namespace dynamicgraph.

#include "ros_queued_subscribe.hxx"
#endif  //! DYNAMIC_GRAPH_QUEUED_ROS_SUBSCRIBE_HH
