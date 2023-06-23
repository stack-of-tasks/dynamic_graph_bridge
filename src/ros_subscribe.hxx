#ifndef DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
#define DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-cast-helper.h>
#include <dynamic-graph/signal-caster.h>
#include <std_msgs/Float64.h>

#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <vector>

#include "dynamic_graph_bridge_msgs/Matrix.h"
#include "dynamic_graph_bridge_msgs/Vector.h"
#include "ros_time.hh"

namespace dg = dynamicgraph;

namespace dynamicgraph {
template <typename R, typename S>
void RosSubscribe::callback(
    boost::shared_ptr<dynamicgraph::SignalPtr<S, sigtime_t> > signal,
    const R& data) {
  typedef S sot_t;
  sot_t value;
  converter(value, data);
  signal->setConstant(value);
}

template <typename R>
void RosSubscribe::callbackTimestamp(
    boost::shared_ptr<dynamicgraph::SignalPtr<ptime, sigtime_t> > signal,
    const R& data) {
  ptime time = rosTimeToPtime(data->header.stamp);
  signal->setConstant(time);
}

namespace internal {
template <typename T>
struct Add {
  void operator()(RosSubscribe& RosSubscribe, const std::string& signal,
                  const std::string& topic) {
    typedef typename SotToRos<T>::sot_t sot_t;
    typedef typename SotToRos<T>::ros_const_ptr_t ros_const_ptr_t;
    typedef typename SotToRos<T>::signalIn_t signal_t;

    // Initialize the bindedSignal object.
    RosSubscribe::bindedSignal_t bindedSignal;

    // Initialize the signal.
    boost::format signalName("RosSubscribe(%1%)::%2%");
    signalName % RosSubscribe.getName() % signal;

    boost::shared_ptr<signal_t> signal_(new signal_t(0, signalName.str()));
    SotToRos<T>::setDefault(*signal_);
    bindedSignal.first = signal_;
    RosSubscribe.signalRegistration(*bindedSignal.first);

    // Initialize the subscriber.
    typedef boost::function<void(const ros_const_ptr_t& data)> callback_t;
    callback_t callback =
        boost::bind(&RosSubscribe::callback<ros_const_ptr_t, sot_t>,
                    &RosSubscribe, signal_, _1);

    bindedSignal.second = boost::make_shared<ros::Subscriber>(
        RosSubscribe.nh().subscribe(topic, 1, callback));

    RosSubscribe.bindedSignal()[signal] = bindedSignal;
  }
};

template <typename T>
struct Add<std::pair<T, dg::Vector> > {
  void operator()(RosSubscribe& RosSubscribe, const std::string& signal,
                  const std::string& topic) {
    typedef std::pair<T, dg::Vector> type_t;

    typedef typename SotToRos<type_t>::sot_t sot_t;
    typedef typename SotToRos<type_t>::ros_const_ptr_t ros_const_ptr_t;
    typedef typename SotToRos<type_t>::signalIn_t signal_t;

    // Initialize the bindedSignal object.
    RosSubscribe::bindedSignal_t bindedSignal;

    // Initialize the signal.
    boost::format signalName("RosSubscribe(%1%)::%2%");
    signalName % RosSubscribe.getName() % signal;

    boost::shared_ptr<signal_t> signal_(new signal_t(0, signalName.str()));
    SotToRos<T>::setDefault(*signal_);
    bindedSignal.first = signal_;
    RosSubscribe.signalRegistration(*bindedSignal.first);

    // Initialize the publisher.
    typedef boost::function<void(const ros_const_ptr_t& data)> callback_t;
    callback_t callback =
        boost::bind(&RosSubscribe::callback<ros_const_ptr_t, sot_t>,
                    &RosSubscribe, signal_, _1);

    bindedSignal.second = boost::make_shared<ros::Subscriber>(
        RosSubscribe.nh().subscribe(topic, 1, callback));

    RosSubscribe.bindedSignal()[signal] = bindedSignal;

    // Timestamp.
    typedef dynamicgraph::SignalPtr<RosSubscribe::ptime, sigtime_t>
        signalTimestamp_t;
    std::string signalTimestamp =
        (boost::format("%1%%2%") % signal % "Timestamp").str();

    // Initialize the bindedSignal object.
    RosSubscribe::bindedSignal_t bindedSignalTimestamp;

    // Initialize the signal.
    boost::format signalNameTimestamp("RosSubscribe(%1%)::%2%");
    signalNameTimestamp % RosSubscribe.name % signalTimestamp;

    boost::shared_ptr<signalTimestamp_t> signalTimestamp_(
        new signalTimestamp_t(0, signalNameTimestamp.str()));

    RosSubscribe::ptime zero(rosTimeToPtime(ros::Time(0, 0)));
    signalTimestamp_->setConstant(zero);
    bindedSignalTimestamp.first = signalTimestamp_;
    RosSubscribe.signalRegistration(*bindedSignalTimestamp.first);

    // Initialize the publisher.
    typedef boost::function<void(const ros_const_ptr_t& data)> callback_t;
    callback_t callbackTimestamp =
        boost::bind(&RosSubscribe::callbackTimestamp<ros_const_ptr_t>,
                    &RosSubscribe, signalTimestamp_, _1);

    bindedSignalTimestamp.second = boost::make_shared<ros::Subscriber>(
        RosSubscribe.nh().subscribe(topic, 1, callbackTimestamp));

    RosSubscribe.bindedSignal()[signalTimestamp] = bindedSignalTimestamp;
  }
};
}  // end of namespace internal.

template <typename T>
void RosSubscribe::add(const std::string& signal, const std::string& topic) {
  internal::Add<T>()(*this, signal, topic);
}
}  // end of namespace dynamicgraph.

#endif  //! DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
