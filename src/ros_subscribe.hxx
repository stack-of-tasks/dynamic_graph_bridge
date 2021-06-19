#ifndef DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
#define DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
#include <vector>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <dynamic-graph/signal-caster.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-cast-helper.h>
#include <std_msgs/msg/float64.hpp>
#include "dynamic_graph_bridge_msgs/msg/matrix.hpp"
#include "dynamic_graph_bridge_msgs/msg/vector.hpp"
#include "ros_time.hh"

namespace dg = dynamicgraph;

namespace dynamicgraph {
  template <typename RosSubscriptionTypeShrPt, typename SoTType>
void RosSubscribe::callback(std::shared_ptr<dynamicgraph::SignalPtr<SoTType, int> > signal, const RosSubscriptionTypeShrPt data) {
  typedef SoTType sot_t;
  sot_t value;
  converter(value, data);
  signal->setConstant(value);
}

template <typename RosSubscriptionTypeShrPt>
void RosSubscribe::callbackTimestamp(std::shared_ptr<dynamicgraph::SignalPtr<ptime, int> > signal, const RosSubscriptionTypeShrPt data) {
  ptime time = rosTimeToPtime(data->header.stamp);
  signal->setConstant(time);
}

namespace internal {
template <typename SoTSubscriptionType>
struct Add {
  void operator()(dg::RosSubscribe& aRosSubscribe, const std::string& signal, const std::string& topic) {
    typedef typename SotToRos<SoTSubscriptionType>::sot_t sot_t;
    typedef typename SotToRos<SoTSubscriptionType>::ros_const_ptr_t ros_const_ptr_t;
    typedef typename SotToRos<SoTSubscriptionType>::ros_t ros_t;
    typedef typename SotToRos<SoTSubscriptionType>::signalIn_t signal_t;

    // Initialize the bindedSignal object.
    typename dg::RosSubscribe::bindedSignal_t bindedSignal;

    // Initialize the signal.
    boost::format signalName("RosSubscribe(%1%)::%2%");
    signalName % aRosSubscribe.getName() % signal;

    std::shared_ptr<signal_t> signal_(new signal_t(0, signalName.str()));
    SotToRos<SoTSubscriptionType>::setDefault(*signal_);
    bindedSignal.first = signal_;
    aRosSubscribe.signalRegistration(*bindedSignal.first);
	
    // Initialize the subscriber.
    typedef boost::function<void(const ros_const_ptr_t data)> callback_t;
    callback_t callback = boost::bind(&dg::RosSubscribe::callback<ros_const_ptr_t, sot_t>,
                                      &aRosSubscribe, signal_, _1);

    bindedSignal.second = aRosSubscribe.nh().create_subscription<ros_t>(topic, 1, callback);
    
    aRosSubscribe.bindedSignal()[signal] = bindedSignal;
  }
};

template <typename T>
struct Add<std::pair<T, dg::Vector> > {
  void operator()(RosSubscribe& aRosSubscribe, const std::string& signal, const std::string& topic) {
    typedef std::pair<T, dg::Vector> type_t;

    typedef typename SotToRos<type_t>::sot_t sot_t;
    typedef typename SotToRos<type_t>::ros_const_ptr_t ros_const_ptr_t;
    typedef typename SotToRos<type_t>::ros_t ros_t;    
    typedef typename SotToRos<type_t>::signalIn_t signal_t;

    // Initialize the bindedSignal object.
    RosSubscribe::bindedSignal_t bindedSignal;

    // Initialize the signal.
    boost::format signalName("RosSubscribe(%1%)::%2%");
    signalName % aRosSubscribe.getName() % signal;

    std::shared_ptr<signal_t> signal_(new signal_t(0, signalName.str()));
    SotToRos<T>::setDefault(*signal_);
    bindedSignal.first = signal_;
    aRosSubscribe.signalRegistration(*bindedSignal.first);

    // Initialize the publisher.
    typedef boost::function<void(const ros_const_ptr_t& data)> callback_t;
    callback_t callback = boost::bind(&RosSubscribe::callback<ros_const_ptr_t, sot_t>, &aRosSubscribe, signal_, _1);

    bindedSignal.second = aRosSubscribe.nh().create_subscription<ros_t>(topic, 1, callback);

    aRosSubscribe.bindedSignal()[signal] = bindedSignal;

    // Timestamp.
    typedef dynamicgraph::SignalPtr<RosSubscribe::ptime, int> signalTimestamp_t;
    std::string signalTimestamp = (boost::format("%1%%2%") % signal % "Timestamp").str();

    // Initialize the bindedSignal object.
    RosSubscribe::bindedSignal_t bindedSignalTimestamp;

    // Initialize the signal.
    boost::format signalNameTimestamp("RosSubscribe(%1%)::%2%");
    signalNameTimestamp % aRosSubscribe.name % signalTimestamp;

    std::shared_ptr<signalTimestamp_t> signalTimestamp_(new signalTimestamp_t(0, signalNameTimestamp.str()));

    RosSubscribe::ptime zero(rosTimeToPtime(rclcpp::Time(0, 0)));
    signalTimestamp_->setConstant(zero);
    bindedSignalTimestamp.first = signalTimestamp_;
    aRosSubscribe.signalRegistration(*bindedSignalTimestamp.first);

    // Initialize the publisher.
    typedef boost::function<void(const ros_const_ptr_t& data)> callback_t;
    callback_t callbackTimestamp =
        boost::bind(&RosSubscribe::callbackTimestamp<ros_const_ptr_t>, &aRosSubscribe, signalTimestamp_, _1);
  
    bindedSignalTimestamp.second = aRosSubscribe.nh().create_subscription<ros_t>(topic, 1, callbackTimestamp);

    aRosSubscribe.bindedSignal()[signalTimestamp] = bindedSignalTimestamp;
  }
};
}  // end of namespace internal.

template <typename T>
void RosSubscribe::add(const std::string& signal, const std::string& topic) {
  internal::Add<T>()(*this, signal, topic);
}
}  // end of namespace dynamicgraph.

#endif  //! DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
