#ifndef DYNAMIC_GRAPH_ROS_PUBLISH_HXX
#define DYNAMIC_GRAPH_ROS_PUBLISH_HXX
#include <vector>
#include <std_msgs/msg/float64.hpp>

#include "dynamic_graph_bridge_msgs/msg/matrix.hpp"
#include "dynamic_graph_bridge_msgs/msg/vector.hpp"

#include "sot_to_ros2.hh"

namespace dynamicgraph {
template <>
inline void RosPublish::sendData<std::pair<sot::MatrixHomogeneous, Vector> >(
    std::shared_ptr<realtime_tools::RealtimePublisher<SotToRos<std::pair<sot::MatrixHomogeneous, Vector> >::ros_t> >
        publisher,
    std::shared_ptr<SotToRos<std::pair<sot::MatrixHomogeneous, Vector> >::signalIn_t> signal, int time) {
  SotToRos<std::pair<sot::MatrixHomogeneous, Vector> >::ros_t result;
  if (publisher->trylock()) {
    publisher->msg_.child_frame_id = "/dynamic_graph/world";
    converter(publisher->msg_, signal->access(time));
    publisher->unlockAndPublish();
  }
}

template <typename T>
void RosPublish::sendData(std::shared_ptr<realtime_tools::RealtimePublisher<typename SotToRos<T>::ros_t> > publisher,
                          std::shared_ptr<typename SotToRos<T>::signalIn_t> signal, int time) {
  typename SotToRos<T>::ros_t result;
  if (publisher->trylock()) {
    converter(publisher->msg_, signal->access(time));
    publisher->unlockAndPublish();
  }
}

template <typename T>
void RosPublish::add(const std::string& signal, const std::string& topic) {
  typedef typename SotToRos<T>::ros_t ros_t;
  typedef typename SotToRos<T>::signalIn_t signal_t;

  // Initialize the bindedSignal object.
  bindedSignal_t bindedSignal;

  // Initialize the publisher.
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  auto pub = nh_->create_publisher<ros_t>(topic, qos);
  
  std::shared_ptr<realtime_tools::RealtimePublisher<ros_t> > pubPtr =
      std::make_shared<realtime_tools::RealtimePublisher<ros_t> >(pub);

  // Initialize the signal.
  std::shared_ptr<signal_t> signalPtr(
      new signal_t(0, MAKE_SIGNAL_STRING(name, true, SotToRos<T>::signalTypeName, signal)));
  boost::get<0>(bindedSignal) = signalPtr;
  SotToRos<T>::setDefault(*signalPtr);
  signalRegistration(*boost::get<0>(bindedSignal));

  // Initialize the callback.
  callback_t callback = std::bind(&RosPublish::sendData<T>, this, pubPtr, signalPtr, std::placeholders::_1);
  boost::get<1>(bindedSignal) = callback;

  bindedSignal_[signal] = bindedSignal;
}

}  // end of namespace dynamicgraph.

#endif  //! DYNAMIC_GRAPH_ROS_PUBLISH_HXX
