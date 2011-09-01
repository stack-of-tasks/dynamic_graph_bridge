#ifndef DYNAMIC_GRAPH_ROS_IMPORT_HXX
# define DYNAMIC_GRAPH_ROS_IMPORT_HXX
# include <vector>
# include <std_msgs/Float64.h>
# include "dynamic_graph_bridge/Matrix.h"
# include "dynamic_graph_bridge/Vector.h"

# include "sot_to_ros.hh"

# include <iostream>

namespace dynamicgraph
{
  template <typename T>
  T&
  RosImport::sendData (boost::shared_ptr<ros::Publisher> publisher,
		       T& data, int time)
  {
    typename SotToRos<T>::ros_t result;
    converter (result, data);
    publisher->publish (result);
    return data;
  }

  template <typename T>
  void RosImport::add (const std::string& signal, const std::string& topic)
  {
    typedef typename SotToRos<T>::sot_t sot_t;
    typedef typename SotToRos<T>::ros_t ros_t;
    typedef typename SotToRos<T>::signal_t signal_t;
    typedef typename SotToRos<T>::callback_t callback_t;

    // Initialize the bindedSignal object.
    bindedSignal_t bindedSignal;

    // Initialize the publisher.
    bindedSignal.second =
      boost::make_shared<ros::Publisher> (nh_.advertise<ros_t>(topic, 1));

    // Initialize the signal.
    boost::format signalName ("RosImport(%1%)::%2%");
    signalName % name % signal;

    callback_t signalCallback = boost::bind
      (&RosImport::sendData<sot_t>, this, bindedSignal.second, _1, _2);

    boost::shared_ptr<signal_t> signalPtr = boost::make_shared<signal_t>
      (signalCallback, sotNOSIGNAL, signalName.str ());
    signalPtr->setNeedUpdateFromAllChildren (true);
    bindedSignal.first = signalPtr;
    signalRegistration (*bindedSignal.first);

    bindedSignal_[signal] = bindedSignal;
  }

} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_IMPORT_HXX
