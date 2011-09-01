#ifndef DYNAMIC_GRAPH_ROS_EXPORT_HXX
# define DYNAMIC_GRAPH_ROS_EXPORT_HXX
# include <vector>
# include <boost/bind.hpp>
# include <jrl/mal/boost.hh>
# include <std_msgs/Float64.h>
# include "dynamic_graph_bridge/Matrix.h"
# include "dynamic_graph_bridge/Vector.h"

#include <iostream>//FIXME:

namespace ml = maal::boost;

namespace dynamicgraph
{
  template <typename R, typename S>
  void
  RosExport::callback
  (boost::shared_ptr<dynamicgraph::SignalPtr<S, int> > signal,
   const R& data)
  {
    typedef S sot_t;
    sot_t value;
    converter (value, data);
    signal->setConstant (value);
  }


  template <typename T>
  void RosExport::add (const std::string& signal, const std::string& topic)
  {
    typedef typename SotToRos<T>::sot_t sot_t;
    typedef typename SotToRos<T>::ros_const_ptr_t ros_const_ptr_t;
    typedef typename SotToRos<T>::signalIn_t signal_t;

    // Initialize the bindedSignal object.
    bindedSignal_t bindedSignal;

    // Initialize the signal.
    boost::format signalName ("RosExport(%1%)::%2%");
    signalName % name % signal;

    boost::shared_ptr<signal_t> signal_ (new signal_t (0, signalName.str ()));
    signal_->setConstant (sot_t ());
    bindedSignal.first = signal_;
    signalRegistration (*bindedSignal.first);

    // Initialize the publisher.
    typedef boost::function<void (const ros_const_ptr_t& data)> callback_t;
    callback_t callback = boost::bind
      (&RosExport::callback<ros_const_ptr_t, sot_t>,
       this, signal_, _1);

    bindedSignal.second =
      boost::make_shared<ros::Subscriber> (nh_.subscribe (topic, 1, callback));

    bindedSignal_[signal] = bindedSignal;
  }
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_EXPORT_HXX
