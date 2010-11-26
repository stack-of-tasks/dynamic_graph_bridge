#ifndef DYNAMIC_GRAPH_ROS_EXPORT_HXX
# define DYNAMIC_GRAPH_ROS_EXPORT_HXX
# include <vector>
# include <boost/bind.hpp>
# include <jrl/mal/boost.hh>
# include <std_msgs/Float64.h>
# include "dynamic_graph/Matrix.h"
# include "dynamic_graph/Vector.h"


namespace ml = maal::boost;

namespace dynamicgraph
{
  template <typename T>
  void
  RosExport::callback (boost::shared_ptr<dynamicgraph::SignalBase<int> > signal,
		       const T& data)
  {
    typedef typename SotToRos<T>::sot_t sot_t;
    sot_t value;
    converter (value, data);
    (*signal) (value);
  }

  template <typename T>
  void RosExport::add (const std::string& signal, const std::string& topic)
  {
    typedef typename SotToRos<T>::sot_t sot_t;
    typedef typename SotToRos<T>::ros_t ros_t;
    typedef typename SotToRos<T>::signal_t signal_t;
    typedef typename SotToRos<T>::callback_t callback_t;

    // Initialize the bindedSignal object.
    bindedSignal_t bindedSignal;

    // Initialize the signal.
    boost::format signalName ("RosExport(%1%)::%2%");
    signalName % name % signal;

    bindedSignal.first = boost::make_shared<signal_t>(0, signalName.str ());
    signalRegistration (*bindedSignal.first);

    // Initialize the publisher.
    bindedSignal.second =
      boost::make_shared<ros::Publisher>
      (nh_.subscribe
       (topic, 1, boost::bind (&RosExport::callback<T>,
			       this, bindedSignal.first)));

    bindedSignal_[signal] = bindedSignal;
  }
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_EXPORT_HXX
