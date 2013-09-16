#ifndef DYNAMIC_GRAPH_ROS_EXPORT_HXX
# define DYNAMIC_GRAPH_ROS_EXPORT_HXX
# include <vector>
# include <boost/bind.hpp>
# include <boost/date_time/posix_time/posix_time.hpp>
# include <dynamic-graph/signal-caster.h>
# include <dynamic-graph/signal-cast-helper.h>
# include <std_msgs/Float64.h>
# include "dynamic_graph_bridge/Matrix.h"
# include "dynamic_graph_bridge/Vector.h"
# include "ros_time.hh"

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

  template <typename R>
  void
  RosExport::callbackTimestamp
  (boost::shared_ptr<dynamicgraph::SignalPtr<ptime, int> > signal,
   const R& data)
  {
    ptime time =
      rosTimeToPtime (data->header.stamp);
    signal->setConstant(time);
  }

  namespace internal
  {
    template <typename T>
    struct Add
    {
      void operator () (RosExport& rosExport,
			const std::string& signal,
			const std::string& topic)
      {
	typedef typename SotToRos<T>::sot_t sot_t;
	typedef typename SotToRos<T>::ros_const_ptr_t ros_const_ptr_t;
	typedef typename SotToRos<T>::signalIn_t signal_t;

	// Initialize the bindedSignal object.
	RosExport::bindedSignal_t bindedSignal;

	// Initialize the signal.
	boost::format signalName ("RosExport(%1%)::%2%");
	signalName % rosExport.getName () % signal;

	boost::shared_ptr<signal_t> signal_
	  (new signal_t (0, signalName.str ()));
	SotToRos<T>::setDefault(*signal_);
	bindedSignal.first = signal_;
	rosExport.signalRegistration (*bindedSignal.first);

	// Initialize the subscriber.
	typedef boost::function<void (const ros_const_ptr_t& data)> callback_t;
	callback_t callback = boost::bind
	  (&RosExport::callback<ros_const_ptr_t, sot_t>,
	   &rosExport, signal_, _1);

	bindedSignal.second =
	  boost::make_shared<ros::Subscriber>
	  (rosExport.nh ().subscribe (topic, 1, callback));

	rosExport.bindedSignal ()[signal] = bindedSignal;
      }
    };

    template <typename T>
    struct Add<std::pair<T, dynamicgraph::Vector> >
    {
      void operator () (RosExport& rosExport,
			const std::string& signal,
			const std::string& topic)
      {
	typedef std::pair<T, dynamicgraph::Vector> type_t;

	typedef typename SotToRos<type_t>::sot_t sot_t;
	typedef typename SotToRos<type_t>::ros_const_ptr_t ros_const_ptr_t;
	typedef typename SotToRos<type_t>::signalIn_t signal_t;

	// Initialize the bindedSignal object.
	RosExport::bindedSignal_t bindedSignal;

	// Initialize the signal.
	boost::format signalName ("RosExport(%1%)::%2%");
	signalName % rosExport.getName () % signal;

	boost::shared_ptr<signal_t> signal_
	  (new signal_t (0, signalName.str ()));
	SotToRos<T>::setDefault(*signal_);
	bindedSignal.first = signal_;
	rosExport.signalRegistration (*bindedSignal.first);

	// Initialize the publisher.
	typedef boost::function<void (const ros_const_ptr_t& data)> callback_t;
	callback_t callback = boost::bind
	  (&RosExport::callback<ros_const_ptr_t, sot_t>,
	   &rosExport, signal_, _1);

	bindedSignal.second =
	  boost::make_shared<ros::Subscriber>
	  (rosExport.nh ().subscribe (topic, 1, callback));

	rosExport.bindedSignal ()[signal] = bindedSignal;


	// Timestamp.
	typedef dynamicgraph::SignalPtr<RosExport::ptime, int>
	  signalTimestamp_t;
	std::string signalTimestamp =
	  (boost::format ("%1%%2%") % signal % "Timestamp").str ();

	// Initialize the bindedSignal object.
	RosExport::bindedSignal_t bindedSignalTimestamp;

	// Initialize the signal.
	boost::format signalNameTimestamp ("RosExport(%1%)::%2%");
	signalNameTimestamp % rosExport.name % signalTimestamp;

	boost::shared_ptr<signalTimestamp_t> signalTimestamp_
	  (new signalTimestamp_t (0, signalNameTimestamp.str ()));

	RosExport::ptime zero (rosTimeToPtime (ros::Time (0, 0)));
	signalTimestamp_->setConstant (zero);
	bindedSignalTimestamp.first = signalTimestamp_;
	rosExport.signalRegistration (*bindedSignalTimestamp.first);

	// Initialize the publisher.
	typedef boost::function<void (const ros_const_ptr_t& data)> callback_t;
	callback_t callbackTimestamp = boost::bind
	  (&RosExport::callbackTimestamp<ros_const_ptr_t>,
	   &rosExport, signalTimestamp_, _1);

	bindedSignalTimestamp.second =
	  boost::make_shared<ros::Subscriber>
	  (rosExport.nh ().subscribe (topic, 1, callbackTimestamp));

	rosExport.bindedSignal ()[signalTimestamp] = bindedSignalTimestamp;
      }
    };
  } // end of namespace internal.

  template <typename T>
  void RosExport::add (const std::string& signal, const std::string& topic)
  {
    internal::Add<T> () (*this, signal, topic);
  }
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_EXPORT_HXX
