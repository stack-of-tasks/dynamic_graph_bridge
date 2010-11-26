#ifndef DYNAMIC_GRAPH_ROS_IMPORT_HH
# define DYNAMIC_GRAPH_ROS_IMPORT_HH
# include <iostream>

# include <boost/shared_ptr.hpp>

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <ros/ros.h>


//FIXME: remove me.
#include <std_msgs/Float64.h>

template <typename SotType>
class SotToRos;

template <>
struct SotToRos<double>
{
  typedef double sot_t;
  typedef std_msgs::Float64 ros_t;
  typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
  typedef boost::function<sot_t& (sot_t&, int)> callback_t;
};

namespace dynamicgraph
{
  template <typename D, typename S>
  void converter (D& dst, const S& src);

  template <>
  void converter (SotToRos<double>::ros_t& dst,
		  const SotToRos<double>::sot_t& src)
  {
    dst.data = src;
  }

  class RosImport : public dynamicgraph::Entity
  {
  public:
    typedef std::pair<boost::shared_ptr<dynamicgraph::SignalBase<int> >,
		      boost::shared_ptr<ros::Publisher> >
      bindedSignal_t;

    static const std::string CLASS_NAME;

    RosImport (const std::string& n);
    virtual ~RosImport ();

    void display (std::ostream& os) const;
    virtual void
    commandLine (const std::string& cmdLine,
		 std::istringstream& cmdArgs,
		 std::ostream& os);


    virtual const std::string& getClassName();

  private:
    void add (const std::string& signal, const std::string& topic);
    void rm (const std::string& signal, const std::string& topic);
    void list ();
    void clear ();

    template <typename T>
    T& sendData (boost::shared_ptr<ros::Publisher> publisher, T& data, int time)
    {
      typename SotToRos<T>::ros_t result;
      converter (result, data);
      publisher->publish (result);
      return data;
    }

    template <typename T>
    void add (const std::string& signal, const std::string& topic)
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

      bindedSignal.first = boost::make_shared<signal_t>
	(signalCallback, sotNOSIGNAL, signalName.str ());
      signalRegistration (*bindedSignal.first);

      bindedSignal_.push_back(bindedSignal);
    }


    ros::NodeHandle nh_;

    std::vector<bindedSignal_t> bindedSignal_;
  };
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_IMPORT_HH
