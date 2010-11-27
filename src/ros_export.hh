#ifndef DYNAMIC_GRAPH_ROS_EXPORT_HH
# define DYNAMIC_GRAPH_ROS_EXPORT_HH
# include <iostream>
# include <map>

# include <boost/shared_ptr.hpp>

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <ros/ros.h>

# include "converter.hh"
# include "sot_to_ros.hh"

namespace dynamicgraph
{
  class RosExport : public dynamicgraph::Entity
  {
  public:
    typedef std::pair<boost::shared_ptr<dynamicgraph::SignalBase<int> >,
		      boost::shared_ptr<ros::Subscriber> >
      bindedSignal_t;

    static const std::string CLASS_NAME;

    RosExport (const std::string& n);
    virtual ~RosExport ();

    void display (std::ostream& os) const;
    virtual void
    commandLine (const std::string& cmdLine,
		 std::istringstream& cmdArgs,
		 std::ostream& os);


    virtual const std::string& getClassName ();

  private:
    void add (const std::string& signal, const std::string& topic);
    void rm (const std::string& signal);
    void list ();
    void clear ();

    template <typename T>
    void add (const std::string& signal, const std::string& topic);

    // template <typename R, typename S>
    // void callback
    // (boost::shared_ptr<dynamicgraph::SignalTimeDependent<S, int> > signal,
    //  const R& data);

    template <typename R, typename S>
    void callback
    (boost::shared_ptr<dynamicgraph::SignalTimeDependent<S, int> > signal,
     const R& data);

    ros::NodeHandle nh_;
    std::map<std::string, bindedSignal_t> bindedSignal_;
  };
} // end of namespace dynamicgraph.

# include "ros_export.hxx"
#endif //! DYNAMIC_GRAPH_ROS_EXPORT_HH
