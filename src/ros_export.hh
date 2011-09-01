#ifndef DYNAMIC_GRAPH_ROS_EXPORT_HH
# define DYNAMIC_GRAPH_ROS_EXPORT_HH
# include <iostream>
# include <map>

# include <boost/shared_ptr.hpp>

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal-time-dependent.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/command.h>

# include <ros/ros.h>

# include "converter.hh"
# include "sot_to_ros.hh"

namespace dynamicgraph
{
  class RosExport;

  namespace command
  {
    namespace rosExport
    {
      using ::dynamicgraph::command::Command;
      using ::dynamicgraph::command::Value;

# define ROS_EXPORT_MAKE_COMMAND(CMD)			\
      class CMD : public Command			\
      {							\
      public:						\
	CMD (RosExport& entity,				\
	     const std::string& docstring);		\
	virtual Value doExecute ();			\
      }

      ROS_EXPORT_MAKE_COMMAND(Add);
      ROS_EXPORT_MAKE_COMMAND(Clear);
      ROS_EXPORT_MAKE_COMMAND(List);
      ROS_EXPORT_MAKE_COMMAND(Rm);

#undef ROS_EXPORT_MAKE_COMMAND

    } // end of namespace errorEstimator.
  } // end of namespace command.


  class RosExport : public dynamicgraph::Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL();
  public:
    typedef std::pair<boost::shared_ptr<dynamicgraph::SignalBase<int> >,
		      boost::shared_ptr<ros::Subscriber> >
      bindedSignal_t;

    RosExport (const std::string& n);
    virtual ~RosExport ();

    void display (std::ostream& os) const;

    void add (const std::string& signal, const std::string& topic);
    void rm (const std::string& signal);
    void list ();
    void clear ();

    template <typename T>
    void add (const std::string& signal, const std::string& topic);

  private:
    template <typename R, typename S>
    void callback
    (boost::shared_ptr<dynamicgraph::SignalPtr<S, int> > signal,
     const R& data);

    ros::NodeHandle nh_;
    std::map<std::string, bindedSignal_t> bindedSignal_;
    ros::AsyncSpinner spinner_;
  };
} // end of namespace dynamicgraph.

# include "ros_export.hxx"
#endif //! DYNAMIC_GRAPH_ROS_EXPORT_HH
