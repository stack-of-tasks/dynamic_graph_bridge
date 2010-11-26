#ifndef DYNAMIC_GRAPH_ROS_IMPORT_HH
# define DYNAMIC_GRAPH_ROS_IMPORT_HH
# include <iostream>

# include <boost/shared_ptr.hpp>

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <ros/ros.h>

namespace dynamicgraph
{
  template <typename SotType>
  class SotToRos;

  template <typename D, typename S>
  void converter (D& dst, const S& src);

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


    virtual const std::string& getClassName ();

  private:
    void add (const std::string& signal, const std::string& topic);
    void rm (const std::string& signal);
    void list ();
    void clear ();

    template <typename T>
    T& sendData (boost::shared_ptr<ros::Publisher> publisher,
		 T& data, int time);

    template <typename T>
    void add (const std::string& signal, const std::string& topic);

    ros::NodeHandle nh_;
    std::map<std::string, bindedSignal_t> bindedSignal_;
  };
} // end of namespace dynamicgraph.

# include "ros_import.hxx"
#endif //! DYNAMIC_GRAPH_ROS_IMPORT_HH
