#ifndef DYNAMIC_GRAPH_JOINT_STATE_HH
# define DYNAMIC_GRAPH_JOINT_STATE_HH
# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <ros/ros.h>
# include <realtime_tools/realtime_publisher.h>
# include <sensor_msgs/JointState.h>

# include "converter.hh"
# include "sot_to_ros.hh"

namespace dynamicgraph
{
  /// \brief Publish current robot configuration to ROS.
  class RosJointState : public dynamicgraph::Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL();
  public:
    /// \brief Vector input signal.
    typedef SignalPtr<ml::Vector, int> signalVectorIn_t;

    static const double ROS_JOINT_STATE_PUBLISHER_RATE;

    RosJointState (const std::string& n);
    virtual ~RosJointState ();

    int& trigger (int&, int);

    sensor_msgs::JointState& jointState ()
    {
      return jointState_;
    }
  private:
    ros::NodeHandle& nh_;
    signalVectorIn_t state_;
    realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher_;
    sensor_msgs::JointState jointState_;
    dynamicgraph::SignalTimeDependent<int,int> trigger_;
    ros::Duration rate_;
    ros::Time lastPublicated_;
  };
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_JOINT_STATE_HH
