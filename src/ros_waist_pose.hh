#ifndef DYNAMIC_GRAPH_WAIST_POSE_HH
# define DYNAMIC_GRAPH_WAIST_POSE_HH
# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <ros/ros.h>
# include <realtime_tools/realtime_publisher.h>
# include <geometry_msgs/TransformStamped.h>

# include "converter.hh"
# include "sot_to_ros.hh"

namespace dynamicgraph
{
  /// \brief Publish current robot reference position to ROS.
  class RosWaistPose : public dynamicgraph::Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL();
  public:
    /// \brief Vector input signal.
    typedef SignalPtr<ml::Vector, int> signalVectorIn_t;

    static const double ROS_WAIST_POSE_PUBLISHER_RATE = 1. / 100.;

    RosWaistPose (const std::string& n);
    virtual ~RosWaistPose ();

    int& trigger (int&, int);

    geometry_msgs::TransformStamped& waistPose ()
    {
      return waistPose_;
    }
  private:
    ros::NodeHandle& nh_;
    signalVectorIn_t state_;
    realtime_tools::RealtimePublisher<geometry_msgs::TransformStamped> publisher_;
    geometry_msgs::TransformStamped waistPose_;
    dynamicgraph::SignalTimeDependent<int,int> trigger_;
    ros::Duration rate_;
    ros::Time lastPublicated_;
  };
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_WAIST_POSE_HH
