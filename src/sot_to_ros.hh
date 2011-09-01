#ifndef DYNAMIC_GRAPH_ROS_SOT_TO_ROS_HH
# define DYNAMIC_GRAPH_ROS_SOT_TO_ROS_HH
# include <vector>
# include <jrl/mal/boost.hh>
# include <std_msgs/Float64.h>
# include "dynamic_graph_bridge/Matrix.h"
# include "dynamic_graph_bridge/Vector.h"

namespace dynamicgraph
{
  namespace ml = maal::boost;

  template <typename SotType>
  class SotToRos;

  template <>
  struct SotToRos<double>
  {
    typedef double sot_t;
    typedef std_msgs::Float64 ros_t;
    typedef std_msgs::Float64ConstPtr ros_const_ptr_t;
    typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
    typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
    typedef boost::function<sot_t& (sot_t&, int)> callback_t;
  };

  template <>
  struct SotToRos<ml::Matrix>
  {
    typedef ml::Matrix sot_t;
    typedef dynamic_graph_bridge::Matrix ros_t;
    typedef dynamic_graph_bridge::MatrixConstPtr ros_const_ptr_t;
    typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
    typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
    typedef boost::function<sot_t& (sot_t&, int)> callback_t;
  };

  template <>
  struct SotToRos<ml::Vector>
  {
    typedef ml::Vector sot_t;
    typedef dynamic_graph_bridge::Vector ros_t;
    typedef dynamic_graph_bridge::VectorConstPtr ros_const_ptr_t;
    typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
    typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
    typedef boost::function<sot_t& (sot_t&, int)> callback_t;
  };
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_SOT_TO_ROS_HH
