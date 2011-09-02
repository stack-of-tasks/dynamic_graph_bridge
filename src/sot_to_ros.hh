#ifndef DYNAMIC_GRAPH_ROS_SOT_TO_ROS_HH
# define DYNAMIC_GRAPH_ROS_SOT_TO_ROS_HH
# include <vector>
# include <utility>

# include <jrl/mal/boost.hh>
# include <std_msgs/Float64.h>
# include "dynamic_graph_bridge/Matrix.h"
# include "dynamic_graph_bridge/Vector.h"

# include "geometry_msgs/Transform.h"
# include "geometry_msgs/TransformStamped.h"

# include <dynamic-graph/signal-time-dependent.h>
# include <dynamic-graph/signal-ptr.h>
# include <sot/core/matrix-homogeneous.hh>

namespace dynamicgraph
{
  namespace ml = maal::boost;

  /// \brief SotToRos trait type.
  ///
  /// This trait provides types associated to a dynamic-graph type:
  /// - ROS corresponding type,
  /// - signal type / input signal type,
  /// - ROS callback type.
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

  template <>
  struct SotToRos<sot::MatrixHomogeneous>
  {
    typedef sot::MatrixHomogeneous sot_t;
    typedef geometry_msgs::Transform ros_t;
    typedef geometry_msgs::TransformConstPtr ros_const_ptr_t;
    typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
    typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
    typedef boost::function<sot_t& (sot_t&, int)> callback_t;
  };

  // Stamped transformation
  template <>
  struct SotToRos<std::pair<sot::MatrixHomogeneous, ml::Vector> >
  {
    typedef sot::MatrixHomogeneous sot_t;
    typedef geometry_msgs::TransformStamped ros_t;
    typedef geometry_msgs::TransformStampedConstPtr ros_const_ptr_t;
    typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
    typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
    typedef boost::function<sot_t& (sot_t&, int)> callback_t;
  };

} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_SOT_TO_ROS_HH
