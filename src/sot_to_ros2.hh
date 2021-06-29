#ifndef DYNAMIC_GRAPH_ROS_SOT_TO_ROS_HH
#define DYNAMIC_GRAPH_ROS_SOT_TO_ROS_HH
#include <vector>
#include <utility>

#include <boost/format.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include "dynamic_graph_bridge_msgs/msg/matrix.hpp"
#include "dynamic_graph_bridge_msgs/msg/vector.hpp"

#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-ptr.h>
#include <sot/core/matrix-geometry.hh>

#define MAKE_SIGNAL_STRING(NAME, IS_INPUT, OUTPUT_TYPE, SIGNAL_NAME) \
  makeSignalString(typeid(*this).name(), NAME, IS_INPUT, OUTPUT_TYPE, SIGNAL_NAME)

namespace dynamicgraph {

/// \brief Types dedicated to identify pairs of (dg,ros) data.
///
/// They do not contain any data but allow to make the difference
/// between different types with the same structure.
/// For instance a vector of six elements vs a twist coordinate.
namespace specific {
class Vector3 {};
class Twist {};
}  // end of namespace specific.

/// \brief SotToRos trait type.
///
/// This trait provides types associated to a dynamic-graph type:
/// - ROS corresponding type,
/// - signal type / input signal type,
/// - ROS callback type.
template <typename SotType>
class SotToRos;

template <>
struct SotToRos<bool> {
  typedef bool sot_t;
  typedef std_msgs::msg::Bool ros_t;
  typedef std_msgs::msg::Bool::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::Signal<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    s.setConstant(false);
  }

  static void setDefault(sot_t& s) { s = false; }
};

template <>
struct SotToRos<double> {
  typedef double sot_t;
  typedef std_msgs::msg::Float64 ros_t;
  typedef std_msgs::msg::Float64::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::Signal<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    s.setConstant(0.);
  }

  static void setDefault(sot_t& s) { s = 0.; }
};

template <>
struct SotToRos<unsigned int> {
  typedef unsigned int sot_t;
  typedef std_msgs::msg::UInt32 ros_t;
  typedef std_msgs::msg::UInt32::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::Signal<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    s.setConstant(0);
  }

  static void setDefault(sot_t& s) { s = 0; }
};

template <>
struct SotToRos<int> {
  typedef int sot_t;
  typedef std_msgs::msg::Int32 ros_t;
  typedef std_msgs::msg::Int32::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::Signal<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    s.setConstant(0);
  }

  static void setDefault(sot_t& s) { s = 0; }
};

template <>
struct SotToRos<std::string> {
  typedef std::string sot_t;
  typedef std_msgs::msg::String ros_t;
  typedef std_msgs::msg::String::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::Signal<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    s.setConstant("");
  }

  static void setDefault(sot_t& s) { s = std::string(); }
};

template <>
struct SotToRos<Matrix> {
  typedef Matrix sot_t;
  typedef dynamic_graph_bridge_msgs::msg::Matrix ros_t;
  typedef dynamic_graph_bridge_msgs::msg::Matrix::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    Matrix m;
    m.resize(0, 0);
    s.setConstant(m);
  }

  static void setDefault(sot_t& s) { s.resize(0, 0); }
};

template <>
struct SotToRos<Vector> {
  typedef Vector sot_t;
  typedef dynamic_graph_bridge_msgs::msg::Vector ros_t;
  typedef dynamic_graph_bridge_msgs::msg::Vector::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    Vector v;
    v.resize(0);
    s.setConstant(v);
  }

  static void setDefault(sot_t& s) { s.resize(0); }
};

template <>
struct SotToRos<specific::Vector3> {
  typedef Vector sot_t;
  typedef geometry_msgs::msg::Vector3 ros_t;
  typedef geometry_msgs::msg::Vector3::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    Vector v(Vector::Zero(3));
    s.setConstant(v);
  }

  static void setDefault(sot_t& s) { s = Vector::Zero(3); }
};

template <>
struct SotToRos<sot::MatrixHomogeneous> {
  typedef sot::MatrixHomogeneous sot_t;
  typedef geometry_msgs::msg::Transform ros_t;
  typedef geometry_msgs::msg::Transform::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    sot::MatrixHomogeneous m;
    s.setConstant(m);
  }

  static void setDefault(sot_t& s) { s.setIdentity(); }
};

template <>
struct SotToRos<specific::Twist> {
  typedef Vector sot_t;
  typedef geometry_msgs::msg::Twist ros_t;
  typedef geometry_msgs::msg::Twist::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    Vector v(6);
    v.setZero();
    s.setConstant(v);
  }

  static void setDefault(sot_t& s) { s = Vector::Zero(6); }
};

// Stamped vector3
template <>
struct SotToRos<std::pair<specific::Vector3, Vector> > {
  typedef Vector sot_t;
  typedef geometry_msgs::msg::Vector3Stamped ros_t;
  typedef geometry_msgs::msg::Vector3Stamped::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    SotToRos<sot_t>::setDefault(s);
  }
};

// Stamped transformation
template <>
struct SotToRos<std::pair<sot::MatrixHomogeneous, Vector> > {
  typedef sot::MatrixHomogeneous sot_t;
  typedef geometry_msgs::msg::TransformStamped ros_t;
  typedef geometry_msgs::msg::TransformStamped::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    SotToRos<sot_t>::setDefault(s);
  }
};

// Stamped twist
template <>
struct SotToRos<std::pair<specific::Twist, Vector> > {
  typedef Vector sot_t;
  typedef geometry_msgs::msg::TwistStamped ros_t;
  typedef geometry_msgs::msg::TwistStamped::ConstSharedPtr ros_const_ptr_t;
  typedef dynamicgraph::SignalTimeDependent<sot_t, int> signal_t;
  typedef dynamicgraph::SignalPtr<sot_t, int> signalIn_t;
  typedef boost::function<sot_t&(sot_t&, int)> callback_t;

  static const char* signalTypeName;

  template <typename S>
  static void setDefault(S& s) {
    SotToRos<sot_t>::setDefault(s);
  }
};

inline std::string makeSignalString(const std::string& className, const std::string& instanceName, bool isInputSignal,
                                    const std::string& signalType, const std::string& signalName) {
  boost::format fmt("%s(%s)::%s(%s)::%s");
  fmt % className % instanceName % (isInputSignal ? "input" : "output") % signalType % signalName;
  return fmt.str();
}
}  // end of namespace dynamicgraph.

#endif  //! DYNAMIC_GRAPH_ROS_SOT_TO_ROS_HH
