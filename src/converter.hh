#ifndef DYNAMIC_GRAPH_ROS_CONVERTER_HH
#define DYNAMIC_GRAPH_ROS_CONVERTER_HH
#include <stdexcept>
#include <vector>
#include "sot_to_ros2.hh"

#include <boost/static_assert.hpp>
#include <boost/date_time/date.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <std_msgs/msg/header.hpp>

#define SOT_TO_ROS_IMPL(T) \
  template <>              \
  inline void converter(SotToRos<T>::ros_t& dst, const SotToRos<T>::sot_t& src)

#define ROS_TO_SOT_IMPL(T) \
  template <>              \
  inline void converter(SotToRos<T>::sot_t& dst, const SotToRos<T>::ros_t& src)

namespace dynamicgraph {
  inline void makeHeader(std_msgs::msg::Header& header) {
    rclcpp::Clock aClock;
    header.stamp = aClock.now();
    header.frame_id = "/dynamic_graph/world";
  }

/// \brief Handle ROS <-> dynamic-graph conversion.
///
/// Implements all ROS/dynamic-graph conversions required by the
/// bridge.
///
/// Relies on the SotToRos type trait to make sure valid pair of
/// types are used.
template <typename D, typename S>
void converter(D& dst, const S& src);

// Boolean
SOT_TO_ROS_IMPL(bool) { dst.data = src; }

ROS_TO_SOT_IMPL(bool) { dst = src.data; }

// Double
SOT_TO_ROS_IMPL(double) { dst.data = src; }

ROS_TO_SOT_IMPL(double) { dst = src.data; }

// Int
SOT_TO_ROS_IMPL(int) { dst.data = src; }

ROS_TO_SOT_IMPL(int) { dst = src.data; }

// Unsigned
SOT_TO_ROS_IMPL(unsigned int) { dst.data = src; }

ROS_TO_SOT_IMPL(unsigned int) { dst = src.data; }

// String
SOT_TO_ROS_IMPL(std::string) { dst.data = src; }

ROS_TO_SOT_IMPL(std::string) { dst = src.data; }

// Vector
SOT_TO_ROS_IMPL(Vector) {
  dst.data.resize(static_cast<std::size_t>(src.size()));
  for (std::size_t i = 0; i < static_cast<std::size_t>(src.size()); ++i)
    dst.data[i] = src(static_cast<Eigen::Index>(i));
}

ROS_TO_SOT_IMPL(Vector) {
  dst.resize(static_cast<Eigen::Index>(src.data.size()));
  for (unsigned int i = 0; i < src.data.size(); ++i) dst(i) = src.data[i];
}

// Vector3
SOT_TO_ROS_IMPL(specific::Vector3) {
  if (src.size() > 0) {
    dst.x = src(0);
    if (src.size() > 1) {
      dst.y = src(1);
      if (src.size() > 2) dst.z = src(2);
    }
  }
}

ROS_TO_SOT_IMPL(specific::Vector3) {
  dst.resize(3);
  dst(0) = src.x;
  dst(1) = src.y;
  dst(2) = src.z;
}

// Matrix
SOT_TO_ROS_IMPL(Matrix) {
  // TODO: Confirm Ros Matrix Storage order. It changes the RosMatrix to
  // ColMajor.
  dst.width = (unsigned int)src.rows();
  dst.data.resize(static_cast<std::size_t>(src.cols() * src.rows()));
  for (int i = 0; i < src.cols() * src.rows(); ++i)
    dst.data[static_cast<std::size_t>(i)] = src.data()[i];
}

ROS_TO_SOT_IMPL(Matrix) {
  dst.resize(src.width, (unsigned int)src.data.size() / (unsigned int)src.width);
  for (unsigned i = 0; i < src.data.size(); ++i) dst.data()[i] = src.data[i];
}

// Homogeneous matrix.
SOT_TO_ROS_IMPL(sot::MatrixHomogeneous) {
  sot::VectorQuaternion q(src.linear());
  dst.translation.x = src.translation()(0);
  dst.translation.y = src.translation()(1);
  dst.translation.z = src.translation()(2);

  dst.rotation.x = q.x();
  dst.rotation.y = q.y();
  dst.rotation.z = q.z();
  dst.rotation.w = q.w();
}

ROS_TO_SOT_IMPL(sot::MatrixHomogeneous) {
  sot::VectorQuaternion q(src.rotation.w, src.rotation.x, src.rotation.y, src.rotation.z);
  dst.linear() = q.matrix();

  // Copy the translation component.
  dst.translation()(0) = src.translation.x;
  dst.translation()(1) = src.translation.y;
  dst.translation()(2) = src.translation.z;
}

// Twist.
SOT_TO_ROS_IMPL(specific::Twist) {
  if (src.size() != 6) throw std::runtime_error("failed to convert invalid twist");
  dst.linear.x = src(0);
  dst.linear.y = src(1);
  dst.linear.z = src(2);
  dst.angular.x = src(3);
  dst.angular.y = src(4);
  dst.angular.z = src(5);
}

ROS_TO_SOT_IMPL(specific::Twist) {
  dst.resize(6);
  dst(0) = src.linear.x;
  dst(1) = src.linear.y;
  dst(2) = src.linear.z;
  dst(3) = src.angular.x;
  dst(4) = src.angular.y;
  dst(5) = src.angular.z;
}

/// \brief This macro generates a converter for a stamped type from
/// dynamic-graph to ROS.  I.e. A data associated with its
/// timestamp.
#define DG_BRIDGE_TO_ROS_MAKE_STAMPED_IMPL(T, ATTRIBUTE, EXTRA)              \
  template <>                                                                \
  inline void converter(SotToRos<std::pair<T, Vector> >::ros_t& dst,         \
                        const SotToRos<std::pair<T, Vector> >::sot_t& src) { \
    makeHeader(dst.header);                                                  \
    converter<SotToRos<T>::ros_t, SotToRos<T>::sot_t>(dst.ATTRIBUTE, src);   \
    do {                                                                     \
      EXTRA                                                                  \
    } while (0);                                                             \
  }                                                                          \
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

DG_BRIDGE_TO_ROS_MAKE_STAMPED_IMPL(specific::Vector3, vector, ;);
DG_BRIDGE_TO_ROS_MAKE_STAMPED_IMPL(sot::MatrixHomogeneous, transform, dst.child_frame_id = "";);
DG_BRIDGE_TO_ROS_MAKE_STAMPED_IMPL(specific::Twist, twist, ;);

/// \brief This macro generates a converter for a shared pointer on
///        a ROS type to a dynamic-graph type.
///
///        A converter for the underlying type is required.  I.e. to
///        convert a shared_ptr<T> to T', a converter from T to T'
///        is required.
#define DG_BRIDGE_MAKE_SHPTR_IMPL(T)                                                                       \
  template <>                                                                                              \
  inline void converter(SotToRos<T>::sot_t& dst, const std::shared_ptr<SotToRos<T>::ros_t const>& src) { \
    converter<SotToRos<T>::sot_t, SotToRos<T>::ros_t>(dst, *src);                                          \
  }                                                                                                        \
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

DG_BRIDGE_MAKE_SHPTR_IMPL(bool);
DG_BRIDGE_MAKE_SHPTR_IMPL(double);
DG_BRIDGE_MAKE_SHPTR_IMPL(int);
DG_BRIDGE_MAKE_SHPTR_IMPL(unsigned int);
DG_BRIDGE_MAKE_SHPTR_IMPL(std::string);
DG_BRIDGE_MAKE_SHPTR_IMPL(Vector);
DG_BRIDGE_MAKE_SHPTR_IMPL(specific::Vector3);
DG_BRIDGE_MAKE_SHPTR_IMPL(Matrix);
DG_BRIDGE_MAKE_SHPTR_IMPL(sot::MatrixHomogeneous);
DG_BRIDGE_MAKE_SHPTR_IMPL(specific::Twist);

/// \brief This macro generates a converter for a stamped type.
/// I.e. A data associated with its timestamp.
///
/// FIXME: the timestamp is not yet forwarded to the dg signal.
#define DG_BRIDGE_MAKE_STAMPED_IMPL(T, ATTRIBUTE, EXTRA)                     \
  template <>                                                                \
  inline void converter(SotToRos<std::pair<T, Vector> >::sot_t& dst,         \
                        const SotToRos<std::pair<T, Vector> >::ros_t& src) { \
    converter<SotToRos<T>::sot_t, SotToRos<T>::ros_t>(dst, src.ATTRIBUTE);   \
    do {                                                                     \
      EXTRA                                                                  \
    } while (0);                                                             \
  }                                                                          \
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

DG_BRIDGE_MAKE_STAMPED_IMPL(specific::Vector3, vector, ;);
DG_BRIDGE_MAKE_STAMPED_IMPL(sot::MatrixHomogeneous, transform, ;);
DG_BRIDGE_MAKE_STAMPED_IMPL(specific::Twist, twist, ;);

/// \brief This macro generates a converter for a shared pointer on
/// a stamped type.  I.e. A data associated with its timestamp.
///
/// FIXME: the timestamp is not yet forwarded to the dg signal.
#define DG_BRIDGE_MAKE_STAMPED_SHPTR_IMPL(T, ATTRIBUTE, EXTRA)                                        \
  template <>                                                                                         \
  inline void converter(SotToRos<std::pair<T, Vector> >::sot_t& dst,                                  \
                        const std::shared_ptr<SotToRos<std::pair<T, Vector> >::ros_t const>& src) { \
    converter<SotToRos<T>::sot_t, SotToRos<T>::ros_t>(dst, src->ATTRIBUTE);                           \
    do {                                                                                              \
      EXTRA                                                                                           \
    } while (0);                                                                                      \
  }                                                                                                   \
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

DG_BRIDGE_MAKE_STAMPED_SHPTR_IMPL(specific::Vector3, vector, ;);
DG_BRIDGE_MAKE_STAMPED_SHPTR_IMPL(sot::MatrixHomogeneous, transform, ;);
DG_BRIDGE_MAKE_STAMPED_SHPTR_IMPL(specific::Twist, twist, ;);

/// \brief If an impossible/unimplemented conversion is required, fail.
///
/// IMPORTANT, READ ME:
///
/// If the compiler generates an error in the following function,
/// this is /normal/.
///
/// This error means that either you try to use an undefined
/// conversion.  You can either fix your code or provide the wanted
/// conversion by updating this header.
template <typename U, typename V>
inline void converter(U& , V& ) {
  // This will always fail if instantiated.
  BOOST_STATIC_ASSERT(sizeof(U) == 0);
}

typedef boost::posix_time::ptime ptime;
typedef boost::posix_time::seconds seconds;
typedef boost::posix_time::microseconds microseconds;
typedef boost::posix_time::time_duration time_duration;
typedef boost::gregorian::date date;

boost::posix_time::ptime rosTimeToPtime(const rclcpp::Time& rosTime) {
  ptime time(date(1970, 1, 1), seconds(rosTime.nanoseconds() / 1000000)) ;
  return time;
}

rclcpp::Time pTimeToRostime(const boost::posix_time::ptime& time) {
  static ptime timeStart(date(1970, 1, 1));
  time_duration diff = time - timeStart;

  uint32_t sec = (unsigned int)diff.ticks() / (unsigned int)time_duration::rep_type::res_adjust();
  uint32_t nsec = (unsigned int)diff.fractional_seconds();

  return rclcpp::Time(static_cast<int32_t>(sec), nsec);
}
}  // end of namespace dynamicgraph.

#endif  //! DYNAMIC_GRAPH_ROS_CONVERTER_HH
