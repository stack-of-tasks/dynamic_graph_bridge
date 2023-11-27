/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2023, LAAS/CNRS
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Define the time stamp signal.
 */

#pragma once

#include <chrono>
#include <ostream>
#include <iostream>
#include <boost/chrono.hpp>

namespace dynamic_graph_bridge {
/** @brief Time stamp type. */
typedef boost::chrono::time_point<boost::chrono::high_resolution_clock> timestamp_t;

}  // namespace dynamic_graph_bridge

//namespace dynamicgraph {
/**
 * @brief out stream the time stamp data.
 *
 * @param os
 * @param time_stamp
 * @return std::ostream&
 *
 * For clang this function needs to be forward declared before the template
 * using it. This is more in accordance to the standard.
 */
std::ostream &operator<<(
    std::ostream &os,
    const dynamic_graph_bridge::timestamp_t &time_stamp) {
  boost::chrono::time_point<boost::chrono::high_resolution_clock,
                          boost::chrono::milliseconds>
      time_stamp_nanosec =
      boost::chrono::time_point_cast<boost::chrono::milliseconds>(time_stamp);
  os << time_stamp_nanosec.time_since_epoch().count();
  return os;
}


//}  // namespace dynamicgraph
